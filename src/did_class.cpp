

#include "double_integrator_dynamics/did_class.h"

// Constructors
DoubleIntegratorDynamics::DoubleIntegratorDynamics() {

}

DoubleIntegratorDynamics::DoubleIntegratorDynamics(const double &k,
									               const double &kd) {
	k_ = k;
	kd_ = kd;
	n_quads_ = 0;
}

void DoubleIntegratorDynamics::PrintQuadNames() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		std::cout << it->name << std::endl;
	}
	std::cout << std::endl;
}

void DoubleIntegratorDynamics::PrintQuadReferences(const std::string &name) {
	std::set<QuadData>::iterator it;
	this->FindQuadIndex(name, &it);
	if (it != quads_.end()) {
		ROS_INFO("Quad %s references: %f\t%f\t%f", name.c_str(), 
			     it->reference.Pos.x, it->reference.Pos.y, it->reference.Pos.z);
	} else {
		ROS_INFO("[didynamics]: Couldn't print reference: quad name not found");
	}
}

void DoubleIntegratorDynamics::SetNoiseStdDev(const double &std_dev_pos_meas,
	                                     	  const double &std_dev_vel_meas) {
	// std_dev_pos_meas_ = std_dev_pos_meas;
	// std_dev_vel_meas_ = std_dev_vel_meas;
	distribution_pos_ = std::normal_distribution<double>(0.0, std_dev_pos_meas);
	distribution_vel_ = std::normal_distribution<double>(0.0, std_dev_vel_meas);
}

void DoubleIntegratorDynamics::AddQuad(const std::string &quad_name,
							           const std::string &output_topic,
							           ros::NodeHandle *nh) {
	QuadData new_quad;
	new_quad.name = quad_name;

	// Check whether quad name already exists
	if(quads_.find(new_quad) != quads_.end()) {
		ROS_WARN("[didynamics]: Tried to add quad ""%s"": already exists!", quad_name.c_str());
	} else {
		mg_msgs::PVA emptyPVA = helper::GetEmptyPVA();
		new_quad.reference = emptyPVA;
		new_quad.vehicle_odom = helper::GetZeroOdom();
		new_quad.vehicle_odom.child_frame_id = quad_name;
		new_quad.is_active = false;
		new_quad.ref_is_active = false;
		new_quad.last_reference_stamp = ros::Time::now();
		new_quad.dynamics_integrator = rk4(k_, kd_);
		new_quad.nh = *nh;
		new_quad.pub_odom = new_quad.nh.advertise
		                       <nav_msgs::Odometry>(output_topic, 1);
		quads_.insert(new_quad);
		n_quads_ = n_quads_ + 1;
	}
}

void DoubleIntegratorDynamics::FindQuadIndex(const std::string &name,
	               std::set<QuadData>::iterator *index) {
	QuadData quad_with_name;
	quad_with_name.name = name;
	*index = quads_.find(quad_with_name);
}

void DoubleIntegratorDynamics::UpdateQuadReference(const std::string &name, 
             			 						   const mg_msgs::PVA &reference) {
	std::set<QuadData>::iterator it;
	this->FindQuadIndex(name, &it);
	if (it != quads_.end()) {
		it->reference = reference;
		it->last_reference_stamp = ros::Time::now();
		
		// If references are inactive, set them to active
		if(!it->ref_is_active) {
			it->ref_is_active = true;

			// If its not active, its the first time it receives info about a quad
			if(!it->is_active) {
				it->dynamics_integrator.ResetStates(reference);
				it->is_active = true;
			}
		}
	} else {
		ROS_INFO("[mediation layer] Couldn't update reference: quad %s not found", name.c_str());
	}
}

// set all quads as innactive. Used for restarting the simulation
void DoubleIntegratorDynamics::DeactivateQuads() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		mg_msgs::PVA emptyPVA = helper::GetEmptyPVA();
		it->reference = emptyPVA;
		it->vehicle_odom = helper::GetZeroOdom();
		it->vehicle_odom.child_frame_id = it->name;
		it->is_active = false;
		it->ref_is_active = false;
		it->last_reference_stamp = ros::Time::now();
		// it->dynamics_integrator = rk4(k_, kd_);
	}
}

void DoubleIntegratorDynamics::UpdateDIDOutputs(const double &dt) {
	const double gravity = 9.81;
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {

		// Update the error dynamics
		it->dynamics_integrator.UpdateStates(it->reference, dt);

		// Get outputs from the dynamics integrator
		geometry_msgs::Point pos;
		geometry_msgs::Vector3 pos_dot, pos_ddot;
		it->dynamics_integrator.GetPos(&pos);
		it->dynamics_integrator.GetVel(&pos_dot);
		it->dynamics_integrator.GetAcc(&pos_ddot);

		// Get orientation
		const geometry_msgs::Vector3 
			Acc = helper::AddVector3(pos_ddot, 
                  helper::SetVector3(0.0, 0.0, gravity));
		const geometry_msgs::Quaternion quat = 
		          helper::TriadQuat(Acc, it->reference.yaw);

		// Add measurement noise
		pos.x = pos.x + distribution_pos_(generator_);
		pos.y = pos.y + distribution_pos_(generator_);
		pos.z = pos.z + distribution_pos_(generator_);
		pos_dot.x = pos_dot.x + distribution_vel_(generator_);
		pos_dot.y = pos_dot.y + distribution_vel_(generator_);
		pos_dot.z = pos_dot.z + distribution_vel_(generator_);

		// Set odometry info
		it->vehicle_odom.pose.pose.position = pos;
		it->vehicle_odom.pose.pose.orientation = quat;
		it->vehicle_odom.twist.twist.linear = pos_dot;

		// Populate structure for new reference data
		// mg_msgs::PVA ml_reference;
		
		// ml_reference.Pos = pos;
		// ml_reference.Vel = pos_dot;
		// ml_reference.Acc = pos_ddot;

		// // Set new reference data
		// it->ml_reference = ml_reference;
	}
}

void DoubleIntegratorDynamics::PublishPositions() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		if(it->is_active) {
			it->pub_odom.publish(it->vehicle_odom);	
		}
	}
}