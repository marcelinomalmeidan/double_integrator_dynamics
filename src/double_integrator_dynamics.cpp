
#include "double_integrator_dynamics/globals.h"

// Global variables--------------------------
globalVariables globals_;
mutexStruct mutexes_;

bool ResetGame(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res) {
	// Update reference
	pthread_mutex_lock(&mutexes_.m_did_class);
		globals_.obj_did.DeactivateQuads();
	pthread_mutex_unlock(&mutexes_.m_did_class);

	return true;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "double_integrator_dynamics");
	ros::NodeHandle node("~");
	ROS_INFO("[didynamics]: Double Dynamics Integrator started!");

	// Get integration parameters
	double k, kd;
	node.getParam("k", k);
	node.getParam("kd", kd);

	// Get thread parameters
	double integrator_rate;
	node.getParam("integrator_rate", integrator_rate);

	// Get all quad names and colors
	std::vector<std::string> quad_names;
	node.getParam("QuadList", quad_names);

	// Get all topic suffixes
	std::string input_ref_topic, output_odom_topic;
	node.getParam("input_ref_topic", input_ref_topic);
	node.getParam("output_odom_topic", output_odom_topic);

	// Add quads to integrator and create publishers/subscribers for them
	std::vector<ros::Subscriber> subsPVA;
	std::string sub_topic_name;
	for(uint i = 0; i < quad_names.size(); i++) {
		// Add quad
		std::string output_topic, visualization_topic;
		output_topic = "/" + quad_names[i] + output_odom_topic;
		globals_.obj_did.AddQuad(quad_names[i], output_topic, &node);

		// Set subscriber to get references to the quad
		sub_topic_name = "/" + quad_names[i] + input_ref_topic;
		ROS_INFO("[didynamics]: Subscribing to: %s", sub_topic_name.c_str());
		subsPVA.push_back(node.subscribe<mg_msgs::PVA>
			(sub_topic_name, 10, boost::bind(callbacks::PVACallback, _1, quad_names[i])));
	}

	// Services -----------------------------------------------------------------
	ros::ServiceServer reset_srv = node.advertiseService("reset_game", services::ResetGame);

	// Threads ------------------------------------------------------------------
	std::thread h_integrator_thread, h_heartbeat_thread;
	h_integrator_thread = std::thread(threads::IntegrationThread, integrator_rate);
	h_heartbeat_thread = std::thread(threads::HeartbeatThread);


	// ROS loop that starts callbacks/publishers
	ros::spin();

	// Kill mutexes
	mutexes_.destroy();

	return 0;
}