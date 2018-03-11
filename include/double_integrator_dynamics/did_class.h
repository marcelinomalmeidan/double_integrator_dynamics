// Mediation layer class

#ifndef DID_CLASS_H_
#define DID_CLASS_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "double_integrator_dynamics/helper.h"
#include "double_integrator_dynamics/rk4.h"

#include "nav_msgs/Odometry.h"

#include "mg_msgs/PVA.h"

struct QuadData {
    std::string name;                        // Unique name for vehicle
    mutable mg_msgs::PVA reference;          // Input reference
    mutable nav_msgs::Odometry vehicle_odom; // Output odometry
    mutable bool is_active;              	 // Flag indicating whether outputs are being published
    mutable bool ref_is_active;              // Flag indicating whether references are being published
    mutable ros::Time last_reference_stamp;  // For heartbeat
    mutable rk4 dynamics_integrator;         // Runge-kutta dynamics integrator
    mutable ros::NodeHandle nh;              // ROS Nodehandle
    mutable ros::Publisher pub_odom;         // Publishes the vehicle odometry

    bool operator<(const QuadData& other) const {
        int compareResult = name.compare(other.name);
        return (compareResult < 0);
    }
};

class DoubleIntegratorDynamics {
 public:
 	std::set<QuadData> quads_;
 	uint n_quads_;
    double k_ = 4.0;          // Proportional gain in the error integrator
    double kd_ = 3.0;         // Derivative gain in the error integrator

    // Constructors
    DoubleIntegratorDynamics();
    DoubleIntegratorDynamics(const double &k,
			                 const double &kd);

    // Methods
    void PrintQuadNames();
    void PrintQuadReferences(const std::string &name);
    void AddQuad(const std::string &quad_name,
                 const std::string &output_topic,
                 ros::NodeHandle *nh);
    void FindQuadIndex(const std::string &name,
    	               std::set<QuadData>::iterator *index);  // Returns -1 if it can't find
    void UpdateQuadReference(const std::string &name, 
                 			 const mg_msgs::PVA &reference);
    void UpdateDIDOutputs(const double &dt);
    void PublishPositions();

 private:

    // ros::NodeHandle nh_;
};

#endif  // DID_CLASS_H_