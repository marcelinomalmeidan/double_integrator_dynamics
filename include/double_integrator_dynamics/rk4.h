

#ifndef RK4_H_
#define RK4_H_

#include <Eigen/Dense>

// ROS message types
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "mg_msgs/PVA.h"
#include "double_integrator_dynamics/helper.h"

class rk4 {  // Runge-kutta for time-invariant systems
 public:

 	// Constructors
 	rk4();
 	rk4(const double &k,
 		const double &kd);

 	// Methods
	void DifferentialEquation(const Eigen::VectorXd &state0,
		  				      const mg_msgs::PVA &Ref,
	                          Eigen::VectorXd *state_dot);
 	void UpdateStates(const mg_msgs::PVA &Ref,
                  	  const double &dt);
    void ResetStates(const mg_msgs::PVA &Ref);
    void ResetStates(const nav_msgs::Odometry &odom);
    void GetPos(Eigen::Vector3d *pos);
    void GetPos(geometry_msgs::Point *pos);
    void GetVel(Eigen::Vector3d *vel);
    void GetVel(geometry_msgs::Vector3 *vel);
    void GetAcc(Eigen::Vector3d *acc);
    void GetAcc(geometry_msgs::Vector3 *acc);

 private:
    Eigen::Vector3d y_;		 // Integrated position
    Eigen::Vector3d y_dot_;	 // Integrated velocity
    Eigen::Vector3d y_ddot_; // Acceleration
    Eigen::Matrix3d K_;
    Eigen::Matrix3d Kd_;
};


#endif  // ML_CLASS_H_