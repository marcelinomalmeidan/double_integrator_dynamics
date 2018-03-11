// Cpp libraries
#include <ros/ros.h>
#include <thread>

// Defined libraries
#include "double_integrator_dynamics/threads.h"
#include "double_integrator_dynamics/callbacks.h"
#include "double_integrator_dynamics/helper.h"
#include "double_integrator_dynamics/did_class.h"

// Structs for global variables
#include "double_integrator_dynamics/structs.h"

// ROS message types
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "mg_msgs/PVA.h"


// Declare global variables
extern globalVariables globals_;
extern mutexStruct mutexes_;