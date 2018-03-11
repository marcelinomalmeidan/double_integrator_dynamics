cmake_minimum_required(VERSION 2.8.3)
project(double_integrator_dynamics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  mg_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(double_integrator_dynamics 
               src/double_integrator_dynamics.cpp
               src/rk4.cpp
               src/helper.cpp
               src/threads.cpp
               src/callbacks.cpp
               src/did_class.cpp)
target_link_libraries(double_integrator_dynamics ${catkin_LIBRARIES})
add_dependencies(double_integrator_dynamics ${catkin_EXPORTED_TARGETS})
add_dependencies(double_integrator_dynamics mg_msgs_generate_messages_cpp)
