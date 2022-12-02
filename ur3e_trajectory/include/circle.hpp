#include <moveit_wrapper/arm_controller.hpp>
#include <cmath>

geometry_msgs::Pose start_pose;
geometry_msgs::Pose origin;
std::vector<geometry_msgs::Pose> waypoints;


void goPoint(float,float);