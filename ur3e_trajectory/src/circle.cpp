#include "../include/circle.hpp"




int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "circle");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;





namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
visual_tools.deleteAllMarkers();


    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");
    std::string reference_frame = "base_link";

    //Write your code for following the circle trajectory here.
    
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    float r = .1; 
    origin.orientation.x = 0;
    origin.orientation.y = -sqrt(.5); 
    origin.orientation.z = sqrt(.5); 
    origin.orientation.w = 0; 
    origin.position.x = 0;
    origin.position.y = -0.3;
    origin.position.z = 1.19;


    bool pose_plan_success;
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, origin, reference_frame, pose_plan);

    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan);
    }


    float tempx=origin.position.x-r,tempz;
    int num_points = 8;
    for (int i=0;i<2;i++){
        for (int k=0;k<num_points;k++){
            tempz = sqrt(r*r-pow((tempx-origin.position.x),2))*pow(-1,i)+origin.position.z;
            // ROS_INFO("k:%d x:%f z:%f",k,tempx,tempz);
            goPoint(tempx,tempz);
            tempx=(2*r)/num_points*pow(-1,i)+tempx;
        }
    }
    goPoint(origin.position.x-r,origin.position.z);

    start_pose = arm_move_group.getCurrentPose().pose;

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);
}

void goPoint(float x,float z){
    geometry_msgs::Pose point = origin;
    point.position.x = x;
    point.position.z = z;
    waypoints.push_back(point);
}

