#include "../include/square.hpp"

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
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


    //Write your code for following the square trajectory here.

    // moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    // std::map<std::string, double> joint_targets;
    // joint_targets["elbow_joint"] = 0.94;
    // joint_targets["shoulder_lift_joint"] = -1.13;
    // joint_targets["shoulder_pan_joint"] = 1.13;
    // joint_targets["wrist_1_joint"] = -1.51;
    // joint_targets["wrist_2_joint"] = 2.51;
    // joint_targets["wrist_3_joint"] = 3.39;

    // bool joint_plan_success;
    // joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    // if(joint_plan_success){
    //     ROS_INFO("Moving to joint target");
    //     arm_move_group.execute(joint_plan);
    // }

    // Create instance of pose target plan
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

    geometry_msgs::Pose pose_target;
    pose_target.position.x = 0.25;
    pose_target.position.y = 0.2;
    pose_target.position.z = 1.1;
    pose_target.orientation.x = 0.0;
    pose_target.orientation.y = 0.0;
    pose_target.orientation.z = 0.0;
    pose_target.orientation.w = 1.0;


    bool pose_plan_success;
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target, reference_frame, pose_plan);

    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan);
    }


     
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

    // Get the start Pose
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose point1 = start_pose;
    geometry_msgs::Pose point2 = start_pose;
    geometry_msgs::Pose point3 = start_pose;
    geometry_msgs::Pose point4 = start_pose;

    // TODO: offset z so tilt
    point1.position.x = 0.25;
    point1.position.y = 0.25;
    point1.position.z = 0.1;
    
    point2.position.x = -0.25;
    point2.position.y = 0.25;
    point2.position.z = 0.95;
    
    point3.position.x = -0.25;
    point3.position.y = -0.25;
    point3.position.z = 0.9;
    
    point4.position.x = 0.25;
    point4.position.y = -0.25;
    point4.position.z = 0.95;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(point1);
    waypoints.push_back(point2);
    waypoints.push_back(point3);
    waypoints.push_back(point4);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);
    
}