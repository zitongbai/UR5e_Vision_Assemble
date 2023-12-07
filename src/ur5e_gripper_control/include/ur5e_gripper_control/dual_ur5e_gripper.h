#ifndef UR5E_GRIPPER_CONTROL_INCLUDE_DUAL_UR5E_GRIPPER_H
#define UR5E_GRIPPER_CONTROL_INCLUDE_DUAL_UR5E_GRIPPER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DualUR5eGripper : public rclcpp::Node{
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;
    
    explicit DualUR5eGripper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    void init();
    
    bool plan_and_execute(const std::vector<double> & left_target_pose, 
        const std::vector<double> & right_target_pose);

    bool left_grasp(double gripper_position);
    bool right_grasp(double gripper_position);

    void get_target_pose_list(std::vector<std::vector<double>> & left_target_pose_list, 
        std::vector<std::vector<double>> & right_target_pose_list);

    void get_cube_pose(
        const std::string & from_frame, 
        const std::string & to_frame, 
        std::vector<double> & cube_pose);

    void go_to_ready_position(bool left);

private:
    std::vector<std::string> left_target_pose_str_list_;
    std::vector<std::string> right_target_pose_str_list_;
    std::vector<std::vector<double>> left_target_pose_list_;
    std::vector<std::vector<double>> right_target_pose_list_;
    void str_list_2_double_list(const std::vector<std::string> & str_list, 
                                std::vector<std::vector<double>> & double_list);

    const std::string left_gripper_action_name_ = "left_gripper_controller/gripper_cmd";
    const std::string right_gripper_action_name_ = "right_gripper_controller/gripper_cmd";
    rclcpp_action::Client<GripperCommand>::SharedPtr left_gripper_action_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr right_gripper_action_client_;
    void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback);
    void result_callback(const GoalHandleGripperCommand::WrappedResult & result);
    rclcpp_action::Client<GripperCommand>::SendGoalOptions send_goal_options_;


    const std::string BOTH_PLANNING_GROUP = "both_manipulators";
    const std::string LEFT_PLANNING_GROUP = "left_ur_manipulator";
    const std::string RIGHT_PLANNING_GROUP = "right_ur_manipulator";

    moveit::planning_interface::MoveGroupInterfacePtr both_move_group_;
    moveit::planning_interface::MoveGroupInterfacePtr left_move_group_;
    moveit::planning_interface::MoveGroupInterfacePtr right_move_group_;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    void get_joint_target_positions(moveit::planning_interface::MoveGroupInterfacePtr move_group, 
                const std::vector<double> & target_pose, 
                const std::string & reference_frame, 
                std::vector<double> & joint_target_positions);

    bool grasp(bool left, double gripper_position);

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif