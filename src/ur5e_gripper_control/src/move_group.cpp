#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/LinearMath/Quaternion.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");


int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("dual_ur_move_group", node_options);
  move_group_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "left_ur_manipulator";

  // interface and joint group
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const moveit::core::JointModelGroup * joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // visual tools
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, 
  //     "left_base_link", "left_move_group_viz", move_group.getRobotModel());
  // visual_tools.deleteAllMarkers();
  // // remote control is an introspection tool that allows users to step through a high level script via buttons and keyboard shortcuts in RViz
  // visual_tools.loadRemoteControl(); 

  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.5;
  // visual_tools.publishText(text_pose, "Dual UR5e with gripper", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // get basic information 
  RCLCPP_INFO(LOGGER, "Planning (Reference) frame: %s", move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  tf2::Quaternion target_quat;
  target_quat.setRPY(0, M_PI, M_PI_2);
  target_quat.normalize();
  geometry_msgs::msg::PoseStamped target_pose_1;
  target_pose_1.header.frame_id = "left_base_link";
  target_pose_1.pose.position.x = 0.5;
  target_pose_1.pose.position.y = -0.3;
  target_pose_1.pose.position.z = 0.2;
  target_pose_1.pose.orientation.w = target_quat.w();
  target_pose_1.pose.orientation.x = target_quat.x();
  target_pose_1.pose.orientation.y = target_quat.y();
  target_pose_1.pose.orientation.z = target_quat.z();
  move_group.setPoseTarget(target_pose_1);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if(success){
    RCLCPP_INFO(LOGGER, "Planning to target pose 1 success");
    move_group.move();
  } else {
    RCLCPP_INFO(LOGGER, "Planning to target pose 1 fail");
    rclcpp::shutdown();
    return 0;
  }
  

  rclcpp::shutdown();
  return 0;
}