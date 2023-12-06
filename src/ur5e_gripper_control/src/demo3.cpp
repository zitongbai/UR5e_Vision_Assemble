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

#include <tf2/LinearMath/Quaternion.h>

#include <control_msgs/action/gripper_command.hpp>


using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("demo3");

void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle){
  if(!goal_handle){
    RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
  } else {
    RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
  }
}
void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback){
  RCLCPP_INFO(LOGGER, "Got Feedback: Current position is %f", feedback->position);
}
void result_callback(const GoalHandleGripperCommand::WrappedResult & result){
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(LOGGER, "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(LOGGER, "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(LOGGER, "Unknown result code");
    return;
  }
  RCLCPP_INFO(LOGGER, "Goal is completed, current position is %f", result.result->position);
}

void str_list_2_double_list(const std::vector<std::string> & str_list, 
                            std::vector<std::vector<double>> & double_list){
  double_list.clear();
  // parse the string
  // each element in the list is a string
  // each string is a list of doubles, with ',' as delimiter
  for (auto & pose_str : str_list){
    std::vector<double> pose;
    std::stringstream ss(pose_str);
    std::string token;
    while (std::getline(ss, token, ',')){
      pose.push_back(std::stod(token));
    }
    double_list.push_back(pose);
  }
}

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto demo3_node = rclcpp::Node::make_shared("demo3", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(demo3_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::vector<std::string> left_target_pose_str_list = 
    demo3_node->get_parameter("left_target_pose_list").as_string_array();
  std::vector<std::string> right_target_pose_str_list = 
    demo3_node->get_parameter("right_target_pose_list").as_string_array();
  std::vector<std::vector<double>> left_target_pose_list, right_target_pose_list;
  str_list_2_double_list(left_target_pose_str_list, left_target_pose_list);
  str_list_2_double_list(right_target_pose_str_list, right_target_pose_list);

  const std::string left_gripper_action_name = "left_gripper_controller/gripper_cmd";
  const std::string right_gripper_action_name = "right_gripper_controller/gripper_cmd";
  auto left_gripper_action_client = rclcpp_action::create_client<GripperCommand>(demo3_node, left_gripper_action_name);
  auto right_gripper_action_client = rclcpp_action::create_client<GripperCommand>(demo3_node, right_gripper_action_name);


  while (!left_gripper_action_client->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO_THROTTLE(LOGGER, *demo3_node->get_clock(), 500, "Waiting for action server to be available...");
  }
  while (!right_gripper_action_client->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO_THROTTLE(LOGGER, *demo3_node->get_clock(), 500, "Waiting for action server to be available...");
  }

  // moveit
  static const std::string BOTH_PLANNING_GROUP = "both_manipulators";
  static const std::string LEFT_PLANNING_GROUP = "left_ur_manipulator";
  static const std::string RIGHT_PLANNING_GROUP = "right_ur_manipulator";

  moveit::planning_interface::MoveGroupInterface both_move_group(demo3_node, BOTH_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface left_move_group(demo3_node, LEFT_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface right_move_group(demo3_node, RIGHT_PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // get basic info
  RCLCPP_INFO(LOGGER, "Reference frame: %s", both_move_group.getPlanningFrame().c_str());

  // add collision object (here is the table) ROS message for the robot to avoid 
  moveit_msgs::msg::CollisionObject collision_table;
  collision_table.header.frame_id = both_move_group.getPlanningFrame();
  collision_table.id = "table";
  // define box to add to the world
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 1.0;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 1.0;
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.5;
  collision_table.primitives.push_back(primitive);
  collision_table.primitive_poses.push_back(box_pose);
  collision_table.operation = collision_table.ADD;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_table);
  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // plan and execute
  moveit::core::RobotStatePtr current_state = both_move_group.getCurrentState(3.0);

  tf2::Quaternion left_target_quat;
  left_target_quat.setRPY(left_target_pose_list[0][3], left_target_pose_list[0][4], left_target_pose_list[0][5]);
  left_target_quat.normalize();
  geometry_msgs::msg::PoseStamped left_target_pose_stamped;
  left_target_pose_stamped.header.frame_id = "left_base_link";
  left_target_pose_stamped.pose.position.x = left_target_pose_list[0][0];
  left_target_pose_stamped.pose.position.y = left_target_pose_list[0][1];
  left_target_pose_stamped.pose.position.z = left_target_pose_list[0][2];
  left_target_pose_stamped.pose.orientation.x = left_target_quat.x();
  left_target_pose_stamped.pose.orientation.y = left_target_quat.y();
  left_target_pose_stamped.pose.orientation.z = left_target_quat.z();
  left_target_pose_stamped.pose.orientation.w = left_target_quat.w();

  tf2::Quaternion right_target_quat;
  right_target_quat.setRPY(right_target_pose_list[0][3], right_target_pose_list[0][4], right_target_pose_list[0][5]);
  right_target_quat.normalize();
  geometry_msgs::msg::PoseStamped right_target_pose_stamped;
  right_target_pose_stamped.header.frame_id = "right_base_link";
  right_target_pose_stamped.pose.position.x = right_target_pose_list[0][0];
  right_target_pose_stamped.pose.position.y = right_target_pose_list[0][1];
  right_target_pose_stamped.pose.position.z = right_target_pose_list[0][2];
  right_target_pose_stamped.pose.orientation.x = right_target_quat.x();
  right_target_pose_stamped.pose.orientation.y = right_target_quat.y();
  right_target_pose_stamped.pose.orientation.z = right_target_quat.z();
  right_target_pose_stamped.pose.orientation.w = right_target_quat.w();

  // Use individual move_groups to calculate desired angles based on pose
  left_move_group.setJointValueTarget(left_target_pose_stamped);
  right_move_group.setJointValueTarget(right_target_pose_stamped);

  std::vector<double> left_joint_angles, right_joint_angles;

  left_move_group.getJointValueTarget(left_joint_angles);
  right_move_group.getJointValueTarget(right_joint_angles);

  std::vector<double> both_joint_angles;
  both_joint_angles.insert(both_joint_angles.end(), left_joint_angles.begin(), left_joint_angles.end());
  both_joint_angles.insert(both_joint_angles.end(), right_joint_angles.begin(), right_joint_angles.end());

  both_move_group.setJointValueTarget(both_joint_angles);

  moveit::planning_interface::MoveGroupInterface::Plan both_plan;
  bool success_plan = (both_move_group.plan(both_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if(success_plan){
    both_move_group.execute(both_plan);
  }

  auto left_gripper_goal_msg = GripperCommand::Goal();
  left_gripper_goal_msg.command.position = 0.38;
  left_gripper_goal_msg.command.max_effort = 1.0;
  if(!left_gripper_action_client->wait_for_action_server(std::chrono::seconds(10))){
    RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
    rclcpp::shutdown();
    return 0;
  }
  RCLCPP_INFO(LOGGER, "Sending gripper goal");
  auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&goal_response_callback, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&feedback_callback, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&result_callback, std::placeholders::_1);
  left_gripper_action_client->async_send_goal(left_gripper_goal_msg, send_goal_options);

  rclcpp::shutdown();
  return 0;
}