#include "ur5e_gripper_control/dual_ur5e_gripper.h"

DualUR5eGripper::DualUR5eGripper(const rclcpp::NodeOptions & options) 
        : Node("dual_ur5e_gripper", options){
    /* get the target pose list from parameter server */
    left_target_pose_str_list_ = this->get_parameter("left_target_pose_list").as_string_array();
    right_target_pose_str_list_ = this->get_parameter("right_target_pose_list").as_string_array();
    str_list_2_double_list(left_target_pose_str_list_, left_target_pose_list_);
    str_list_2_double_list(right_target_pose_str_list_, right_target_pose_list_);

    /* create action client */
    left_gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(this, left_gripper_action_name_);
    right_gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(this, right_gripper_action_name_);
    /* wait for the action server to be available */
    while (!left_gripper_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Waiting for left gripper action server to be available...");
    }
    while (!right_gripper_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Waiting for right gripper action server to be available...");
    }
    send_goal_options_.goal_response_callback = std::bind(&DualUR5eGripper::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback = std::bind(&DualUR5eGripper::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback = std::bind(&DualUR5eGripper::result_callback, this, std::placeholders::_1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(this->get_logger(), "Create Tf buffer and listener");
}

/**
 * @brief initialize the move group interface
 *  MoveGroupInterface needs the shared point of the Node, but shared_ptr won't be created until after the constructor returns
 *  so we need to create the move group interface in a separate function
 * @ref https://robotics.stackexchange.com/questions/96027/getting-a-nodesharedptr-from-this
 */
void DualUR5eGripper::init(){
  /*create move group interface*/
  both_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), BOTH_PLANNING_GROUP);
  left_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), LEFT_PLANNING_GROUP);
  right_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), RIGHT_PLANNING_GROUP);
  RCLCPP_INFO(this->get_logger(), "Reference frame: %s", both_move_group_->getPlanningFrame().c_str());

  /* add collision objects */
  moveit_msgs::msg::CollisionObject collision_table;
  collision_table.header.frame_id = both_move_group_->getPlanningFrame();
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
  RCLCPP_INFO(this->get_logger(), "Add an object into the world");
  planning_scene_interface_.addCollisionObjects(collision_objects);
}

void DualUR5eGripper::goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle){
  if(!goal_handle){
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}
void DualUR5eGripper::feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback){
  RCLCPP_INFO(this->get_logger(), "Got Feedback: Current position is %f", feedback->position);
}
void DualUR5eGripper::result_callback(const GoalHandleGripperCommand::WrappedResult & result){
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal is completed, current position is %f", result.result->position);
}

void DualUR5eGripper::get_target_pose_list(std::vector<std::vector<double>> & left_target_pose_list, 
        std::vector<std::vector<double>> & right_target_pose_list){
    left_target_pose_list = left_target_pose_list_;
    right_target_pose_list = right_target_pose_list_;
}

/**
 * @brief convert a list of string to a list of double
 * 
 * @param str_list 
 * @param double_list 
 */
void DualUR5eGripper::str_list_2_double_list(const std::vector<std::string> & str_list, 
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


void DualUR5eGripper::get_joint_target_positions(moveit::planning_interface::MoveGroupInterfacePtr move_group, 
            const std::vector<double> & target_pose, 
            const std::string & reference_frame, 
            std::vector<double> & joint_target_positions){
    // assert that the target pose has 6 elements
    assert(target_pose.size() == 6); // x, y, z, roll, pitch, yaw
    // set the target pose
    tf2::Quaternion quat;
    quat.setRPY(target_pose[3], target_pose[4], target_pose[5]);
    quat.normalize();
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.frame_id = reference_frame;
    target_pose_stamped.pose.position.x = target_pose[0];
    target_pose_stamped.pose.position.y = target_pose[1];
    target_pose_stamped.pose.position.z = target_pose[2];
    target_pose_stamped.pose.orientation.x = quat.x();
    target_pose_stamped.pose.orientation.y = quat.y();
    target_pose_stamped.pose.orientation.z = quat.z();
    target_pose_stamped.pose.orientation.w = quat.w();

    // set the target pose of the end effector
    move_group->setJointValueTarget(target_pose_stamped);
    // get the joint space target
    move_group->getJointValueTarget(joint_target_positions);
}



bool DualUR5eGripper::plan_and_execute(const std::vector<double> & left_target_pose, 
        const std::vector<double> & right_target_pose){
    // calculate the target in joint space
    std::vector<double> left_joint_target_positions, right_joint_target_positions;
    if(left_target_pose.size() != 6){ // x, y, z, roll, pitch, yaw
        left_joint_target_positions = left_move_group_->getCurrentJointValues();
    } else {
        get_joint_target_positions(left_move_group_, left_target_pose, "left_base_link", left_joint_target_positions);
    }
    if(right_target_pose.size() != 6){
        right_joint_target_positions = right_move_group_->getCurrentJointValues();
    } else {
        get_joint_target_positions(right_move_group_, right_target_pose, "right_base_link", right_joint_target_positions);
    }
    std::vector<double> both_joint_target_positions;
    both_joint_target_positions.insert(both_joint_target_positions.end(), left_joint_target_positions.begin(), left_joint_target_positions.end());
    both_joint_target_positions.insert(both_joint_target_positions.end(), right_joint_target_positions.begin(), right_joint_target_positions.end());
    
    // set the target positions for both manipulators in joint space
    both_move_group_->setJointValueTarget(both_joint_target_positions);

    // plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan both_plan;
    bool success_plan = (both_move_group_->plan(both_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success_plan){
        both_move_group_->execute(both_plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan");
        return false;
    }
    return true;

}

bool DualUR5eGripper::grasp(bool left, double gripper_position){
    auto gripper_goal_msg = GripperCommand::Goal();
    gripper_goal_msg.command.position = gripper_position;
    gripper_goal_msg.command.max_effort = -1.0; // do not limit the effort

    auto gripper_action_client = left ? left_gripper_action_client_ : right_gripper_action_client_;
    if(!gripper_action_client->wait_for_action_server(std::chrono::seconds(10))){
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Sending gripper goal");
    gripper_action_client->async_send_goal(gripper_goal_msg, send_goal_options_);
    return true;
}

bool DualUR5eGripper::left_grasp(double gripper_position){
    return grasp(true, gripper_position);
}

bool DualUR5eGripper::right_grasp(double gripper_position){
    return grasp(false, gripper_position);
}


void DualUR5eGripper::get_cube_pose(
    const std::string & from_frame, 
    const std::string & to_frame, 
    std::vector<double> & cube_pose){
  cube_pose.clear();
  geometry_msgs::msg::TransformStamped tf_msg;
  try{
    tf_msg = tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    rclcpp::shutdown();
    return;
  }
  cube_pose.push_back(tf_msg.transform.translation.x);
  cube_pose.push_back(tf_msg.transform.translation.y);
  cube_pose.push_back(tf_msg.transform.translation.z);
  cube_pose.push_back(0);
  cube_pose.push_back(0);
  cube_pose.push_back(0);
}

void DualUR5eGripper::go_to_ready_position(bool left){
  moveit::planning_interface::MoveGroupInterfacePtr move_group = left ? left_move_group_ : right_move_group_;
  std::string group_state = left ? "left_ready" : "right_ready";
  move_group->setNamedTarget(group_state);
  move_group->move();
}