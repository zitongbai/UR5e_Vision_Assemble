/**
 * @file demo4.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ur5e_gripper_control/dual_ur5e_gripper.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<DualUR5eGripper>(node_options);
  node->init();
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::vector<std::vector<double>>  left_target_pose_list, right_target_pose_list;
  node->get_target_pose_list(left_target_pose_list, right_target_pose_list);

  std::string from_frame_left = "left_base_link";
  std::string from_frame_right = "right_base_link";
  std::vector<std::string> to_frame_list = {
    "cube1", "cube2", "cube3", "cube4", "cube5", "cube6", 
  };

  std::vector<std::vector<double>> left_cube_pose_list, right_cube_pose_list;
  for(size_t i = 0; i < to_frame_list.size(); i++){
    std::vector<double> cube_pose;
    node->get_cube_pose("world", to_frame_list[i], cube_pose);
    double y = cube_pose[1]; // cube position y relative to world frame
    if(y > 0){
      node->get_cube_pose(from_frame_left, to_frame_list[i], cube_pose);
      cube_pose[0] -= 0.025; // modify x a little bit for grasp
      cube_pose[1] = 0.0;
      cube_pose[2] += 0.14; // modify z a little bit for grasp
      cube_pose[3] = 0.0; // roll
      cube_pose[4] = M_PI; // pitch
      cube_pose[5] = 0.0; // yaw
      left_cube_pose_list.push_back(cube_pose);
    } else {
      node->get_cube_pose(from_frame_right, to_frame_list[i], cube_pose);
      cube_pose[0] -= 0.025; // modify x a little bit for grasp
      cube_pose[1] = 0.0; 
      cube_pose[2] += 0.14; // modify z a little bit for grasp
      cube_pose[3] = 0.0; // roll
      cube_pose[4] = M_PI; // pitch
      cube_pose[5] = 0.0; // yaw
      right_cube_pose_list.push_back(cube_pose);
    }
  }
  std::vector<double> do_nothing;
  // there are 3 cubes on both sides
  for(size_t i=0; i<3; i++){
    // first go to the cube position and grasp
    node->plan_and_execute(left_cube_pose_list[i], right_cube_pose_list[i]);
    node->left_grasp(0.38);
    node->right_grasp(0.38);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // then go to the target position and release
    node->plan_and_execute(left_target_pose_list[i], do_nothing);
    node->left_grasp(0.0); // release
    rclcpp::sleep_for(std::chrono::seconds(1));
    node->go_to_ready_position(true);
    node->plan_and_execute(do_nothing, right_target_pose_list[i]);
    node->right_grasp(0.0); // release
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  node->go_to_ready_position(false);

  rclcpp::shutdown();
  return 0;
}