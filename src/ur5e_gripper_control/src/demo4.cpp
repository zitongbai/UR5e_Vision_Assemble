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
      cube_pose[0] -= 0.02; // modify x a little bit for grasp
      cube_pose[2] += 0.12; // modify z a little bit for grasp
      cube_pose[3] = 0.0; // roll
      cube_pose[4] = M_PI; // pitch
      cube_pose[5] = M_PI_2; // yaw
      left_cube_pose_list.push_back(cube_pose);
    } else {
      node->get_cube_pose(from_frame_right, to_frame_list[i], cube_pose);
      cube_pose[0] -= 0.02; // modify x a little bit for grasp
      cube_pose[2] += 0.12; // modify z a little bit for grasp
      cube_pose[3] = 0.0; // roll
      cube_pose[4] = M_PI; // pitch
      cube_pose[5] = M_PI_2; // yaw
      right_cube_pose_list.push_back(cube_pose);
    }
  }

  node->plan_and_execute(left_cube_pose_list[0], right_cube_pose_list[0]);

  node->left_grasp(0.38);
  node->right_grasp(0.38);
  // sleep for 1 sec
  rclcpp::sleep_for(std::chrono::seconds(1));

  std::vector<double> do_nothing;
  node->plan_and_execute(left_target_pose_list[0], do_nothing);
  node->plan_and_execute(left_cube_pose_list[1], do_nothing);
  node->plan_and_execute(do_nothing, right_target_pose_list[0]);



  // std::vector<std::vector<double>> cube_pose_list;
  // cube_pose_list.resize(to_frame_list.size());

  // for(size_t i = 0; i < to_frame_list.size(); i++){
  //   node->get_cube_pose(from_frame_left, to_frame_list[i], cube_pose_list[i]);
  //   std::cout << to_frame_list[i] << ": ";
  //   for(size_t j = 0; j < cube_pose_list[i].size(); j++){
  //     std::cout << cube_pose_list[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }

  



  // for (size_t i = 0; i < left_target_pose_list.size(); i++){
  //   for (size_t j = 0; j<left_target_pose_list[i].size(); j++){
  //     std::cout << left_target_pose_list[i][j] << " ";
  //   }
  //   std::cout << std::endl;
    
  //   auto begin_left = left_target_pose_list[i].begin();
  //   auto end_left = left_target_pose_list[i].begin()+6;
  //   std::vector<double> left_target_pose(begin_left, end_left);
  //   auto begin_right = right_target_pose_list[i].begin();
  //   auto end_right = right_target_pose_list[i].begin()+6;
  //   std::vector<double> right_target_pose(begin_right, end_right);
  //   node->plan_and_execute(left_target_pose, right_target_pose);
  // }

  rclcpp::shutdown();
  return 0;
}