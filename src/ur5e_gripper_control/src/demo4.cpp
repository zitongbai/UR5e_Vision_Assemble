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

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<DualUR5eGripper>(node_options);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::vector<std::vector<double>>  left_target_pose_list, right_target_pose_list;
  node->get_target_pose_list(left_target_pose_list, right_target_pose_list);


  for (size_t i = 0; i < left_target_pose_list.size(); i++){
    for (size_t j = 0; j<left_target_pose_list[i].size(); j++){
      std::cout << left_target_pose_list[i][j] << " ";
    }
    std::cout << std::endl;
    node->plan_and_execute(left_target_pose_list[i], right_target_pose_list[i]);
  }

  rclcpp::shutdown();
  return 0;
}