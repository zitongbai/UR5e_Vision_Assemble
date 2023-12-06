cmake_minimum_required(VERSION 3.8)
project(ur5e_gripper_control)

# use cpp 17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

find_package(tf2 REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)

add_executable(test_dual src/test_dual.cpp)
ament_target_dependencies(test_dual rclcpp moveit_ros_planning_interface tf2)

add_executable(move_group src/move_group.cpp)
ament_target_dependencies(move_group 
  rclcpp moveit_ros_planning_interface tf2 
  moveit_core moveit_ros_planning moveit_visual_tools)

add_executable(demo1 src/demo1.cpp)
ament_target_dependencies(demo1 
  rclcpp moveit_ros_planning_interface tf2 
  moveit_core moveit_ros_planning control_msgs)

add_executable(demo2 src/demo2.cpp)
ament_target_dependencies(demo2 
  rclcpp moveit_ros_planning_interface tf2 
  moveit_core moveit_ros_planning control_msgs)

add_executable(demo3 src/demo3.cpp)
ament_target_dependencies(demo3
  rclcpp moveit_ros_planning_interface tf2 
  moveit_core moveit_ros_planning control_msgs)

add_executable(demo4 src/demo4.cpp src/dual_ur5e_gripper.cpp)
ament_target_dependencies(demo4
  rclcpp moveit_ros_planning_interface tf2 
  moveit_core moveit_ros_planning control_msgs)

install(TARGETS test_moveit test_dual move_group demo1 demo2 demo3 demo4
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    launch
    config
  DESTINATION 
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
