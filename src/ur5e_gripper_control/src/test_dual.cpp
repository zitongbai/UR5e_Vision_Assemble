#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>


int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    auto const node = rclcpp::Node::make_shared("test_dual_moveit");
    auto const logger = rclcpp::get_logger("test_dual_moveit");

    using moveit::planning_interface::MoveGroupInterface;
    auto left_ur_manipulator_move_group_interface = MoveGroupInterface(node, "left_ur_manipulator");
    auto right_ur_manipulator_move_group_interface = MoveGroupInterface(node, "right_ur_manipulator");
    auto left_gripper_move_group_interface = MoveGroupInterface(node, "left_gripper");
    auto right_gripper_move_group_interface = MoveGroupInterface(node, "right_gripper");

    tf2::Quaternion left_gripper_orientation;
    tf2::Quaternion right_gripper_orientation;
    left_gripper_orientation.setRPY(0, M_PI, M_PI_2);
    right_gripper_orientation.setRPY(0, M_PI, M_PI_2);
    left_gripper_orientation.normalize();
    right_gripper_orientation.normalize();

    auto const left_target_pose = [&left_gripper_orientation]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "left_base_link";
        msg.pose.position.x = 0.3;
        msg.pose.position.y = 0.0;
        msg.pose.position.z = 0.3;
        msg.pose.orientation.w = left_gripper_orientation.w();
        msg.pose.orientation.x = left_gripper_orientation.x();
        msg.pose.orientation.y = left_gripper_orientation.y();
        msg.pose.orientation.z = left_gripper_orientation.z();
        return msg;
    }();
    auto const right_target_pose = [&right_gripper_orientation]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "right_base_link";
        msg.pose.position.x = 0.3;
        msg.pose.position.y = 0.0;
        msg.pose.position.z = 0.3;
        msg.pose.orientation.w = right_gripper_orientation.w();
        msg.pose.orientation.x = right_gripper_orientation.x();
        msg.pose.orientation.y = right_gripper_orientation.y();
        msg.pose.orientation.z = right_gripper_orientation.z();
        return msg;
    }();

    left_ur_manipulator_move_group_interface.setPoseTarget(left_target_pose);
    right_ur_manipulator_move_group_interface.setPoseTarget(right_target_pose);

    auto const [left_success, left_plan] = [&left_ur_manipulator_move_group_interface]{
        MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(left_ur_manipulator_move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();
    auto const [right_success, right_plan] = [&right_ur_manipulator_move_group_interface]{
        MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(right_ur_manipulator_move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if(left_success){
        RCLCPP_INFO(logger, "Left plan success");
        left_ur_manipulator_move_group_interface.execute(left_plan);
    } else {
        RCLCPP_ERROR(logger, "Left plan failed");
    }

    if(right_success){
        RCLCPP_INFO(logger, "Right plan success");
        right_ur_manipulator_move_group_interface.execute(right_plan);
    } else {
        RCLCPP_ERROR(logger, "Right plan failed");
    }

    rclcpp::shutdown();

    return 0;
}