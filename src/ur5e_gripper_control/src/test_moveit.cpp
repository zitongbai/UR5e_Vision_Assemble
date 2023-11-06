#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    auto const node = rclcpp::Node::make_shared("test_moveit");
    auto const logger = rclcpp::get_logger("test_moveit");

    using moveit::planning_interface::MoveGroupInterface;
    
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.5;
        msg.position.y = 0.1;
        msg.position.z = 0.2;
        msg.orientation.w = 1.0;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    auto const [success, plan] = [&move_group_interface]{
        MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();
    
    if(success){
        RCLCPP_INFO(logger, "Plan success");
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(logger, "Plan failed");
    }


    rclcpp::shutdown();
    return 0;
}