#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "cartesian_controller",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("moveit_logger");

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "panda_arm");
    move_group_interface.setPlannerId("LIN");
    move_group_interface.setMaxVelocityScalingFactor(0.2);
    move_group_interface.setMaxAccelerationScalingFactor(0.1);
 
    std::vector<geometry_msgs::msg::Pose> target_poses;
    
    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.28;
    pose1.position.y = -0.2;
    pose1.position.z = 1.0;
    pose1.orientation.w = 1.0;
    target_poses.push_back(pose1);

    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 0.28;
    pose2.position.y = 0.2;
    pose2.position.z = 1.0;
    pose2.orientation.w = 1.0;
    target_poses.push_back(pose2);

    geometry_msgs::msg::Pose pose3;
    pose3.position.x = -0.08;
    pose3.position.y = 0.2;
    pose3.position.z = 1.0;
    pose3.orientation.w = 1.0;
    target_poses.push_back(pose3);

    for (size_t i = 0; i < target_poses.size(); ++i) {
        RCLCPP_INFO(logger, "Moving to pose %zu", i + 1);
        
        move_group_interface.setPoseTarget(target_poses[i]);
        
        auto const [success, plan] = [&move_group_interface]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        
        if(success) {
            RCLCPP_INFO(logger, "Planning successful, executing motion to pose %zu", i + 1);
            move_group_interface.execute(plan);
            
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        } else {
            RCLCPP_ERROR(logger, "Planning failed for pose %zu!", i + 1);
            break;
        }
    }

    RCLCPP_INFO(logger, "Motion sequence completed");
    rclcpp::shutdown();
    return 0;
}