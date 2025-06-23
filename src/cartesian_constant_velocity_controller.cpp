#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <fstream>
#include <sstream>
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
    MoveGroupInterface move_group_interface(node, "panda_arm");
    move_group_interface.setPlannerId("LIN");
    move_group_interface.setMaxAccelerationScalingFactor(0.1);

    std::vector<geometry_msgs::msg::Pose> target_poses;

    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.18;
    pose1.position.y = -0.2;
    pose1.position.z = 1.0;
    pose1.orientation.w = 1.0;
    target_poses.push_back(pose1);

    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 0.18;
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

    geometry_msgs::msg::Pose pose4;
    pose4.position.x = 0.08;
    pose4.position.y = 0.2;
    pose4.position.z = 1.0;
    pose4.orientation.w = 1.0;
    target_poses.push_back(pose4);

    std::ofstream csv_file("joint_velocities.csv");
    csv_file << "velocity_scaling_factor,time_from_start";
    for (const auto& joint_name : move_group_interface.getJointNames()) {
        csv_file << "," << joint_name;
    }
    csv_file << "\n";

    std::vector<double> velocity_scaling_factors = {0.05, 0.1, 0.15};

    for (double vsf : velocity_scaling_factors)
    {
        RCLCPP_INFO(logger, "Trying velocity scaling factor: %.1f", vsf);
        move_group_interface.setMaxVelocityScalingFactor(vsf);

        for (size_t i = 0; i < target_poses.size(); ++i)
        {
            RCLCPP_INFO(logger, "Planning motion to pose %zu", i + 1);
            move_group_interface.setPoseTarget(target_poses[i]);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = static_cast<bool>(move_group_interface.plan(plan));

            if (success)
            {
                RCLCPP_INFO(logger, "Executing motion...");
                move_group_interface.execute(plan);

                const auto& points = plan.trajectory_.joint_trajectory.points;
                for (const auto& point : points)
                {
                    double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
                    csv_file << vsf << "," << time;
                    for (double vel : point.velocities) {
                        csv_file << "," << vel;
                    }
                    csv_file << "\n";
                }

                rclcpp::sleep_for(std::chrono::seconds(1));
            }
            else
            {
                RCLCPP_ERROR(logger, "Planning failed for pose %zu!", i + 1);
                break;
            }
        }
    }

    csv_file.close();
    RCLCPP_INFO(logger, "Motion sequence and logging completed.");
    rclcpp::shutdown();
    return 0;
}
