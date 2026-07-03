#include <memory>
#include <string>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/move_it_error_codes.hpp"

template <typename T>
T get_or_declare_parameter(
  const rclcpp::Node::SharedPtr& node,
  const std::string& name,
  const T& default_value)
{
  if (!node->has_parameter(name))
  {
    node->declare_parameter<T>(name, default_value);
  }

  T value;
  node->get_parameter(name, value);
  return value;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "move_to_target_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  const std::string planning_group =
    get_or_declare_parameter<std::string>(node, "planning_group", "manipulator");

  const double velocity_scale =
    get_or_declare_parameter<double>(node, "velocity_scale", 0.02);

  const double acceleration_scale =
    get_or_declare_parameter<double>(node, "acceleration_scale", 0.02);

  const double planning_time =
    get_or_declare_parameter<double>(node, "planning_time", 60.0);

  const int planning_attempts =
    get_or_declare_parameter<int>(node, "planning_attempts", 20);

  double x = get_or_declare_parameter<double>(node, "x", 1.594);
  double y = get_or_declare_parameter<double>(node, "y", 0.819);
  double z = get_or_declare_parameter<double>(node, "z", 1.050);

  double qx = get_or_declare_parameter<double>(node, "qx", 0.7574);
  double qy = get_or_declare_parameter<double>(node, "qy", 0.1506);
  double qz = get_or_declare_parameter<double>(node, "qz", -0.1492);
  double qw = get_or_declare_parameter<double>(node, "qw", 0.6176);

  const bool execute_motion =
    get_or_declare_parameter<bool>(node, "execute_motion", true);

  RCLCPP_INFO(node->get_logger(), "Planning group: %s", planning_group.c_str());

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

  const std::string planning_frame = move_group.getPlanningFrame();
  const std::string end_effector_link = move_group.getEndEffectorLink();

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", planning_frame.c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", end_effector_link.c_str());

  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(planning_attempts);
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(acceleration_scale);

  move_group.setGoalPositionTolerance(0.005);
  move_group.setGoalOrientationTolerance(0.01);
  move_group.setGoalJointTolerance(0.001);

  move_group.setStartStateToCurrentState();

  double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

  if (norm < 1e-9)
  {
    RCLCPP_ERROR(node->get_logger(), "Quaternion norm is zero. Invalid orientation.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  qx /= norm;
  qy /= norm;
  qz /= norm;
  qw /= norm;

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = planning_frame;
  target_pose.header.stamp = node->now();

  target_pose.pose.position.x = x;
  target_pose.pose.position.y = y;
  target_pose.pose.position.z = z;

  target_pose.pose.orientation.x = qx;
  target_pose.pose.orientation.y = qy;
  target_pose.pose.orientation.z = qz;
  target_pose.pose.orientation.w = qw;

  RCLCPP_INFO(node->get_logger(), "Target pose:");
  RCLCPP_INFO(node->get_logger(), "  position: x=%.4f, y=%.4f, z=%.4f", x, y, z);
  RCLCPP_INFO(node->get_logger(), "  orientation: qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f", qx, qy, qz, qw);

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  RCLCPP_INFO(node->get_logger(), "Start planning...");

  bool success = static_cast<bool>(move_group.plan(plan));

  if (!success)
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed. The target pose may be unreachable or in collision.");
    move_group.clearPoseTargets();

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded.");

  if (execute_motion)
  {
    RCLCPP_INFO(node->get_logger(), "Executing trajectory...");

    auto result = move_group.execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Execution succeeded. Robot reached target pose.");
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Execution failed.");
    }
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "execute_motion=false, only planned, not executed.");
  }

  move_group.clearPoseTargets();

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();

  return 0;
}
