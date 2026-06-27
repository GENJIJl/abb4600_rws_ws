#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct JointSafetyLimit
{
  double lower;
  double upper;
  double max_velocity;
};

// ABB IRB4600-60/2.05 的安全范围
// 由 URDF 硬限制向内收缩 0.10 rad 得到
static const std::map<std::string, JointSafetyLimit> kSafeLimits = {
  {"joint_1", {-3.041,  3.041, 3.054}},
  {"joint_2", {-1.470,  2.517, 3.054}},
  {"joint_3", {-3.041,  1.209, 3.054}},
  {"joint_4", {-6.881,  6.881, 4.363}},
  {"joint_5", {-2.081,  1.994, 4.363}},
  {"joint_6", {-6.881,  6.881, 6.283}},
};

template<typename T>
T getParam(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

double durationToSec(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

bool validatePlannedTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const double wrist_singularity_threshold,
  const double max_joint_jump,
  std::string & error_message)
{
  if (trajectory.joint_names.empty()) {
    error_message = "trajectory.joint_names is empty.";
    return false;
  }

  if (trajectory.points.empty()) {
    error_message = "trajectory has no points.";
    return false;
  }

  std::map<std::string, size_t> joint_index;
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    joint_index[trajectory.joint_names[i]] = i;
  }

  for (const auto & item : kSafeLimits) {
    if (joint_index.find(item.first) == joint_index.end()) {
      std::ostringstream oss;
      oss << "missing required joint in trajectory: " << item.first;
      error_message = oss.str();
      return false;
    }
  }

  for (size_t p = 0; p < trajectory.points.size(); ++p) {
    const auto & point = trajectory.points[p];

    if (point.positions.size() != trajectory.joint_names.size()) {
      std::ostringstream oss;
      oss << "point " << p << " position size does not match joint_names size.";
      error_message = oss.str();
      return false;
    }

    for (const auto & item : kSafeLimits) {
      const std::string & joint_name = item.first;
      const auto & limit = item.second;
      const size_t idx = joint_index[joint_name];
      const double q = point.positions[idx];

      if (q < limit.lower || q > limit.upper) {
        std::ostringstream oss;
        oss << "point " << p << ": " << joint_name
            << " = " << q
            << " is outside safe range ["
            << limit.lower << ", " << limit.upper << "] rad.";
        error_message = oss.str();
        return false;
      }

      if (joint_name == "joint_5" && std::abs(q) < wrist_singularity_threshold) {
        std::ostringstream oss;
        oss << "point " << p
            << ": joint_5 = " << q
            << " rad is too close to wrist singularity. "
            << "Required abs(joint_5) >= "
            << wrist_singularity_threshold << " rad.";
        error_message = oss.str();
        return false;
      }
    }

    if (p > 0) {
      const auto & prev = trajectory.points[p - 1];
      const double t_prev = durationToSec(prev.time_from_start);
      const double t_curr = durationToSec(point.time_from_start);
      const double dt = t_curr - t_prev;

      if (dt <= 0.0) {
        std::ostringstream oss;
        oss << "non-positive time step at segment " << p - 1
            << " -> " << p << ", dt = " << dt;
        error_message = oss.str();
        return false;
      }

      for (const auto & item : kSafeLimits) {
        const std::string & joint_name = item.first;
        const auto & limit = item.second;
        const size_t idx = joint_index[joint_name];

        const double q_prev = prev.positions[idx];
        const double q_curr = point.positions[idx];
        const double dq = std::abs(q_curr - q_prev);
        const double estimated_velocity = dq / dt;

        if (dq > max_joint_jump) {
          std::ostringstream oss;
          oss << "segment " << p - 1 << " -> " << p
              << ": " << joint_name
              << " jump = " << dq
              << " rad is too large. max_joint_jump = "
              << max_joint_jump << " rad.";
          error_message = oss.str();
          return false;
        }

        if (estimated_velocity > limit.max_velocity) {
          std::ostringstream oss;
          oss << "segment " << p - 1 << " -> " << p
              << ": " << joint_name
              << " estimated velocity = " << estimated_velocity
              << " rad/s exceeds max velocity = "
              << limit.max_velocity << " rad/s.";
          error_message = oss.str();
          return false;
        }
      }
    }
  }

  return true;
}

void writeReport(
  const std::string & path,
  const geometry_msgs::msg::Pose & current_pose,
  const geometry_msgs::msg::Pose & target_pose,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const bool guard_passed,
  const std::string & guard_message)
{
  std::ofstream file(path);
  if (!file.is_open()) {
    return;
  }

  file << std::fixed << std::setprecision(6);

  file << "Vision target pose planner report\n";
  file << "================================\n\n";

  file << "Current pose:\n";
  file << "  position: "
       << current_pose.position.x << ", "
       << current_pose.position.y << ", "
       << current_pose.position.z << "\n";
  file << "  orientation: "
       << current_pose.orientation.x << ", "
       << current_pose.orientation.y << ", "
       << current_pose.orientation.z << ", "
       << current_pose.orientation.w << "\n\n";

  file << "Target pose:\n";
  file << "  position: "
       << target_pose.position.x << ", "
       << target_pose.position.y << ", "
       << target_pose.position.z << "\n";
  file << "  orientation: "
       << target_pose.orientation.x << ", "
       << target_pose.orientation.y << ", "
       << target_pose.orientation.z << ", "
       << target_pose.orientation.w << "\n\n";

  file << "Trajectory points: " << trajectory.points.size() << "\n";
  file << "Joint names:\n";
  for (const auto & name : trajectory.joint_names) {
    file << "  " << name << "\n";
  }

  file << "\nGuard result: " << (guard_passed ? "PASS" : "REJECT") << "\n";
  file << "Guard message: " << guard_message << "\n\n";

  if (!trajectory.points.empty()) {
    file << "Final joint position:\n";
    const auto & last = trajectory.points.back();
    for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      file << "  " << trajectory.joint_names[i] << ": "
           << last.positions[i] << "\n";
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "vision_target_pose_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  const std::string planning_group =
    getParam<std::string>(node, "planning_group", "manipulator");

  std::string eef_link =
    getParam<std::string>(node, "eef_link", "tool0");

  const bool use_absolute_target =
    getParam<bool>(node, "use_absolute_target", false);

  const bool execute =
    getParam<bool>(node, "execute", false);

  const double velocity_scale =
    getParam<double>(node, "velocity_scale", 0.10);

  const double acceleration_scale =
    getParam<double>(node, "acceleration_scale", 0.10);

  const double planning_time =
    getParam<double>(node, "planning_time", 60.0);

  const int planning_attempts =
    getParam<int>(node, "planning_attempts", 10);

  const double wrist_singularity_threshold =
    getParam<double>(node, "wrist_singularity_threshold", 0.15);

  const double max_joint_jump =
    getParam<double>(node, "max_joint_jump", 0.50);

  const std::string report_path =
    getParam<std::string>(node, "report_path", "/tmp/vision_target_plan_report.txt");

  RCLCPP_INFO(node->get_logger(), "Creating MoveGroupInterface...");
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

  if (eef_link.empty()) {
    eef_link = move_group.getEndEffectorLink();
  }

  std::string reference_frame =
    getParam<std::string>(node, "reference_frame", "");

  if (reference_frame.empty()) {
    reference_frame = move_group.getPlanningFrame();
  }

  move_group.setPoseReferenceFrame(reference_frame);
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(acceleration_scale);
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(planning_attempts);

  RCLCPP_INFO(node->get_logger(), "Planning group: %s", planning_group.c_str());
  RCLCPP_INFO(node->get_logger(), "EEF link: %s", eef_link.c_str());
  RCLCPP_INFO(node->get_logger(), "Reference frame: %s", reference_frame.c_str());
  RCLCPP_INFO(node->get_logger(), "Execute after planning: %s", execute ? "true" : "false");

  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose(eef_link).pose;
  geometry_msgs::msg::Pose target_pose;

  if (use_absolute_target) {
    target_pose.position.x = getParam<double>(node, "target_x", 1.0);
    target_pose.position.y = getParam<double>(node, "target_y", 0.0);
    target_pose.position.z = getParam<double>(node, "target_z", 1.0);

    target_pose.orientation.x = getParam<double>(node, "target_qx", 0.0);
    target_pose.orientation.y = getParam<double>(node, "target_qy", 0.0);
    target_pose.orientation.z = getParam<double>(node, "target_qz", 0.0);
    target_pose.orientation.w = getParam<double>(node, "target_qw", 1.0);
  } else {
    const double offset_x = getParam<double>(node, "offset_x", 0.00);
    const double offset_y = getParam<double>(node, "offset_y", 0.00);
    const double offset_z = getParam<double>(node, "offset_z", 0.05);

    target_pose = current_pose;
    target_pose.position.x += offset_x;
    target_pose.position.y += offset_y;
    target_pose.position.z += offset_z;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Current pose position: x=%.4f, y=%.4f, z=%.4f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "Target pose position: x=%.4f, y=%.4f, z=%.4f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);

  RCLCPP_INFO(node->get_logger(), "Planning to target pose with MoveIt2 IK + OMPL...");

  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose, eef_link);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool plan_success = static_cast<bool>(move_group.plan(plan));

  if (!plan_success) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt2 failed to plan to target pose.");

    writeReport(
      report_path,
      current_pose,
      target_pose,
      plan.trajectory_.joint_trajectory,
      false,
      "MoveIt2 planning failed.");

    RCLCPP_ERROR(node->get_logger(), "Report written to: %s", report_path.c_str());

    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "MoveIt2 planning succeeded. Trajectory points: %zu",
    plan.trajectory_.joint_trajectory.points.size());

  std::string guard_error;
  const bool guard_passed = validatePlannedTrajectory(
    plan.trajectory_.joint_trajectory,
    wrist_singularity_threshold,
    max_joint_jump,
    guard_error);

  if (!guard_passed) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Trajectory rejected by safety validation: %s",
      guard_error.c_str());

    writeReport(
      report_path,
      current_pose,
      target_pose,
      plan.trajectory_.joint_trajectory,
      false,
      guard_error);

    RCLCPP_ERROR(node->get_logger(), "Report written to: %s", report_path.c_str());

    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 3;
  }

  RCLCPP_INFO(node->get_logger(), "Trajectory safety validation passed.");

  writeReport(
    report_path,
    current_pose,
    target_pose,
    plan.trajectory_.joint_trajectory,
    true,
    "Trajectory safety validation passed.");

  RCLCPP_INFO(node->get_logger(), "Report written to: %s", report_path.c_str());

  if (execute) {
    RCLCPP_WARN(node->get_logger(), "Executing trajectory on robot/controller...");
    const bool execute_success = static_cast<bool>(move_group.execute(plan));

    if (!execute_success) {
      RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed.");

      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
      return 4;
    }

    RCLCPP_INFO(node->get_logger(), "Trajectory execution succeeded.");
  } else {
    RCLCPP_INFO(node->get_logger(), "Dry run only. Set execute:=true to execute.");
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
