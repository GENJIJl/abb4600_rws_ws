#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
enum class MoveType
{
  kMoveJ,
  kMoveL,
};

struct RapidTarget
{
  std::array<double, 3> position_mm;
  std::array<double, 4> rapid_quaternion;
};

struct RapidCommand
{
  MoveType type;
  std::string target;
  double rapid_speed_mm_s{0.0};
};

struct JointTarget
{
  std::array<double, 6> position_deg;
};

tf2::Quaternion rapidQuaternion(const std::array<double, 4> & q)
{
  tf2::Quaternion quaternion(q[1], q[2], q[3], q[0]);
  quaternion.normalize();
  return quaternion;
}

tf2::Transform rapidTransform(const RapidTarget & target)
{
  return tf2::Transform(
    rapidQuaternion(target.rapid_quaternion),
    tf2::Vector3(
      target.position_mm[0] / 1000.0,
      target.position_mm[1] / 1000.0,
      target.position_mm[2] / 1000.0));
}

const std::map<std::string, RapidTarget> & rapidTargets()
{
  static const std::map<std::string, RapidTarget> targets = {
    {"p10", {{-240.28, -448.49, -956.53}, {0.962361, -0.0531456, -0.000527138, -0.266528}}},
    {"p110", {{-5.99, 131.21, -94.57}, {0.966681, 0.0155295, 0.0145726, -0.255096}}},
    {"p120", {{-40.42, 471.51, 148.10}, {0.976909, 0.0149407, -0.0221023, 0.211982}}},
    {"p130", {{-92.23, 568.87, 100.99}, {0.919239, 0.00627401, 0.00964571, 0.393533}}},
    {"p140", {{-114.40, 629.99, 109.13}, {0.916448, -0.0115674, 0.00387897, 0.399968}}},
    {"p150", {{-71.10, 654.32, 99.07}, {0.91906, -0.0136463, 0.0176852, 0.393483}}},
    {"p160", {{-11.31, 678.01, 18.58}, {0.924838, -0.0190686, 0.0513172, 0.3764}}},
    {"p170", {{-83.64, 670.42, 6.70}, {0.921182, -0.026205, 0.0352454, 0.386646}}},
    {"p180", {{-84.61, 712.64, -2.06}, {0.998298, -0.0283092, -0.0283154, 0.0423924}}},
    {"p190", {{-50.83, 706.84, -17.85}, {0.990146, -0.0209887, -0.0259615, -0.136004}}},
    {"p210", {{-113.14, 718.87, -24.94}, {0.989631, -0.0217464, -0.0444301, -0.134846}}},
    {"p220", {{-64.70, 753.21, -17.15}, {0.965434, -0.0228608, -0.0437023, -0.255941}}},
    {"p230", {{-97.10, 755.19, -10.04}, {0.959959, -0.0329234, -0.0288072, -0.276704}}},
    {"p240", {{-120.89, 762.32, -14.50}, {0.959917, -0.0435689, -0.0192049, -0.276209}}},
    {"p250", {{-174.98, 763.83, -18.74}, {0.962247, -0.0404032, -0.0338304, -0.267027}}},
    {"p260", {{-185.93, 770.58, -24.21}, {0.958088, -0.0410262, -0.038816, -0.28085}}},
    {"p270", {{-178.73, 761.63, -22.50}, {0.928816, -0.0290181, -0.050781, -0.365896}}},
    {"p280", {{-167.10, 749.96, -21.85}, {0.880816, -0.0151546, -0.0601601, -0.469377}}},
    {"p290", {{-158.24, 697.07, -29.33}, {0.846724, -0.00362067, -0.0531305, -0.529361}}},
    {"p300", {{-154.74, 649.85, -32.56}, {0.845601, 0.00393526, -0.0441031, -0.531976}}},
    {"p310", {{-130.16, 639.33, -30.91}, {0.848237, -0.0271749, -0.00993883, -0.528826}}},
    {"p320", {{-150.82, 616.25, -13.26}, {0.850563, -0.0160921, -0.0110442, -0.525511}}},
    {"p330", {{-189.76, 605.25, -13.40}, {0.841633, -0.00691736, -0.0206339, -0.539612}}},
    {"p340", {{-266.14, 599.79, -22.37}, {0.848212, 0.0053971, -0.0394868, -0.528155}}},
    {"p350", {{-292.72, 565.93, -25.97}, {0.897418, 0.0111695, -0.0370571, -0.43948}}},
    {"p360", {{-250.97, 534.00, -32.27}, {0.89417, 0.0115141, -0.0208665, -0.447093}}},
    {"p410", {{-304.79, 533.85, -41.02}, {0.897805, 0.0180078, -0.0362611, -0.438528}}},
    {"p420", {{-243.86, 584.27, -28.58}, {0.894095, -0.000469832, -0.0261542, -0.447114}}},
    {"p430", {{-323.25, 546.57, -25.45}, {0.899235, 0.0201843, -0.042894, -0.43489}}},
    {"p440", {{-346.23, 564.94, -37.75}, {0.900774, 0.0171865, -0.0521139, -0.430808}}},
    {"p450", {{-138.04, 594.34, 11.25}, {0.886922, -0.0106064, 0.00100168, -0.461797}}},
    {"p460", {{-48.07, 603.28, 133.15}, {0.884049, -0.00235037, 0.0178409, -0.467048}}},
    {"p470", {{133.20, -50.14, -304.85}, {0.841863, 0.0788371, 0.205315, -0.492845}}},
    {"p490", {{-553.00, 469.38, -483.06}, {0.867352, -0.122127, 0.252829, 0.410929}}},
    {"p530", {{-639.73, 532.12, -56.21}, {0.858335, 0.105378, 0.0871521, 0.494532}}},
    {"p570", {{-734.00, 564.79, -39.50}, {0.987774, 0.118404, 0.0769107, -0.0660861}}},
    {"p590", {{51.24, 297.72, -614.02}, {0.927652, 0.0177942, 0.292074, -0.232031}}},
    {"p630", {{-596.32, 695.82, -40.28}, {0.86485, 0.0788215, 0.114523, 0.482396}}},
    {"p650", {{-813.47, 460.94, -45.19}, {0.973446, 0.0683803, 0.0716415, 0.206384}}},
    {"p660", {{-1035.36, 352.75, -79.18}, {0.996288, 0.0833622, 0.0108149, 0.0185494}}},
    {"p680", {{-796.93, 511.77, -12.17}, {0.994855, 0.049084, 0.0842449, -0.0275174}}},
    {"p700", {{-436.32, 688.82, 107.36}, {0.979059, 0.00991241, 0.163971, -0.120243}}},
    {"p710", {{-593.45, 637.33, 57.75}, {0.861392, 0.105433, 0.0974993, 0.487218}}},
    {"p720", {{-679.40, 737.92, 73.68}, {0.992206, -0.00555317, 0.0959748, -0.0792812}}},
    {"p730", {{-653.34, 500.56, -42.97}, {0.846632, 0.072262, 0.123352, 0.512617}}},
  };
  return targets;
}

const std::map<std::string, JointTarget> & exportedJointTargets()
{
  static const std::map<std::string, JointTarget> targets = {
    {"p10", {{-86.511894, -27.839258, 74.960037, -270.837494, 90.811653, 236.007568}}},
    {"p110", {{7.700370, 18.806314, 56.061596, -168.514175, 75.559418, 191.455215}}},
    {"p120", {{23.518154, 37.393784, 31.890192, -151.012131, 73.721405, 238.752274}}},
    {"p470", {{9.834903, 7.284501, 66.603394, -146.428406, 62.762802, 151.402481}}},
    {"p490", {{26.597059, -34.802433, 55.158951, -252.994034, -1.855013, 352.469879}}},
    {"p710", {{35.840519, 10.476694, 18.695530, -277.797058, -40.896229, 397.118103}}},
  };
  return targets;
}

double moveJVelocityScaleForTarget(const std::string & target_name, const double default_velocity_scale)
{
  if (target_name == "p110" || target_name == "p470") {
    return default_velocity_scale * 0.50;
  }
  return default_velocity_scale;
}

double moveJAccelerationScaleForTarget(const std::string & target_name, const double default_acceleration_scale)
{
  if (target_name == "p110" || target_name == "p470") {
    return default_acceleration_scale * 0.50;
  }
  return default_acceleration_scale;
}

double degToRad(const double degrees)
{
  return degrees * M_PI / 180.0;
}

std::vector<RapidCommand> guiji1()
{
  return {
    {MoveType::kMoveJ, "p10"}, {MoveType::kMoveJ, "p110"}, {MoveType::kMoveJ, "p120"},
    {MoveType::kMoveL, "p130", 2000.0},
    {MoveType::kMoveL, "p140", 500.0}, {MoveType::kMoveL, "p150", 500.0},
    {MoveType::kMoveL, "p160", 500.0}, {MoveType::kMoveL, "p170", 500.0},
    {MoveType::kMoveL, "p180", 500.0},
    {MoveType::kMoveL, "p190", 100.0}, {MoveType::kMoveL, "p210", 100.0},
    {MoveType::kMoveL, "p220", 100.0}, {MoveType::kMoveL, "p230", 100.0},
    {MoveType::kMoveL, "p240", 100.0}, {MoveType::kMoveL, "p250", 100.0},
    {MoveType::kMoveL, "p260", 100.0}, {MoveType::kMoveL, "p270", 100.0},
    {MoveType::kMoveL, "p280", 100.0}, {MoveType::kMoveL, "p290", 100.0},
    {MoveType::kMoveL, "p300", 100.0}, {MoveType::kMoveL, "p310", 100.0},
    {MoveType::kMoveL, "p320", 100.0}, {MoveType::kMoveL, "p330", 100.0},
    {MoveType::kMoveL, "p340", 100.0}, {MoveType::kMoveL, "p350", 100.0},
    {MoveType::kMoveL, "p360", 100.0},
    {MoveType::kMoveL, "p410", 1000.0}, {MoveType::kMoveL, "p420", 1000.0},
    {MoveType::kMoveL, "p430", 2000.0}, {MoveType::kMoveL, "p440", 2000.0},
    {MoveType::kMoveL, "p450", 2000.0}, {MoveType::kMoveL, "p460", 2000.0},
    {MoveType::kMoveJ, "p470"},
    {MoveType::kMoveJ, "p10"},
  };
}

std::vector<RapidCommand> guiji2()
{
  return {
    {MoveType::kMoveJ, "p10"}, {MoveType::kMoveJ, "p490"}, {MoveType::kMoveJ, "p710"},
    {MoveType::kMoveL, "p630", 3000.0},
    {MoveType::kMoveL, "p530", 1000.0}, {MoveType::kMoveL, "p730", 1000.0},
    {MoveType::kMoveL, "p650", 1000.0}, {MoveType::kMoveL, "p660", 1000.0},
    {MoveType::kMoveL, "p570", 1000.0},
    {MoveType::kMoveL, "p660", 3000.0}, {MoveType::kMoveL, "p680", 3000.0},
    {MoveType::kMoveL, "p720", 3000.0}, {MoveType::kMoveL, "p700", 3000.0},
    {MoveType::kMoveL, "p590", 3000.0}, {MoveType::kMoveJ, "p10"},
  };
}

geometry_msgs::msg::Pose toTool0Pose(const RapidTarget & wobj_target)
{
  const RapidTarget wobj_low_300_1200{
    {1510.44, 75.6853, 430.0},
    {0.707107, 7.45058e-09, 0.707107, 7.45058e-09}};
  const RapidTarget tooldata_hook_m{
    {62.602238005, 108.155222142, 397.729019222},
    {0.926941042, -0.012790803, 0.012046937, -0.374795373}};

  const tf2::Transform base_to_wobj = rapidTransform(wobj_low_300_1200);
  const tf2::Transform wobj_to_tcp = rapidTransform(wobj_target);
  const tf2::Transform tool0_to_tcp = rapidTransform(tooldata_hook_m);
  const tf2::Transform base_to_tool0 = base_to_wobj * wobj_to_tcp * tool0_to_tcp.inverse();

  geometry_msgs::msg::Pose pose;
  pose.position.x = base_to_tool0.getOrigin().x();
  pose.position.y = base_to_tool0.getOrigin().y();
  pose.position.z = base_to_tool0.getOrigin().z();
  pose.orientation = tf2::toMsg(base_to_tool0.getRotation());
  return pose;
}

double getOrDeclareDouble(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const double default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<double>(name, default_value);
  }

  double value = default_value;
  node->get_parameter(name, value);
  return value;
}

int getOrDeclareInt(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const int default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<int>(name, default_value);
  }

  int value = default_value;
  node->get_parameter(name, value);
  return value;
}

bool getOrDeclareBool(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const bool default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<bool>(name, default_value);
  }

  bool value = default_value;
  node->get_parameter(name, value);
  return value;
}

std::string degString(const double radians)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1) << radians * 180.0 / M_PI;
  return stream.str();
}

const std::map<std::string, std::pair<double, double>> & jointPositionLimits()
{
  static const std::map<std::string, std::pair<double, double>> limits = {
    {"joint_1", {-3.141, 3.141}},
    {"joint_2", {-1.570, 2.617}},
    {"joint_3", {-3.141, 1.309}},
    {"joint_4", {-6.981, 6.981}},
    {"joint_5", {-2.181, 2.094}},
    {"joint_6", {-6.981, 6.981}},
  };
  return limits;
}

bool inspectTrajectory(
  const rclcpp::Logger & logger,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::string & label,
  const double max_joint_step_deg)
{
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.points.empty()) {
    RCLCPP_ERROR(logger, "%s 轨迹为空。", label.c_str());
    return false;
  }

  std::vector<double> min_positions(joint_trajectory.joint_names.size(), std::numeric_limits<double>::infinity());
  std::vector<double> max_positions(joint_trajectory.joint_names.size(), -std::numeric_limits<double>::infinity());
  std::vector<double> max_step(joint_trajectory.joint_names.size(), 0.0);
  std::vector<double> max_velocity(joint_trajectory.joint_names.size(), 0.0);
  bool ok = true;

  for (std::size_t point_index = 0; point_index < joint_trajectory.points.size(); ++point_index) {
    const auto & point = joint_trajectory.points[point_index];
    for (std::size_t joint_index = 0; joint_index < joint_trajectory.joint_names.size(); ++joint_index) {
      const auto & joint_name = joint_trajectory.joint_names[joint_index];
      if (joint_index >= point.positions.size()) {
        RCLCPP_ERROR(logger, "%s 第 %zu 个轨迹点缺少 %s 位置。", label.c_str(), point_index, joint_name.c_str());
        return false;
      }

      const double position = point.positions[joint_index];
      min_positions[joint_index] = std::min(min_positions[joint_index], position);
      max_positions[joint_index] = std::max(max_positions[joint_index], position);

      const auto limit_it = jointPositionLimits().find(joint_name);
      if (limit_it != jointPositionLimits().end()) {
        constexpr double kLimitMargin = 1.0e-3;
        constexpr double kNearLimitMargin = 5.0 * M_PI / 180.0;
        if (position < limit_it->second.first - kLimitMargin ||
          position > limit_it->second.second + kLimitMargin)
        {
          RCLCPP_ERROR(
            logger,
            "%s %s 超出 URDF 关节范围：%.1f deg，不在 [%.1f, %.1f] deg。",
            label.c_str(), joint_name.c_str(),
            position * 180.0 / M_PI,
            limit_it->second.first * 180.0 / M_PI,
            limit_it->second.second * 180.0 / M_PI);
          ok = false;
        } else if (position < limit_it->second.first + kNearLimitMargin ||
          position > limit_it->second.second - kNearLimitMargin)
        {
          RCLCPP_WARN(
            logger,
            "%s %s 接近 URDF 关节边界：%.1f deg，边界 [%.1f, %.1f] deg。EGM 下这类贴边轨迹容易触发 50459。",
            label.c_str(), joint_name.c_str(),
            position * 180.0 / M_PI,
            limit_it->second.first * 180.0 / M_PI,
            limit_it->second.second * 180.0 / M_PI);
        }
      }

      if (point_index > 0) {
        const auto & previous = joint_trajectory.points[point_index - 1];
        if (joint_index < previous.positions.size()) {
          max_step[joint_index] =
            std::max(max_step[joint_index], std::abs(position - previous.positions[joint_index]));
        }
      }

      if (joint_index < point.velocities.size()) {
        max_velocity[joint_index] = std::max(max_velocity[joint_index], std::abs(point.velocities[joint_index]));
      }
    }
  }

  const double max_joint_step = max_joint_step_deg * M_PI / 180.0;
  const auto & final_time = joint_trajectory.points.back().time_from_start;
  const double total_duration =
    static_cast<double>(final_time.sec) + static_cast<double>(final_time.nanosec) * 1.0e-9;
  RCLCPP_INFO(
    logger,
    "%s trajectory points=%zu, duration=%.3f s",
    label.c_str(), joint_trajectory.points.size(), total_duration);

  for (std::size_t joint_index = 0; joint_index < joint_trajectory.joint_names.size(); ++joint_index) {
    const auto & joint_name = joint_trajectory.joint_names[joint_index];
    RCLCPP_INFO(
      logger,
      "%s %s: range=[%s, %s] deg, max_step=%s deg, max_velocity=%s deg/s",
      label.c_str(), joint_name.c_str(),
      degString(min_positions[joint_index]).c_str(),
      degString(max_positions[joint_index]).c_str(),
      degString(max_step[joint_index]).c_str(),
      degString(max_velocity[joint_index]).c_str());

    if (max_step[joint_index] > max_joint_step) {
      RCLCPP_ERROR(
        logger,
        "%s %s 相邻轨迹点最大跳变 %s deg，超过阈值 %.1f deg。这通常意味着 IK 构型跳变或轨迹点过稀。",
        label.c_str(), joint_name.c_str(), degString(max_step[joint_index]).c_str(), max_joint_step_deg);
      ok = false;
    }
  }

  return ok;
}

void scaleDuration(builtin_interfaces::msg::Duration & duration, const double scale)
{
  const int64_t total_ns =
    static_cast<int64_t>(duration.sec) * 1000000000LL + static_cast<int64_t>(duration.nanosec);
  const int64_t scaled_ns = static_cast<int64_t>(static_cast<double>(total_ns) * scale);
  duration.sec = static_cast<int32_t>(scaled_ns / 1000000000LL);
  duration.nanosec = static_cast<uint32_t>(scaled_ns % 1000000000LL);
}

void scaleTrajectoryTiming(moveit_msgs::msg::RobotTrajectory & trajectory, const double duration_scale)
{
  if (duration_scale <= 1.0) {
    return;
  }

  for (auto & point : trajectory.joint_trajectory.points) {
    scaleDuration(point.time_from_start, duration_scale);

    for (auto & velocity : point.velocities) {
      velocity /= duration_scale;
    }

    const double acceleration_scale = duration_scale * duration_scale;
    for (auto & acceleration : point.accelerations) {
      acceleration /= acceleration_scale;
    }
  }
}

std::string getOrDeclareString(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const std::string & default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<std::string>(name, default_value);
  }

  std::string value = default_value;
  node->get_parameter(name, value);
  return value;
}
}  // namespace

class TrainHookExecutor
{
public:
  TrainHookExecutor(
    const rclcpp::Node::SharedPtr & node,
    moveit::planning_interface::MoveGroupInterface & move_group,
    double velocity_scale,
    double acceleration_scale,
	    double eef_step,
	    double minimum_cartesian_fraction,
	    double trajectory_duration_scale,
	    bool use_approximate_movej_ik,
	    bool use_exported_movej_jointtargets,
	    bool dry_run,
	    bool validate_trajectory,
	    double max_joint_step_deg)
	  : node_(node),
	    move_group_(move_group),
	    velocity_scale_(velocity_scale),
	    acceleration_scale_(acceleration_scale),
	    eef_step_(eef_step),
	    minimum_cartesian_fraction_(minimum_cartesian_fraction),
	    trajectory_duration_scale_(trajectory_duration_scale),
	    use_approximate_movej_ik_(use_approximate_movej_ik),
	    use_exported_movej_jointtargets_(use_exported_movej_jointtargets),
	    dry_run_(dry_run),
	    validate_trajectory_(validate_trajectory),
	    max_joint_step_deg_(max_joint_step_deg)
	  {
	  }

  bool execute(const std::vector<RapidCommand> & commands)
  {
    for (std::size_t i = 0; i < commands.size();) {
      if (commands[i].type == MoveType::kMoveJ) {
        if (!executeMoveJ(commands[i].target)) {
          return false;
        }
        ++i;
        continue;
      }

      std::vector<std::string> linear_targets;
      const double rapid_speed_mm_s = commands[i].rapid_speed_mm_s;
      while (i < commands.size() && commands[i].type == MoveType::kMoveL &&
        std::abs(commands[i].rapid_speed_mm_s - rapid_speed_mm_s) < 1.0e-6)
      {
        linear_targets.push_back(commands[i].target);
        ++i;
      }

      if (!executeMoveL(linear_targets, rapid_speed_mm_s)) {
        return false;
      }
    }

    return true;
  }

private:
  geometry_msgs::msg::Pose poseForTarget(const std::string & target_name) const
  {
    const auto & targets = rapidTargets();
    const auto target_it = targets.find(target_name);
    if (target_it == targets.end()) {
      throw std::runtime_error("Unknown RAPID target: " + target_name);
    }
    return toTool0Pose(target_it->second);
  }

	  bool executeMoveJ(const std::string & target_name)
	  {
	    RCLCPP_INFO(node_->get_logger(), "MoveJ -> %s", target_name.c_str());

	    move_group_.setStartStateToCurrentState();
	    const double local_velocity_scale = moveJVelocityScaleForTarget(target_name, velocity_scale_);
	    const double local_acceleration_scale =
	      moveJAccelerationScaleForTarget(target_name, acceleration_scale_);
	    move_group_.setMaxVelocityScalingFactor(local_velocity_scale);
	    move_group_.setMaxAccelerationScalingFactor(local_acceleration_scale);
	    RCLCPP_INFO(
	      node_->get_logger(), "MoveJ %s velocity_scale=%.4f, acceleration_scale=%.4f",
	      target_name.c_str(), local_velocity_scale, local_acceleration_scale);

	    const auto joint_target_it = exportedJointTargets().find(target_name);
	    if (use_exported_movej_jointtargets_ && joint_target_it != exportedJointTargets().end()) {
	      auto joints = joint_target_it->second.position_deg;
	      std::map<std::string, double> joint_values = {
	        {"joint_1", degToRad(joints[0])},
	        {"joint_2", degToRad(joints[1])},
	        {"joint_3", degToRad(joints[2])},
	        {"joint_4", degToRad(joints[3])},
	        {"joint_5", degToRad(joints[4])},
	        {"joint_6", degToRad(joints[5])},
	      };
	      RCLCPP_INFO(
	        node_->get_logger(),
	        "%s 使用 ABB CalcJointT 导出的关节目标 [deg]=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
	        target_name.c_str(), joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
	      move_group_.setJointValueTarget(joint_values);
	    } else {
	      const auto target_pose = poseForTarget(target_name);
	      RCLCPP_INFO(
	        node_->get_logger(),
	        "%s 没有导出关节目标，回退到 tool0 位姿 IK。position=[%.4f, %.4f, %.4f], "
	        "orientation=[%.5f, %.5f, %.5f, %.5f]",
	        target_name.c_str(),
	        target_pose.position.x, target_pose.position.y, target_pose.position.z,
	        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

	      if (use_approximate_movej_ik_) {
	        if (!move_group_.setApproximateJointValueTarget(target_pose, "tool0")) {
	          RCLCPP_ERROR(node_->get_logger(), "MoveJ 近似 IK 设置失败：%s", target_name.c_str());
	          return false;
	        }
	      } else {
	        move_group_.setPoseTarget(target_pose, "tool0");
	      }
	    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "MoveJ 规划失败：%s", target_name.c_str());
      return false;
	    }
	    scaleTrajectoryTiming(plan.trajectory_, trajectory_duration_scale_);
	    if (validate_trajectory_ &&
	      !inspectTrajectory(node_->get_logger(), plan.trajectory_, "MoveJ " + target_name, max_joint_step_deg_))
	    {
	      return false;
	    }

	    if (dry_run_) {
	      RCLCPP_INFO(node_->get_logger(), "dry_run=true，跳过执行 MoveJ：%s", target_name.c_str());
	      return true;
	    }

	    if (move_group_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
	      RCLCPP_ERROR(node_->get_logger(), "MoveJ 执行失败：%s", target_name.c_str());
	      move_group_.setMaxVelocityScalingFactor(velocity_scale_);
	      move_group_.setMaxAccelerationScalingFactor(acceleration_scale_);
      return false;
    }

    move_group_.setMaxVelocityScalingFactor(velocity_scale_);
    move_group_.setMaxAccelerationScalingFactor(acceleration_scale_);
    return true;
  }

  double scaledVelocityForRapidSpeed(const double rapid_speed_mm_s) const
  {
    if (rapid_speed_mm_s <= 0.0) {
      return velocity_scale_;
    }

    constexpr double kReferenceRapidSpeed = 2000.0;
    const double rapid_ratio = std::clamp(rapid_speed_mm_s / kReferenceRapidSpeed, 0.10, 1.0);
    return std::max(0.001, velocity_scale_ * rapid_ratio);
  }

  double scaledAccelerationForRapidSpeed(const double rapid_speed_mm_s) const
  {
    if (rapid_speed_mm_s <= 0.0) {
      return acceleration_scale_;
    }

    constexpr double kReferenceRapidSpeed = 2000.0;
    const double rapid_ratio = std::clamp(rapid_speed_mm_s / kReferenceRapidSpeed, 0.10, 1.0);
    return std::max(0.001, acceleration_scale_ * rapid_ratio);
  }

  bool executeMoveL(const std::vector<std::string> & target_names, const double rapid_speed_mm_s)
  {
    if (target_names.empty()) {
      return true;
    }

    const double local_velocity_scale = scaledVelocityForRapidSpeed(rapid_speed_mm_s);
    const double local_acceleration_scale = scaledAccelerationForRapidSpeed(rapid_speed_mm_s);
    RCLCPP_INFO(
      node_->get_logger(),
      "MoveL segment: %s -> %s (%zu waypoints), RAPID speed=v%.0f, velocity_scale=%.4f, acceleration_scale=%.4f",
      target_names.front().c_str(), target_names.back().c_str(), target_names.size(),
      rapid_speed_mm_s, local_velocity_scale, local_acceleration_scale);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(target_names.size());
    for (const auto & target_name : target_names) {
      waypoints.push_back(poseForTarget(target_name));
    }

    moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
    const double fraction =
      move_group_.computeCartesianPath(waypoints, eef_step_, 0.0, cartesian_trajectory);

    RCLCPP_INFO(node_->get_logger(), "MoveL 笛卡尔路径比例：%.2f%%", fraction * 100.0);
    if (fraction < minimum_cartesian_fraction_) {
      RCLCPP_ERROR(
        node_->get_logger(), "MoveL 笛卡尔路径比例不足，要求 %.2f%%，实际 %.2f%%",
        minimum_cartesian_fraction_ * 100.0, fraction * 100.0);
      return false;
    }

    robot_trajectory::RobotTrajectory retimed_trajectory(move_group_.getRobotModel(), "manipulator");
    const auto current_state = move_group_.getCurrentState(2.0);
    if (!current_state) {
      RCLCPP_ERROR(node_->get_logger(), "无法读取当前机器人状态，不能给 MoveL 轨迹生成时间戳。");
      return false;
    }
    retimed_trajectory.setRobotTrajectoryMsg(*current_state, cartesian_trajectory);

    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        retimed_trajectory, local_velocity_scale, local_acceleration_scale))
    {
      RCLCPP_ERROR(node_->get_logger(), "MoveL 轨迹时间参数化失败。");
      return false;
	    }
	    retimed_trajectory.getRobotTrajectoryMsg(cartesian_trajectory);
	    scaleTrajectoryTiming(cartesian_trajectory, trajectory_duration_scale_);
	    if (validate_trajectory_ &&
	      !inspectTrajectory(
	        node_->get_logger(), cartesian_trajectory,
	        "MoveL " + target_names.front() + "->" + target_names.back(), max_joint_step_deg_))
	    {
	      return false;
	    }

	    if (dry_run_) {
	      RCLCPP_INFO(
	        node_->get_logger(), "dry_run=true，跳过执行 MoveL：%s -> %s",
	        target_names.front().c_str(), target_names.back().c_str());
	      return true;
	    }

	    if (move_group_.execute(cartesian_trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
	      RCLCPP_ERROR(node_->get_logger(), "MoveL 执行失败。");
      return false;
    }

    return true;
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface & move_group_;
  double velocity_scale_;
  double acceleration_scale_;
  double eef_step_;
	  double minimum_cartesian_fraction_;
	  double trajectory_duration_scale_;
	  bool use_approximate_movej_ik_;
	  bool use_exported_movej_jointtargets_;
	  bool dry_run_;
	  bool validate_trajectory_;
	  double max_joint_step_deg_;
	};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "execute_train_hook_moveit_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");
  move_group.setEndEffectorLink("tool0");
  move_group.setPoseReferenceFrame("base_link");

  const double velocity_scale = getOrDeclareDouble(node, "velocity_scale", 0.02);
  const double acceleration_scale = getOrDeclareDouble(node, "acceleration_scale", 0.02);
  const double planning_time = getOrDeclareDouble(node, "planning_time", 10.0);
  const double eef_step = getOrDeclareDouble(node, "eef_step", 0.02);
  const double minimum_cartesian_fraction =
    getOrDeclareDouble(node, "minimum_cartesian_fraction", 0.90);
  const std::string procedure = getOrDeclareString(node, "procedure", "guiji_1");
  const std::string planner_id = getOrDeclareString(node, "planner_id", "RRTConnectkConfigDefault");
  const int planning_attempts = getOrDeclareInt(node, "planning_attempts", 10);
  const double goal_position_tolerance = getOrDeclareDouble(node, "goal_position_tolerance", 0.005);
	  const double goal_orientation_tolerance = getOrDeclareDouble(node, "goal_orientation_tolerance", 0.05);
	  const double trajectory_duration_scale = getOrDeclareDouble(node, "trajectory_duration_scale", 1.0);
	  const bool use_approximate_movej_ik = getOrDeclareBool(node, "use_approximate_movej_ik", true);
	  const bool use_exported_movej_jointtargets =
	    getOrDeclareBool(node, "use_exported_movej_jointtargets", true);
	  const bool dry_run = getOrDeclareBool(node, "dry_run", false);
	  const bool validate_trajectory = getOrDeclareBool(node, "validate_trajectory", true);
	  const double max_joint_step_deg = getOrDeclareDouble(node, "max_joint_step_deg", 90.0);

  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(acceleration_scale);
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(planning_attempts);
  move_group.setPlannerId(planner_id);
  move_group.setGoalPositionTolerance(goal_position_tolerance);
  move_group.setGoalOrientationTolerance(goal_orientation_tolerance);

  std::vector<RapidCommand> commands;
  if (procedure == "guiji_1") {
    commands = guiji1();
  } else if (procedure == "guiji_2") {
    commands = guiji2();
  } else {
    RCLCPP_ERROR(node->get_logger(), "未知 procedure='%s'，仅支持 guiji_1 或 guiji_2。", procedure.c_str());
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "准备执行 TrainHookGrab.%s，共 %zu 条运动指令。velocity_scale=%.3f, acceleration_scale=%.3f",
    procedure.c_str(), commands.size(), velocity_scale, acceleration_scale);

	  TrainHookExecutor train_hook_executor(
	    node, move_group, velocity_scale, acceleration_scale, eef_step, minimum_cartesian_fraction,
	    trajectory_duration_scale, use_approximate_movej_ik, use_exported_movej_jointtargets, dry_run,
	    validate_trajectory, max_joint_step_deg);
  const bool ok = train_hook_executor.execute(commands);

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
