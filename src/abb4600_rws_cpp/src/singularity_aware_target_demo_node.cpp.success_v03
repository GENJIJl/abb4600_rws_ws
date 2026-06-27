#include <array>
#include <map>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "abb4600_rws_cpp/singularity_aware_planner.hpp"

namespace
{

struct RapidTarget
{
  std::string name;
  std::array<double, 3> position_mm;
  std::array<double, 4> rapid_quaternion;
};

geometry_msgs::msg::Pose toTool0Pose(const RapidTarget & target)
{
  tf2::Quaternion tcp_orientation(
    target.rapid_quaternion[1],
    target.rapid_quaternion[2],
    target.rapid_quaternion[3],
    target.rapid_quaternion[0]);
  tcp_orientation.normalize();

  const tf2::Vector3 tcp_position(
    target.position_mm[0] / 1000.0,
    target.position_mm[1] / 1000.0,
    target.position_mm[2] / 1000.0);

  const tf2::Transform base_to_tcp(tcp_orientation, tcp_position);

  // 当前项目 GZ 工具：tool0 -> TCP 偏移
  const tf2::Transform tool0_to_gz_tcp(
    tf2::Quaternion::getIdentity(),
    tf2::Vector3(-0.075, 0.0, 0.216));

  const tf2::Transform base_to_tool0 = base_to_tcp * tool0_to_gz_tcp.inverse();

  geometry_msgs::msg::Pose pose;
  pose.position.x = base_to_tool0.getOrigin().x();
  pose.position.y = base_to_tool0.getOrigin().y();
  pose.position.z = base_to_tool0.getOrigin().z();
  pose.orientation = tf2::toMsg(base_to_tool0.getRotation());

  return pose;
}

std::vector<RapidTarget> makePath10Targets()
{
  return {
    {"Target_210", {245.832758885, -1339.12895635, 777.464240183},
      {0.478400558, -0.521495507, 0.506705506, -0.492366604}},
    {"Target_30", {1619.584450471, -378.494495066, 655.666637316},
      {0.49193704, -0.507171975, 0.521041538, -0.478842618}},
    {"Target_40", {1619.584424127, 85.971895363, 655.66656862},
      {0.49193703, -0.507171938, 0.521041646, -0.478842549}},
    {"Target_50", {1621.091434395, 80.467737853, 681.868711263},
      {0.569617259, -0.405851496, 0.603317837, -0.383181849}},
    {"Target_60", {1542.091373439, 84.999617909, 658.660023036},
      {0.569617231, -0.405851529, 0.60331785, -0.383181835}},
    {"Target_70", {1540.6423709, 98.101948859, 633.454361049},
      {0.495114067, -0.503691567, 0.524407042, -0.475556642}},
    {"Target_80", {1555.131448251, 98.101972671, 815.902147503},
      {0.495113696, -0.503691987, 0.524406627, -0.475557042}},
    {"Target_90", {1555.131420108, 98.101992456, 893.488734831},
      {0.495113721, -0.503691945, 0.524406672, -0.475557011}},
    {"Target_100", {1543.836348907, 96.610449733, 893.488864478},
      {0.495113724, -0.503691952, 0.524406637, -0.475557039}},
    {"Target_110", {1542.309350856, 92.33936431, 900.547870765},
      {0.402000238, -0.589419901, 0.425784129, -0.556496059}},
    {"Target_120", {1542.309355626, 41.795477908, 911.806915943},
      {0.401999776, -0.58942009, 0.425783845, -0.556496411}},
    {"Target_130", {1542.309406172, 14.649335824, 936.758778901},
      {0.40199986, -0.589420025, 0.425783937, -0.556496349}},
    {"Target_140", {1542.309394052, -50.086859014, 991.490022432},
      {0.401999863, -0.589420062, 0.425783863, -0.556496364}},
    {"Target_150", {1542.309389906, -125.338150744, 1030.917991469},
      {0.4019997, -0.58942013, 0.425783821, -0.556496441}},
    {"Target_160", {1543.538547361, -121.570669946, 1052.290846321},
      {0.478400645, -0.521496192, 0.506704746, -0.492366575}},
    {"Target_170", {1543.538324064, -119.123263464, 1125.30806886},
      {0.478400532, -0.521496219, 0.50670467, -0.492366734}},
    {"Target_180", {1543.538325459, -250.366397395, 1145.547068237},
      {0.478400608, -0.521496077, 0.506704704, -0.492366777}},
    {"Target_190", {1543.538314403, -444.694603495, 1144.412018467},
      {0.478400753, -0.521495973, 0.50670489, -0.492366554}},
    {"Target_200", {1266.543390848, -444.69461636, 742.986355055},
      {0.478400777, -0.521495921, 0.506704892, -0.492366584}},
    {"Target_210", {245.832758885, -1339.12895635, 777.464240183},
      {0.478400558, -0.521495507, 0.506705506, -0.492366604}},
  };
}

RapidTarget getRapidTargetByName(const std::string & target_name)
{
  const auto targets = makePath10Targets();

  for (const auto & target : targets) {
    if (target.name == target_name) {
      return target;
    }
  }

  throw std::runtime_error("Unknown target_name: " + target_name);
}


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

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "singularity_aware_target_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  const bool execute = getParam<bool>(node, "execute", false);
  const bool return_to_start = getParam<bool>(node, "return_to_start", true);
  const double velocity_scale = getParam<double>(node, "velocity_scale", 0.05);
  const double acceleration_scale = getParam<double>(node, "acceleration_scale", 0.05);
  const double planning_time = getParam<double>(node, "planning_time", 60.0);

  const std::string target_name = getParam<std::string>(node, "target_name", "Target_60");

  RCLCPP_INFO(node->get_logger(), "Creating MoveGroupInterface...");

  moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");
  move_group.setEndEffectorLink("tool0");
  move_group.setPoseReferenceFrame("base_link");
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(acceleration_scale);
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(10);

  // 正确 Target_210 六轴构型
  const std::map<std::string, double> target_210_joint_target = {
    {"joint_1", -1.6445413879080628},
    {"joint_2",  0.46621774082971995},
    {"joint_3",  0.2512714958911459},
    {"joint_4",  1.478954819758533},
    {"joint_5",  1.5885588123173982},
    {"joint_6", -3.8906218901927923},
  };

  // 1. 先规划到安全起点 Target_210
  RCLCPP_INFO(node->get_logger(), "Planning to fixed Target_210 first...");

  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(target_210_joint_target);

  moveit::planning_interface::MoveGroupInterface::Plan start_plan;
  const bool start_ok = static_cast<bool>(move_group.plan(start_plan));

  if (!start_ok) {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan to Target_210.");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  auto start_risk = abb4600_rws_cpp::analyzeTrajectoryRisk(
    start_plan.trajectory_.joint_trajectory);

  if (start_risk.type != abb4600_rws_cpp::TrajectoryRiskType::SAFE) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Target_210 plan rejected: %s",
      start_risk.message.c_str());

    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 2;
  }

  if (execute) {
    RCLCPP_WARN(node->get_logger(), "Executing move to Target_210...");
    const bool exec_ok = static_cast<bool>(move_group.execute(start_plan));

    if (!exec_ok) {
      RCLCPP_ERROR(node->get_logger(), "Execution to Target_210 failed.");
      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
      return 3;
    }
  }

  // 2. 根据 target_name 选择 Path10 中的目标点
  RapidTarget target;

  try {
    target = getRapidTargetByName(target_name);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 30;
  }

  const auto target_pose = toTool0Pose(target);

  RCLCPP_INFO(
    node->get_logger(),
    "Target pose from %s: x=%.4f y=%.4f z=%.4f",
    target.name.c_str(),
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);

  // 3. 多 seed IK 筛选
  std::map<std::string, double> q_previous = target_210_joint_target;

  RCLCPP_INFO(node->get_logger(), "Solving safe IK for target pose...");

  const auto safe_ik = abb4600_rws_cpp::solveSafeIkForPose(
    move_group,
    target_pose,
    q_previous,
    "tool0");

  if (!safe_ik.success) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Safe IK failed: %s",
      safe_ik.message.c_str());

    RCLCPP_WARN(
      node->get_logger(),
      "Fallback: keep robot at Target_210 / return_home.");

    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 4;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Safe IK selected. cost=%.3f, message=%s",
    safe_ik.cost,
    safe_ik.message.c_str());

  for (const auto & item : safe_ik.joint_target) {
    RCLCPP_INFO(
      node->get_logger(),
      "  %s = %.6f",
      item.first.c_str(),
      item.second);
  }

  // 4. 使用筛选后的安全 IK 作为关节目标
  RCLCPP_INFO(node->get_logger(), "Planning to selected safe IK joint target...");

  move_group.clearPoseTargets();

  if (execute) {
    move_group.setStartStateToCurrentState();
  } else {
    auto virtual_start_state = move_group.getCurrentState(2.0);

    if (!virtual_start_state) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get robot state for virtual Target_210 start.");
      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
      return 10;
    }

    const auto * jmg = virtual_start_state->getJointModelGroup(move_group.getName());

    if (!jmg) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get JointModelGroup for virtual Target_210 start.");
      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
      return 11;
    }

    const auto joint_names = jmg->getVariableNames();
    std::vector<double> virtual_start_positions;
    virtual_start_positions.reserve(joint_names.size());

    for (const auto & joint_name : joint_names) {
      auto it = target_210_joint_target.find(joint_name);

      if (it == target_210_joint_target.end()) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Target_210 joint target missing joint: %s",
          joint_name.c_str());

        executor.cancel();
        if (spinner.joinable()) {
          spinner.join();
        }
        rclcpp::shutdown();
        return 12;
      }

      virtual_start_positions.push_back(it->second);
    }

    virtual_start_state->setJointGroupPositions(jmg, virtual_start_positions);
    virtual_start_state->update();
    move_group.setStartState(*virtual_start_state);
  }

  move_group.setJointValueTarget(safe_ik.joint_target);

  moveit::planning_interface::MoveGroupInterface::Plan target_plan;
  const bool target_plan_ok = static_cast<bool>(move_group.plan(target_plan));

  if (!target_plan_ok) {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan to safe IK joint target.");

    const auto q_recover =
      abb4600_rws_cpp::makeBoundaryRetreatTarget(q_previous);

    RCLCPP_WARN(node->get_logger(), "Trying boundary retreat target...");

    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(q_recover);

    moveit::planning_interface::MoveGroupInterface::Plan recover_plan;
    const bool recover_ok = static_cast<bool>(move_group.plan(recover_plan));

    if (!recover_ok) {
      RCLCPP_ERROR(node->get_logger(), "Boundary retreat failed. Return home.");
      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
      return 5;
    }

    const auto recover_risk =
      abb4600_rws_cpp::analyzeTrajectoryRisk(
        recover_plan.trajectory_.joint_trajectory);

    if (recover_risk.type != abb4600_rws_cpp::TrajectoryRiskType::SAFE) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Boundary retreat rejected: %s",
        recover_risk.message.c_str());

      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
      return 6;
    }

    if (execute) {
      move_group.execute(recover_plan);
    }

    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 7;
  }

  // 5. 轨迹风险检查
  const auto target_risk =
    abb4600_rws_cpp::analyzeTrajectoryRisk(
      target_plan.trajectory_.joint_trajectory);

  if (target_risk.type != abb4600_rws_cpp::TrajectoryRiskType::SAFE) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Target plan rejected: %s",
      target_risk.message.c_str());

    if (target_risk.type ==
        abb4600_rws_cpp::TrajectoryRiskType::INTERNAL_SINGULARITY) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Internal singularity detected. Use another IK, restore intermediate points, or use Cartesian path.");
    }

    if (target_risk.type ==
        abb4600_rws_cpp::TrajectoryRiskType::EXTERNAL_SINGULARITY) {
      RCLCPP_ERROR(
        node->get_logger(),
        "External singularity detected. Retreat to safe joint target / Target_210.");
    }

    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 8;
  }

  RCLCPP_INFO(node->get_logger(), "Target plan passed singularity-aware guard.");

  if (execute) {
    RCLCPP_WARN(node->get_logger(), "Executing target plan...");
    const bool exec_target_ok = static_cast<bool>(move_group.execute(target_plan));

    if (!exec_target_ok) {
      RCLCPP_ERROR(node->get_logger(), "Target execution failed.");
      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
      return 9;
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Dry run only. Set execute:=true to execute.");
  }

  if (return_to_start) {
    RCLCPP_INFO(node->get_logger(), "Planning return to fixed Target_210...");

    move_group.clearPoseTargets();

    if (execute) {
      move_group.setStartStateToCurrentState();
    } else {
      auto virtual_return_start = move_group.getCurrentState(2.0);

      if (!virtual_return_start) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get robot state for virtual return start.");
        executor.cancel();
        if (spinner.joinable()) {
          spinner.join();
        }
        rclcpp::shutdown();
        return 20;
      }

      const auto * jmg = virtual_return_start->getJointModelGroup(move_group.getName());

      if (!jmg) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get JointModelGroup for virtual return start.");
        executor.cancel();
        if (spinner.joinable()) {
          spinner.join();
        }
        rclcpp::shutdown();
        return 21;
      }

      const auto joint_names = jmg->getVariableNames();
      std::vector<double> virtual_target_positions;
      virtual_target_positions.reserve(joint_names.size());

      for (const auto & joint_name : joint_names) {
        auto it = safe_ik.joint_target.find(joint_name);

        if (it == safe_ik.joint_target.end()) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Safe IK joint target missing joint: %s",
            joint_name.c_str());

          executor.cancel();
          if (spinner.joinable()) {
            spinner.join();
          }
          rclcpp::shutdown();
          return 22;
        }

        virtual_target_positions.push_back(it->second);
      }

      virtual_return_start->setJointGroupPositions(jmg, virtual_target_positions);
      virtual_return_start->update();

      move_group.setStartState(*virtual_return_start);
    }

    move_group.setJointValueTarget(target_210_joint_target);

    moveit::planning_interface::MoveGroupInterface::Plan return_plan;
    const bool return_plan_ok = static_cast<bool>(move_group.plan(return_plan));

    if (!return_plan_ok) {
      RCLCPP_ERROR(node->get_logger(), "Failed to plan return to Target_210.");
      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
      return 23;
    }

    const auto return_risk =
      abb4600_rws_cpp::analyzeTrajectoryRisk(
        return_plan.trajectory_.joint_trajectory);

    if (return_risk.type != abb4600_rws_cpp::TrajectoryRiskType::SAFE) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Return plan rejected: %s",
        return_risk.message.c_str());

      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
      return 24;
    }

    RCLCPP_INFO(node->get_logger(), "Return plan passed singularity-aware guard.");

    if (execute) {
      RCLCPP_WARN(node->get_logger(), "Executing return to Target_210...");
      const bool exec_return_ok = static_cast<bool>(move_group.execute(return_plan));

      if (!exec_return_ok) {
        RCLCPP_ERROR(node->get_logger(), "Return execution failed.");
        executor.cancel();
        if (spinner.joinable()) {
          spinner.join();
        }
        rclcpp::shutdown();
        return 25;
      }
    }
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
