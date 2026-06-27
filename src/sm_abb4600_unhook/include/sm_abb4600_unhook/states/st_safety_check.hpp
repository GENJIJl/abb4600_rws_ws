#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <smacc2/smacc.hpp>

#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"
#include "sm_abb4600_unhook/common/error_reporter.hpp"

namespace sm_abb4600_unhook
{

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

using smacc2::Transition;

class StWaitOperatorConfirm;
class StError;

struct EvSafetyCheckPassed : sc::event<EvSafetyCheckPassed> {};
struct EvSafetyCheckFailed : sc::event<EvSafetyCheckFailed> {};

struct JointSoftLimit
{
  double lower;
  double upper;
};

// 根据 irb4600_60_205_macro.xacro 中的 URDF joint limit 设置。
// 单位：rad
static const std::map<std::string, JointSoftLimit> kJointSoftLimits = {
  {"joint_1", {-3.141,  3.141}},
  {"joint_2", {-1.570,  2.617}},
  {"joint_3", {-3.141,  1.309}},
  {"joint_4", {-6.981,  6.981}},
  {"joint_5", {-2.181,  2.094}},
  {"joint_6", {-6.981,  6.981}},
};

class StSafetyCheck : public smacc2::SmaccState<StSafetyCheck, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvSafetyCheckPassed, StWaitOperatorConfirm, smacc2::default_transition_tags::SUCCESS>,
    Transition<EvSafetyCheckFailed, StError, smacc2::default_transition_tags::ABORT>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StSafetyCheck] Checking all 6 robot joints...");

    sensor_msgs::msg::JointState joint_state;
    auto node = getNode();

    const bool got_msg = rclcpp::wait_for_message(
      joint_state,
      node,
      "/joint_states",
      std::chrono::seconds(3));

    if (!got_msg)
    {
      writeLastErrorReport(
        getLogger(),
        "StSafetyCheck",
        "Could not receive /joint_states within 3 seconds.",
        "Check whether joint_state_broadcaster is active:\n\n"
        "ros2 control list_controllers\n"
        "ros2 topic echo /joint_states --once\n\n"
        "If joint_state_broadcaster is not active, run:\n\n"
        "ros2 run controller_manager spawner joint_state_broadcaster \\\n"
        "  --controller-manager /controller_manager \\\n"
        "  --param-file ~/abb4600_rws_ws/install/abb_bringup/share/abb_bringup/config/abb_controllers.yaml \\\n"
        "  --controller-manager-timeout 120 \\\n"
        "  --service-call-timeout 60\n");

      this->postEvent<EvSafetyCheckFailed>();
      return;
    }

    if (joint_state.name.empty() || joint_state.position.empty())
    {
      writeLastErrorReport(
        getLogger(),
        "StSafetyCheck",
        "/joint_states message was received, but name[] or position[] is empty.",
        "Check /joint_states:\n\n"
        "ros2 topic echo /joint_states --once\n\n"
        "Also check controllers:\n\n"
        "ros2 control list_controllers\n");

      this->postEvent<EvSafetyCheckFailed>();
      return;
    }

    for (size_t i = 0; i < joint_state.name.size() && i < joint_state.position.size(); ++i)
    {
      RCLCPP_INFO(
        getLogger(),
        "[StSafetyCheck] %s = %.6f rad",
        joint_state.name[i].c_str(),
        joint_state.position[i]);
    }

    std::string error_message;

    if (!checkAllExpectedJointsExist(joint_state, error_message))
    {
      fail(error_message);
      return;
    }

    if (!checkJointFinite(joint_state, error_message))
    {
      fail(error_message);
      return;
    }

    if (!checkAllJointSoftLimits(joint_state, error_message))
    {
      fail(error_message);
      return;
    }

    RCLCPP_INFO(getLogger(), "[StSafetyCheck] All joint safety checks passed.");
    this->postEvent<EvSafetyCheckPassed>();
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StSafetyCheck] Leaving state.");
  }

private:
  const double safety_margin_{0.10};  // rad，约 5.7 度

  void fail(const std::string & reason)
  {
    writeLastErrorReport(
      getLogger(),
      "StSafetyCheck",
      reason,
      "Move the robot to a safer posture in RobotStudio using Jog mode, then check:\n\n"
      "ros2 topic echo /joint_states --once\n\n"
      "If the soft limit is too conservative, adjust kJointSoftLimits or safety_margin_ in:\n\n"
      "~/abb4600_rws_ws/src/sm_abb4600_unhook/include/sm_abb4600_unhook/states/st_safety_check.hpp\n\n"
      "After editing, rebuild and restart the state machine.");

    this->postEvent<EvSafetyCheckFailed>();
  }

  bool findJointPosition(
    const sensor_msgs::msg::JointState & joint_state,
    const std::string & joint_name,
    double & position,
    std::string & error_message)
  {
    const auto it = std::find(
      joint_state.name.begin(),
      joint_state.name.end(),
      joint_name);

    if (it == joint_state.name.end())
    {
      error_message = "Cannot find " + joint_name + " in /joint_states.";
      return false;
    }

    const size_t index = static_cast<size_t>(std::distance(joint_state.name.begin(), it));

    if (index >= joint_state.position.size())
    {
      error_message = joint_name + " index is outside /joint_states.position array.";
      return false;
    }

    position = joint_state.position[index];
    return true;
  }

  bool checkAllExpectedJointsExist(
    const sensor_msgs::msg::JointState & joint_state,
    std::string & error_message)
  {
    for (const auto & item : kJointSoftLimits)
    {
      double position = 0.0;
      if (!findJointPosition(joint_state, item.first, position, error_message))
      {
        return false;
      }
    }

    return true;
  }

  bool checkJointFinite(
    const sensor_msgs::msg::JointState & joint_state,
    std::string & error_message)
  {
    for (const auto & item : kJointSoftLimits)
    {
      double position = 0.0;

      if (!findJointPosition(joint_state, item.first, position, error_message))
      {
        return false;
      }

      if (!std::isfinite(position))
      {
        std::ostringstream oss;
        oss << item.first << " position is NaN or Inf.";
        error_message = oss.str();
        return false;
      }
    }

    return true;
  }

  bool checkAllJointSoftLimits(
    const sensor_msgs::msg::JointState & joint_state,
    std::string & error_message)
  {
    for (const auto & item : kJointSoftLimits)
    {
      const std::string & joint_name = item.first;
      const JointSoftLimit & limit = item.second;

      double position = 0.0;

      if (!findJointPosition(joint_state, joint_name, position, error_message))
      {
        return false;
      }

      const double safe_lower = limit.lower + safety_margin_;
      const double safe_upper = limit.upper - safety_margin_;

      RCLCPP_INFO(
        getLogger(),
        "[StSafetyCheck] %s = %.6f rad, hard=[%.3f, %.3f], safe=[%.3f, %.3f]",
        joint_name.c_str(),
        position,
        limit.lower,
        limit.upper,
        safe_lower,
        safe_upper);

      if (position < limit.lower || position > limit.upper)
      {
        std::ostringstream oss;
        oss << joint_name
            << " is outside hard URDF limit. "
            << joint_name << "=" << position
            << " rad, hard limit=["
            << limit.lower << ", " << limit.upper << "] rad.";

        error_message = oss.str();
        return false;
      }

      if (position < safe_lower || position > safe_upper)
      {
        std::ostringstream oss;
        oss << joint_name
            << " is too close to joint limit. "
            << joint_name << "=" << position
            << " rad, safe range with margin=["
            << safe_lower << ", " << safe_upper << "] rad, "
            << "hard limit=["
            << limit.lower << ", " << limit.upper << "] rad, "
            << "safety_margin=" << safety_margin_ << " rad.";

        error_message = oss.str();
        return false;
      }
    }

    return true;
  }
};

}  // namespace sm_abb4600_unhook
