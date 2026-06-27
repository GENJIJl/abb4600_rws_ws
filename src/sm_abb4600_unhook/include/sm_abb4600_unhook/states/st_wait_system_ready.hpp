#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <smacc2/smacc.hpp>

#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"
#include "sm_abb4600_unhook/common/error_reporter.hpp"
#include "sm_abb4600_unhook/states/st_safety_check.hpp"
#include "sm_abb4600_unhook/states/st_error.hpp"

namespace sm_abb4600_unhook
{

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

using smacc2::Transition;

struct EvSystemReady : sc::event<EvSystemReady> {};
struct EvSystemNotReady : sc::event<EvSystemNotReady> {};

class StWaitSystemReady : public smacc2::SmaccState<StWaitSystemReady, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  typedef mpl::list<
    Transition<EvSystemReady, StSafetyCheck, smacc2::default_transition_tags::SUCCESS>,
    Transition<EvSystemNotReady, StError, smacc2::default_transition_tags::ABORT>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StWaitSystemReady] Checking ROS2 control, controllers, joint_states, and MoveIt...");

    auto node = getNode();

    start_time_ = std::chrono::steady_clock::now();
    got_joint_states_ = false;
    posted_ = false;
    tick_count_ = 0;

    joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr)
      {
        got_joint_states_ = true;
        last_joint_state_time_ = std::chrono::steady_clock::now();
      });

    trajectory_action_client_ =
      rclcpp_action::create_client<FollowJointTrajectory>(
        node,
        "/joint_trajectory_controller/follow_joint_trajectory");

    timer_ = node->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]()
      {
        if (posted_)
        {
          return;
        }

        tick_count_++;

        const bool controller_manager_ok = hasService("/controller_manager/list_controllers");
        const bool joint_states_ok = hasRecentJointStates();
        const bool trajectory_action_ok =
          trajectory_action_client_->wait_for_action_server(std::chrono::milliseconds(100));
        const bool move_group_ok = hasNodeNameContaining("move_group");

        RCLCPP_INFO(
          this->getLogger(),
          "[StWaitSystemReady] controller_manager=%s, /joint_states=%s, trajectory_action=%s, move_group=%s",
          controller_manager_ok ? "OK" : "WAIT",
          joint_states_ok ? "OK" : "WAIT",
          trajectory_action_ok ? "OK" : "WAIT",
          move_group_ok ? "OK" : "WAIT");

        if (controller_manager_ok && joint_states_ok && trajectory_action_ok && move_group_ok)
        {
          posted_ = true;
          RCLCPP_INFO(this->getLogger(), "[StWaitSystemReady] System ready.");
          this->postEvent<EvSystemReady>();
          return;
        }

        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start_time_).count();

        if (elapsed > timeout_sec_)
        {
          posted_ = true;

          writeLastErrorReport(
            this->getLogger(),
            "StWaitSystemReady",
            buildFailureReason(controller_manager_ok, joint_states_ok, trajectory_action_ok, move_group_ok),
            buildSuggestedFix(controller_manager_ok, joint_states_ok, trajectory_action_ok, move_group_ok));

          this->postEvent<EvSystemNotReady>();
        }
      });
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StWaitSystemReady] Leaving state.");
    joint_state_sub_.reset();
    timer_.reset();
    trajectory_action_client_.reset();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_joint_state_time_;

  bool got_joint_states_{false};
  bool posted_{false};
  int tick_count_{0};
  double timeout_sec_{20.0};

  bool hasService(const std::string & service_name)
  {
    const auto node = getNode();
    const auto services = node->get_service_names_and_types();

    for (const auto & service : services)
    {
      if (service.first == service_name)
      {
        return true;
      }
    }

    return false;
  }

  bool hasNodeNameContaining(const std::string & keyword)
  {
    const auto node = getNode();
    const auto names = node->get_node_names();

    for (const auto & name : names)
    {
      if (name.find(keyword) != std::string::npos)
      {
        return true;
      }
    }

    return false;
  }

  bool hasRecentJointStates()
  {
    if (!got_joint_states_)
    {
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const double age = std::chrono::duration<double>(now - last_joint_state_time_).count();

    return age < 2.0;
  }

  std::string buildFailureReason(
    bool controller_manager_ok,
    bool joint_states_ok,
    bool trajectory_action_ok,
    bool move_group_ok)
  {
    std::ostringstream oss;

    if (!controller_manager_ok)
    {
      oss << "controller_manager service /controller_manager/list_controllers was not found. "
          << "abb_control or ros2_control_node is probably not running.";
      return oss.str();
    }

    if (!joint_states_ok)
    {
      oss << "/joint_states is not receiving data. "
          << "Most likely joint_state_broadcaster was not spawned or is not active.";
      return oss.str();
    }

    if (!trajectory_action_ok)
    {
      oss << "Action server /joint_trajectory_controller/follow_joint_trajectory was not found. "
          << "Most likely joint_trajectory_controller was not spawned or is not active.";
      return oss.str();
    }

    if (!move_group_ok)
    {
      oss << "move_group node was not found. "
          << "MoveIt was probably not started.";
      return oss.str();
    }

    oss << "Unknown system readiness failure.";
    return oss.str();
  }

  std::string buildSuggestedFix(
    bool controller_manager_ok,
    bool joint_states_ok,
    bool trajectory_action_ok,
    bool move_group_ok)
  {
    std::ostringstream oss;

    if (!controller_manager_ok)
    {
      oss << "Start abb_control first:\n\n"
          << "cd ~/abb4600_rws_ws\n"
          << "unset ROS_DOMAIN_ID\n"
          << "source /opt/ros/humble/setup.bash\n"
          << "source install/setup.bash\n"
          << "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n\n"
          << "ros2 launch abb_bringup abb_control.launch.py \\\n"
          << "  description_package:=abb_irb4600_support \\\n"
          << "  description_file:=irb4600_60_205.xacro \\\n"
          << "  moveit_config_package:=abb_irb4600_60_205_moveit_config \\\n"
          << "  runtime_config_package:=abb_bringup \\\n"
          << "  controllers_file:=abb_controllers.yaml \\\n"
          << "  initial_joint_controller:=joint_trajectory_controller \\\n"
          << "  start_initial_joint_controller:=true \\\n"
          << "  rws_ip:=192.168.31.12 \\\n"
          << "  rws_port:=80 \\\n"
          << "  egm_port:=6511 \\\n"
          << "  use_fake_hardware:=false \\\n"
          << "  launch_rviz:=false\n";
      return oss.str();
    }

    if (!joint_states_ok)
    {
      oss << "Spawn joint_state_broadcaster first:\n\n"
          << "cd ~/abb4600_rws_ws\n"
          << "unset ROS_DOMAIN_ID\n"
          << "source /opt/ros/humble/setup.bash\n"
          << "source install/setup.bash\n"
          << "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n\n"
          << "ros2 run controller_manager spawner joint_state_broadcaster \\\n"
          << "  --controller-manager /controller_manager \\\n"
          << "  --param-file ~/abb4600_rws_ws/install/abb_bringup/share/abb_bringup/config/abb_controllers.yaml \\\n"
          << "  --controller-manager-timeout 120 \\\n"
          << "  --service-call-timeout 60\n\n"
          << "Wait until you see:\n"
          << "  Configured and activated joint_state_broadcaster\n";
      return oss.str();
    }

    if (!trajectory_action_ok)
    {
      oss << "Spawn joint_trajectory_controller after joint_state_broadcaster is active:\n\n"
          << "cd ~/abb4600_rws_ws\n"
          << "unset ROS_DOMAIN_ID\n"
          << "source /opt/ros/humble/setup.bash\n"
          << "source install/setup.bash\n"
          << "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n\n"
          << "ros2 run controller_manager spawner joint_trajectory_controller \\\n"
          << "  --controller-manager /controller_manager \\\n"
          << "  --param-file ~/abb4600_rws_ws/install/abb_bringup/share/abb_bringup/config/abb_controllers.yaml \\\n"
          << "  --controller-manager-timeout 120 \\\n"
          << "  --service-call-timeout 60\n";
      return oss.str();
    }

    if (!move_group_ok)
    {
      oss << "Start MoveIt:\n\n"
          << "cd ~/abb4600_rws_ws\n"
          << "unset ROS_DOMAIN_ID\n"
          << "source /opt/ros/humble/setup.bash\n"
          << "source install/setup.bash\n"
          << "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n\n"
          << "ros2 launch abb_bringup abb_moveit.launch.py \\\n"
          << "  robot_xacro_file:=irb4600_60_205.xacro \\\n"
          << "  support_package:=abb_irb4600_support \\\n"
          << "  moveit_config_package:=abb_irb4600_60_205_moveit_config \\\n"
          << "  moveit_config_file:=abb_irb4600_60_205.srdf.xacro\n";
      return oss.str();
    }

    oss << "Check state machine terminal logs, RTA transition history, and ROS2 graph.";
    return oss.str();
  }
};

}  // namespace sm_abb4600_unhook
