#pragma once

#include <chrono>
#include <map>
#include <string>
#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <smacc2/smacc.hpp>

#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"
#include "sm_abb4600_unhook/states/st_run_unhook_demo.hpp"
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
    Transition<EvSystemReady, StRunUnhookDemo, smacc2::default_transition_tags::SUCCESS>,
    Transition<EvSystemNotReady, StError, smacc2::default_transition_tags::ABORT>
  > reactions;

  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StWaitSystemReady] Checking ABB ROS2 system...");

    auto node = getNode();

    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      node,
      "/joint_trajectory_controller/follow_joint_trajectory"
    );

    bool joint_states_ok = false;
    bool controller_ok = false;

    auto start_time = node->now();
    rclcpp::Rate rate(2.0);

    while (rclcpp::ok() && (node->now() - start_time).seconds() < 20.0)
    {
      auto topics = node->get_topic_names_and_types();

      joint_states_ok = topics.find("/joint_states") != topics.end();
      controller_ok = action_client->wait_for_action_server(std::chrono::milliseconds(200));

      RCLCPP_INFO_THROTTLE(
        getLogger(),
        *node->get_clock(),
        2000,
        "[StWaitSystemReady] /joint_states=%s, trajectory_action=%s",
        joint_states_ok ? "OK" : "WAIT",
        controller_ok ? "OK" : "WAIT"
      );

      if (joint_states_ok && controller_ok)
      {
        RCLCPP_INFO(getLogger(), "[StWaitSystemReady] System ready.");
        this->postEvent<EvSystemReady>();
        return;
      }

      rate.sleep();
    }

    RCLCPP_ERROR(
      getLogger(),
      "[StWaitSystemReady] Timeout. joint_states=%s, trajectory_action=%s",
      joint_states_ok ? "OK" : "FAIL",
      controller_ok ? "OK" : "FAIL"
    );

    this->postEvent<EvSystemNotReady>();
  }

  void onExit()
  {
  }
};

}  // namespace sm_abb4600_unhook
