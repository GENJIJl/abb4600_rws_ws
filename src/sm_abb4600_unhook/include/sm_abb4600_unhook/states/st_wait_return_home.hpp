#pragma once

#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <smacc2/smacc.hpp>

#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"
#include "sm_abb4600_unhook/states/st_return_home.hpp"
#include "sm_abb4600_unhook/states/st_done.hpp"

namespace sm_abb4600_unhook
{

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

using smacc2::Transition;

struct EvReturnHomeCommand : sc::event<EvReturnHomeCommand> {};
struct EvSkipReturnHomeCommand : sc::event<EvSkipReturnHomeCommand> {};

class StWaitReturnHome : public smacc2::SmaccState<StWaitReturnHome, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvReturnHomeCommand, StReturnHome, smacc2::default_transition_tags::SUCCESS>,
    Transition<EvSkipReturnHomeCommand, StDone, smacc2::default_transition_tags::ABORT>
  > reactions;

  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(),
      "[StWaitReturnHome] Path task finished. Waiting for return-home command...");
    RCLCPP_INFO(
      getLogger(),
      "[StWaitReturnHome] Publish /abb4600_unhook/return_home to return home.");
    RCLCPP_INFO(
      getLogger(),
      "[StWaitReturnHome] Publish /abb4600_unhook/skip_return_home to finish without returning home.");

    auto node = getNode();

    return_home_sub_ = node->create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/return_home",
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Empty::SharedPtr)
      {
        RCLCPP_INFO(this->getLogger(), "[StWaitReturnHome] Return-home command received.");
        this->postEvent<EvReturnHomeCommand>();
      });

    skip_return_home_sub_ = node->create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/skip_return_home",
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Empty::SharedPtr)
      {
        RCLCPP_WARN(this->getLogger(), "[StWaitReturnHome] Skip return-home command received.");
        this->postEvent<EvSkipReturnHomeCommand>();
      });
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StWaitReturnHome] Leaving wait-return-home state.");
    return_home_sub_.reset();
    skip_return_home_sub_.reset();
  }

private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr return_home_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr skip_return_home_sub_;
};

}  // namespace sm_abb4600_unhook
