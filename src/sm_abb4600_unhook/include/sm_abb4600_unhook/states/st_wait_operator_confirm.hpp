#pragma once

#include <memory>

#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <smacc2/smacc.hpp>

#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"

namespace sm_abb4600_unhook
{

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

using smacc2::Transition;

class StRunUnhookDemo;
class StRecover;

struct EvOperatorConfirmed : sc::event<EvOperatorConfirmed> {};
struct EvOperatorCanceled : sc::event<EvOperatorCanceled> {};

class StWaitOperatorConfirm
: public smacc2::SmaccState<StWaitOperatorConfirm, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvOperatorConfirmed, StRunUnhookDemo, smacc2::default_transition_tags::SUCCESS>,
    Transition<EvOperatorCanceled, StRecover, smacc2::default_transition_tags::ABORT>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_WARN(
      getLogger(),
      "[StWaitOperatorConfirm] Safety check passed. Waiting for operator confirmation.");

    RCLCPP_WARN(
      getLogger(),
      "[StWaitOperatorConfirm] Publish /abb4600_unhook/confirm_start to execute Path10.");

    RCLCPP_WARN(
      getLogger(),
      "[StWaitOperatorConfirm] Publish /abb4600_unhook/cancel to cancel and recover.");

    auto node = getNode();

    confirm_sub_ = node->create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/confirm_start",
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Empty::SharedPtr)
      {
        RCLCPP_INFO(this->getLogger(), "[StWaitOperatorConfirm] Operator confirmed start.");
        this->postEvent<EvOperatorConfirmed>();
      });

    cancel_sub_ = node->create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/cancel",
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Empty::SharedPtr)
      {
        RCLCPP_WARN(this->getLogger(), "[StWaitOperatorConfirm] Operator canceled task.");
        this->postEvent<EvOperatorCanceled>();
      });
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StWaitOperatorConfirm] Leaving state.");
    confirm_sub_.reset();
    cancel_sub_.reset();
  }

private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr confirm_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
};

}  // namespace sm_abb4600_unhook
