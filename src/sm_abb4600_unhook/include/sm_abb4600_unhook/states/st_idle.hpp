#pragma once

#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <smacc2/smacc.hpp>

#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"
#include "sm_abb4600_unhook/states/st_wait_system_ready.hpp"

namespace sm_abb4600_unhook
{

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

using smacc2::Transition;

struct EvStartUnhook : sc::event<EvStartUnhook> {};

class StIdle : public smacc2::SmaccState<StIdle, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvStartUnhook, StWaitSystemReady, smacc2::default_transition_tags::SUCCESS>
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
      "[StIdle] Waiting for start signal on /abb4600_unhook/start ...");

    auto node = getNode();

    start_sub_ = node->create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/start",
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Empty::SharedPtr)
      {
        RCLCPP_INFO(this->getLogger(), "[StIdle] Start signal received.");
        this->postEvent<EvStartUnhook>();
      });
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StIdle] Leaving idle state.");
    start_sub_.reset();
  }

private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
};

}  // namespace sm_abb4600_unhook
