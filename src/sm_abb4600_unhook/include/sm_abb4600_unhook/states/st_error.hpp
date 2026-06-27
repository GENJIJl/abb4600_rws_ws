#pragma once

#include <memory>

#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <smacc2/smacc.hpp>

#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"
#include "sm_abb4600_unhook/common/error_reporter.hpp"

namespace sm_abb4600_unhook
{

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

using smacc2::Transition;

class StRecover;

struct EvResetRequested : sc::event<EvResetRequested> {};

class StError : public smacc2::SmaccState<StError, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvResetRequested, StRecover, smacc2::default_transition_tags::SUCCESS>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_ERROR(getLogger(), "[StError] State machine entered ERROR state.");

    auto node = getNode();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    error_report_pub_ =
      node->create_publisher<std_msgs::msg::String>(
        "/abb4600_unhook/error_report",
        qos);

    const auto report = readLastErrorReport();

    std_msgs::msg::String msg;
    msg.data = report;
    error_report_pub_->publish(msg);

    RCLCPP_ERROR(getLogger(), "%s", report.c_str());

    RCLCPP_ERROR(
      getLogger(),
      "[StError] After fixing the problem, publish /abb4600_unhook/reset to recover to StIdle.");

    reset_sub_ = node->create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/reset",
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Empty::SharedPtr)
      {
        RCLCPP_WARN(this->getLogger(), "[StError] Reset requested.");
        this->postEvent<EvResetRequested>();
      });
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StError] Leaving error state.");
    reset_sub_.reset();
    error_report_pub_.reset();
  }

private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_report_pub_;
};

}  // namespace sm_abb4600_unhook
