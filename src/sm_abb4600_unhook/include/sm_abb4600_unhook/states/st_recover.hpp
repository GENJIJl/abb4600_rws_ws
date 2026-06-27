#pragma once

#include <chrono>
#include <memory>

#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <rclcpp/rclcpp.hpp>

#include <smacc2/smacc.hpp>

#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"

namespace sm_abb4600_unhook
{

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

using smacc2::Transition;

class StIdle;

struct EvRecoveryDone : sc::event<EvRecoveryDone> {};

class StRecover : public smacc2::SmaccState<StRecover, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvRecoveryDone, StIdle, smacc2::default_transition_tags::SUCCESS>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_WARN(getLogger(), "[StRecover] Recovering state machine...");
    RCLCPP_WARN(getLogger(), "[StRecover] Current version only resets SMACC2 flow back to StIdle.");
    RCLCPP_WARN(getLogger(), "[StRecover] Robot motion recovery must still be handled manually if robot is faulted.");

    auto node = getNode();

    timer_ = node->create_wall_timer(
      std::chrono::milliseconds(800),
      [this]()
      {
        RCLCPP_INFO(this->getLogger(), "[StRecover] Recovery finished. Going back to StIdle.");
        this->postEvent<EvRecoveryDone>();
      });
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StRecover] Leaving state.");
    timer_.reset();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sm_abb4600_unhook
