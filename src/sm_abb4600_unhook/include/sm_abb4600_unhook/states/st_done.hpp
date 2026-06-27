#pragma once

#include <smacc2/smacc.hpp>
#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"

namespace sm_abb4600_unhook
{

class StDone : public smacc2::SmaccState<StDone, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;

  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StDone] ABB unhook task finished.");
  }

  void onExit()
  {
  }
};

}  // namespace sm_abb4600_unhook
