#pragma once

#include <smacc2/smacc.hpp>
#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"

namespace sm_abb4600_unhook
{

class StError : public smacc2::SmaccState<StError, SmAbb4600Unhook>
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
    RCLCPP_ERROR(getLogger(), "[StError] ABB unhook task failed.");
  }

  void onExit()
  {
  }
};

}  // namespace sm_abb4600_unhook
