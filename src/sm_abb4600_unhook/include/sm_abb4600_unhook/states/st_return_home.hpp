#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <smacc2/smacc.hpp>
#include <abb4600_interfaces/action/run_robot_path.hpp>

#include "sm_abb4600_unhook/sm_abb4600_unhook.hpp"
#include "sm_abb4600_unhook/common/error_reporter.hpp"
#include "sm_abb4600_unhook/states/st_done.hpp"
#include "sm_abb4600_unhook/states/st_error.hpp"

namespace sm_abb4600_unhook
{

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

using smacc2::Transition;

struct EvReturnHomeSucceeded : sc::event<EvReturnHomeSucceeded> {};
struct EvReturnHomeFailed : sc::event<EvReturnHomeFailed> {};

class StReturnHome : public smacc2::SmaccState<StReturnHome, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;
  using RunRobotPath = abb4600_interfaces::action::RunRobotPath;
  using GoalHandleRunRobotPath = rclcpp_action::ClientGoalHandle<RunRobotPath>;

  typedef mpl::list<
    Transition<EvReturnHomeSucceeded, StDone, smacc2::default_transition_tags::SUCCESS>,
    Transition<EvReturnHomeFailed, StError, smacc2::default_transition_tags::ABORT>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StReturnHome] Sending return_home action goal to /abb4600/run_robot_path ...");

    auto node = std::make_shared<rclcpp::Node>("return_home_smacc_client");

    auto client = rclcpp_action::create_client<RunRobotPath>(
      node,
      "/abb4600/run_robot_path");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    if (!client->wait_for_action_server(std::chrono::seconds(30)))
    {
      writeLastErrorReport(
        getLogger(),
        "StReturnHome",
        "Action server /abb4600/run_robot_path is not available.",
        "Start the action server first:\n\n"
        "ros2 launch abb4600_rws_cpp run_robot_path_action_server.launch.py\n");

      this->postEvent<EvReturnHomeFailed>();
      return;
    }

    RunRobotPath::Goal goal;
    goal.path_name = "return_home";
    goal.velocity_scale = 0.10;
    goal.acceleration_scale = 0.10;
    goal.planning_time = 60.0;
    goal.eef_step = 0.01;
    goal.minimum_cartesian_fraction = 0.90;
    goal.return_to_start = false;

    auto options = rclcpp_action::Client<RunRobotPath>::SendGoalOptions();

    options.feedback_callback =
      [this](
        GoalHandleRunRobotPath::SharedPtr,
        const std::shared_ptr<const RunRobotPath::Feedback> feedback)
      {
        RCLCPP_INFO(
          this->getLogger(),
          "[StReturnHome] feedback %.0f%%: %s",
          feedback->progress * 100.0,
          feedback->current_stage.c_str());
      };

    auto goal_handle_future = client->async_send_goal(goal, options);

    if (executor.spin_until_future_complete(goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      writeLastErrorReport(
        getLogger(),
        "StReturnHome",
        "Failed to send return_home goal.",
        "Check action server terminal and ROS graph:\n\n"
        "ros2 action info /abb4600/run_robot_path\n");

      this->postEvent<EvReturnHomeFailed>();
      return;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle)
    {
      writeLastErrorReport(
        getLogger(),
        "StReturnHome",
        "return_home goal was rejected by action server.",
        "Check action server logs and return_home configuration.");

      this->postEvent<EvReturnHomeFailed>();
      return;
    }

    RCLCPP_INFO(getLogger(), "[StReturnHome] Goal accepted. Waiting for result...");

    auto result_future = client->async_get_result(goal_handle);

    if (executor.spin_until_future_complete(result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      writeLastErrorReport(
        getLogger(),
        "StReturnHome",
        "Failed while waiting for return_home action result.",
        "Check action server terminal logs.");

      this->postEvent<EvReturnHomeFailed>();
      return;
    }

    auto wrapped_result = result_future.get();

    if (
      wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped_result.result &&
      wrapped_result.result->success)
    {
      RCLCPP_INFO(
        getLogger(),
        "[StReturnHome] return_home succeeded: %s",
        wrapped_result.result->message.c_str());

      this->postEvent<EvReturnHomeSucceeded>();
      return;
    }

    std::string message = "unknown error";

    if (wrapped_result.result)
    {
      message = wrapped_result.result->message;
    }

    writeLastErrorReport(
      getLogger(),
      "StReturnHome",
      "return_home action failed: " + message,
      "Check run_robot_path_action_server terminal.\n\n"
      "If TrajectoryGuard rejected the trajectory, reduce velocity_scale/acceleration_scale, "
      "configure a safer return_home.middle_joints, or move robot to a safer posture.");

    this->postEvent<EvReturnHomeFailed>();
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StReturnHome] Leaving state.");
  }
};

}  // namespace sm_abb4600_unhook
