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
#include "sm_abb4600_unhook/states/st_wait_return_home.hpp"
#include "sm_abb4600_unhook/states/st_error.hpp"

namespace sm_abb4600_unhook
{

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

using smacc2::Transition;

struct EvUnhookPathSucceeded : sc::event<EvUnhookPathSucceeded> {};
struct EvUnhookPathFailed : sc::event<EvUnhookPathFailed> {};

class StRunUnhookDemo : public smacc2::SmaccState<StRunUnhookDemo, SmAbb4600Unhook>
{
public:
  using SmaccState::SmaccState;
  using RunRobotPath = abb4600_interfaces::action::RunRobotPath;
  using GoalHandleRunRobotPath = rclcpp_action::ClientGoalHandle<RunRobotPath>;

  typedef mpl::list<
    Transition<EvUnhookPathSucceeded, StWaitReturnHome, smacc2::default_transition_tags::SUCCESS>,
    Transition<EvUnhookPathFailed, StError, smacc2::default_transition_tags::ABORT>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StRunUnhookDemo] Sending action goal to /abb4600/run_robot_path ...");

    auto node = std::make_shared<rclcpp::Node>("run_robot_path_smacc_client");
    auto client = rclcpp_action::create_client<RunRobotPath>(
      node,
      "/abb4600/run_robot_path");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    if (!client->wait_for_action_server(std::chrono::seconds(30)))
    {
      writeLastErrorReport(
        getLogger(),
        "StRunUnhookDemo",
        "Action server /abb4600/run_robot_path is not available.",
        "Start the path action server first:\n\n"
        "cd ~/abb4600_rws_ws\n"
        "unset ROS_DOMAIN_ID\n"
        "source /opt/ros/humble/setup.bash\n"
        "source install/setup.bash\n"
        "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n\n"
        "ros2 launch abb4600_rws_cpp run_robot_path_action_server.launch.py\n");

      this->postEvent<EvUnhookPathFailed>();
      return;
    }

    RunRobotPath::Goal goal;
    goal.path_name = "path10";
    goal.velocity_scale = 0.20;
    goal.acceleration_scale = 0.20;
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
          "[StRunUnhookDemo] feedback %.0f%%: %s",
          feedback->progress * 100.0,
          feedback->current_stage.c_str());
      };

    auto goal_handle_future = client->async_send_goal(goal, options);

    if (executor.spin_until_future_complete(goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      writeLastErrorReport(
        getLogger(),
        "StRunUnhookDemo",
        "Failed to send goal to /abb4600/run_robot_path.",
        "Check action server terminal and ROS graph:\n\n"
        "ros2 action info /abb4600/run_robot_path\n");

      this->postEvent<EvUnhookPathFailed>();
      return;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle)
    {
      writeLastErrorReport(
        getLogger(),
        "StRunUnhookDemo",
        "Goal was rejected by /abb4600/run_robot_path.",
        "Check path_name and action server logs.");

      this->postEvent<EvUnhookPathFailed>();
      return;
    }

    RCLCPP_INFO(getLogger(), "[StRunUnhookDemo] Goal accepted. Waiting for result...");

    auto result_future = client->async_get_result(goal_handle);

    if (executor.spin_until_future_complete(result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      writeLastErrorReport(
        getLogger(),
        "StRunUnhookDemo",
        "Failed while waiting for /abb4600/run_robot_path result.",
        "Check action server terminal logs.");

      this->postEvent<EvUnhookPathFailed>();
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
        "[StRunUnhookDemo] Path action succeeded: %s",
        wrapped_result.result->message.c_str());

      this->postEvent<EvUnhookPathSucceeded>();
      return;
    }

    std::string message = "unknown error";
    if (wrapped_result.result) {
      message = wrapped_result.result->message;
    }

    writeLastErrorReport(
      getLogger(),
      "StRunUnhookDemo",
      "Path action failed: " + message,
      "Check /abb4600/run_robot_path action server terminal.\n\n"
      "If TrajectoryGuard rejected the trajectory, reduce velocity_scale/acceleration_scale, "
      "change path waypoints, or move robot to a safer start posture.");

    this->postEvent<EvUnhookPathFailed>();
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StRunUnhookDemo] Leaving state.");
  }
};

}  // namespace sm_abb4600_unhook
