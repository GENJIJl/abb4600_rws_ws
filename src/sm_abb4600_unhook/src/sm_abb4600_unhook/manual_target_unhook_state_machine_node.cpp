#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <abb4600_interfaces/action/run_robot_path.hpp>

using namespace std::chrono_literals;

namespace sm_abb4600_unhook
{

class ManualTargetUnhookStateMachine : public rclcpp::Node
{
public:
  using RunRobotPath = abb4600_interfaces::action::RunRobotPath;
  using GoalHandleRunRobotPath = rclcpp_action::ClientGoalHandle<RunRobotPath>;

  enum class State
  {
    IDLE,
    WAIT_OPERATOR_CONFIRM,
    WAIT_MANUAL_TARGET,
    MOVING_TO_TARGET,
    EXECUTING_PRESET_UNHOOK_PATH,
    WAIT_MANUAL_VERIFY,
    WAIT_RETURN_HOME,
    RETURNING_HOME,
    DONE,
    ERROR
  };

  ManualTargetUnhookStateMachine()
  : Node("manual_target_unhook_state_machine")
  {
    action_client_ = rclcpp_action::create_client<RunRobotPath>(
      this,
      "/abb4600/run_robot_path");

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/start",
      rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) { onStart(); });

    confirm_start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/confirm_start",
      rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) { onConfirmStart(); });

    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/cancel",
      rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) { onCancel(); });

    reset_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/reset",
      rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) { onReset(); });

    manual_target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/abb4600_unhook/manual_target",
      rclcpp::QoS(10),
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) { onManualTarget(msg); });

    unhook_success_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/unhook_success",
      rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) { onUnhookSuccess(); });

    unhook_failed_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/unhook_failed",
      rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) { onUnhookFailed(); });

    return_home_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/return_home",
      rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) { onReturnHome(); });

    skip_return_home_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/abb4600_unhook/skip_return_home",
      rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) { onSkipReturnHome(); });

    state_timer_ = create_wall_timer(
      1s,
      [this]() { printStateHeartbeat(); });

    RCLCPP_INFO(get_logger(), "Manual target unhook state machine started.");
    RCLCPP_INFO(get_logger(), "Start:          ros2 topic pub -1 /abb4600_unhook/start std_msgs/msg/Empty \"{}\"");
    RCLCPP_INFO(get_logger(), "Confirm:        ros2 topic pub -1 /abb4600_unhook/confirm_start std_msgs/msg/Empty \"{}\"");
    RCLCPP_INFO(get_logger(), "Manual target:  ros2 topic pub -1 /abb4600_unhook/manual_target geometry_msgs/msg/PointStamped \"{header: {frame_id: 'base_link'}, point: {x: 1.594, y: 0.819, z: 1.050}}\"");
  }

private:
  // p720 成功姿态，ROS 四元数顺序 qx,qy,qz,qw
  static constexpr double kFixedQx = -0.059987;
  static constexpr double kFixedQy =  0.769460;
  static constexpr double kFixedQz = -0.052134;
  static constexpr double kFixedQw =  0.633731;

  // 预设摘钩路径偏移，单位 mm
  // 当前只是最小验证版：先上抬 80 mm，再沿 x 负方向退出 120 mm
  static constexpr double kLiftZMm = 80.0;
  static constexpr double kRetreatXMm = -120.0;

  // 目标点允许范围，单位 m
  static constexpr double kMinX = 1.20;
  static constexpr double kMaxX = 1.80;
  static constexpr double kMinY = 0.30;
  static constexpr double kMaxY = 1.10;
  static constexpr double kMinZ = 0.70;
  static constexpr double kMaxZ = 1.40;

  std::mutex mutex_;
  State state_{State::IDLE};

  double target_x_m_{0.0};
  double target_y_m_{0.0};
  double target_z_m_{0.0};

  rclcpp_action::Client<RunRobotPath>::SharedPtr action_client_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr confirm_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr manual_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr unhook_success_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr unhook_failed_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr return_home_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr skip_return_home_sub_;

  rclcpp::TimerBase::SharedPtr state_timer_;

  std::string stateName(State s) const
  {
    switch (s) {
      case State::IDLE: return "IDLE";
      case State::WAIT_OPERATOR_CONFIRM: return "WAIT_OPERATOR_CONFIRM";
      case State::WAIT_MANUAL_TARGET: return "WAIT_MANUAL_TARGET";
      case State::MOVING_TO_TARGET: return "MOVING_TO_TARGET";
      case State::EXECUTING_PRESET_UNHOOK_PATH: return "EXECUTING_PRESET_UNHOOK_PATH";
      case State::WAIT_MANUAL_VERIFY: return "WAIT_MANUAL_VERIFY";
      case State::WAIT_RETURN_HOME: return "WAIT_RETURN_HOME";
      case State::RETURNING_HOME: return "RETURNING_HOME";
      case State::DONE: return "DONE";
      case State::ERROR: return "ERROR";
    }
    return "UNKNOWN";
  }

  void setState(State next)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == next) {
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "[FSM] %s -> %s",
      stateName(state_).c_str(),
      stateName(next).c_str());

    state_ = next;
  }

  State getState()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  void printStateHeartbeat()
  {
    const auto s = getState();

    if (s == State::WAIT_OPERATOR_CONFIRM) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[FSM] Waiting confirm_start: ros2 topic pub -1 /abb4600_unhook/confirm_start std_msgs/msg/Empty \"{}\"");
    } else if (s == State::WAIT_MANUAL_TARGET) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[FSM] Waiting manual target: ros2 topic pub -1 /abb4600_unhook/manual_target geometry_msgs/msg/PointStamped \"{header: {frame_id: 'base_link'}, point: {x: 1.594, y: 0.819, z: 1.050}}\"");
    } else if (s == State::WAIT_MANUAL_VERIFY) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[FSM] Waiting verify: /abb4600_unhook/unhook_success or /abb4600_unhook/unhook_failed");
    } else if (s == State::WAIT_RETURN_HOME) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[FSM] Waiting return choice: /abb4600_unhook/return_home or /abb4600_unhook/skip_return_home");
    }
  }

  bool isTargetValid(double x, double y, double z, std::string & reason)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      reason = "target contains NaN or Inf";
      return false;
    }

    if (x < kMinX || x > kMaxX) {
      reason = "x out of range";
      return false;
    }

    if (y < kMinY || y > kMaxY) {
      reason = "y out of range";
      return false;
    }

    if (z < kMinZ || z > kMaxZ) {
      reason = "z out of range";
      return false;
    }

    return true;
  }

  std::string makeSafeTcpBaseMmPath(
    double x_mm,
    double y_mm,
    double z_mm)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "safe_tcp_base_mm:"
        << x_mm << ","
        << y_mm << ","
        << z_mm << ","
        << std::setprecision(6)
        << kFixedQx << ","
        << kFixedQy << ","
        << kFixedQz << ","
        << kFixedQw;
    return oss.str();
  }

  RunRobotPath::Goal makeGoal(const std::string & path_name)
  {
    RunRobotPath::Goal goal;
    goal.path_name = path_name;
    goal.velocity_scale = 0.02;
    goal.acceleration_scale = 0.02;
    goal.planning_time = 60.0;
    goal.eef_step = 0.01;
    goal.minimum_cartesian_fraction = 0.80;
    goal.return_to_start = false;
    return goal;
  }

  void sendRunRobotPathGoal(
    const std::string & path_name,
    std::function<void(bool, const std::string &)> done_cb)
  {
    if (!action_client_->wait_for_action_server(5s)) {
      done_cb(false, "Action server /abb4600/run_robot_path not available");
      return;
    }

    auto goal = makeGoal(path_name);

    RCLCPP_WARN(
      get_logger(),
      "[FSM] Sending action path_name='%s'",
      goal.path_name.c_str());

    auto send_goal_options =
      rclcpp_action::Client<RunRobotPath>::SendGoalOptions();

    send_goal_options.feedback_callback =
      [this](
        GoalHandleRunRobotPath::SharedPtr,
        const std::shared_ptr<const RunRobotPath::Feedback> feedback)
      {
        RCLCPP_INFO(
          get_logger(),
          "[RunRobotPath feedback] %.2f %s",
          feedback->progress,
          feedback->current_stage.c_str());
      };

    send_goal_options.result_callback =
      [this, done_cb](
        const GoalHandleRunRobotPath::WrappedResult & result)
      {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          std::ostringstream oss;
          oss << "Action finished with non-success code: "
              << static_cast<int>(result.code);
          done_cb(false, oss.str());
          return;
        }

        if (!result.result) {
          done_cb(false, "Action result is null");
          return;
        }

        done_cb(result.result->success, result.result->message);
      };

    action_client_->async_send_goal(goal, send_goal_options);
  }

  void onStart()
  {
    const auto s = getState();

    if (s != State::IDLE && s != State::DONE && s != State::ERROR) {
      RCLCPP_WARN(
        get_logger(),
        "[FSM] Ignore start because current state is %s",
        stateName(s).c_str());
      return;
    }

    RCLCPP_WARN(get_logger(), "[FSM] Start received.");
    setState(State::WAIT_OPERATOR_CONFIRM);
  }

  void onConfirmStart()
  {
    const auto s = getState();

    if (s != State::WAIT_OPERATOR_CONFIRM) {
      RCLCPP_WARN(
        get_logger(),
        "[FSM] Ignore confirm_start because current state is %s",
        stateName(s).c_str());
      return;
    }

    RCLCPP_WARN(get_logger(), "[FSM] Confirm start received. Waiting manual target.");
    setState(State::WAIT_MANUAL_TARGET);
  }

  void onManualTarget(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const auto s = getState();

    if (s != State::WAIT_MANUAL_TARGET) {
      RCLCPP_WARN(
        get_logger(),
        "[FSM] Ignore manual_target because current state is %s",
        stateName(s).c_str());
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != "base_link") {
      RCLCPP_ERROR(
        get_logger(),
        "[FSM] manual_target frame_id must be base_link, got '%s'",
        msg->header.frame_id.c_str());
      setState(State::ERROR);
      return;
    }

    const double x = msg->point.x;
    const double y = msg->point.y;
    const double z = msg->point.z;

    std::string reason;
    if (!isTargetValid(x, y, z, reason)) {
      RCLCPP_ERROR(
        get_logger(),
        "[FSM] Invalid manual target x=%.3f y=%.3f z=%.3f, reason=%s",
        x, y, z, reason.c_str());
      setState(State::ERROR);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      target_x_m_ = x;
      target_y_m_ = y;
      target_z_m_ = z;
    }

    RCLCPP_WARN(
      get_logger(),
      "[FSM] Manual target accepted: x=%.3f y=%.3f z=%.3f m",
      x, y, z);

    setState(State::MOVING_TO_TARGET);
    executeMoveToManualTarget();
  }

  void executeMoveToManualTarget()
  {
    double x_mm;
    double y_mm;
    double z_mm;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      x_mm = target_x_m_ * 1000.0;
      y_mm = target_y_m_ * 1000.0;
      z_mm = target_z_m_ * 1000.0;
    }

    const std::string path_name = makeSafeTcpBaseMmPath(x_mm, y_mm, z_mm);

    sendRunRobotPathGoal(
      path_name,
      [this](bool success, const std::string & message)
      {
        if (!success) {
          RCLCPP_ERROR(
            get_logger(),
            "[FSM] Move to manual target failed: %s",
            message.c_str());
          setState(State::ERROR);
          return;
        }

        RCLCPP_WARN(
          get_logger(),
          "[FSM] Move to manual target succeeded: %s",
          message.c_str());

        setState(State::EXECUTING_PRESET_UNHOOK_PATH);
        executePresetUnhookPathStep1();
      });
  }

  void executePresetUnhookPathStep1()
  {
    double x_mm;
    double y_mm;
    double z_mm;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      x_mm = target_x_m_ * 1000.0;
      y_mm = target_y_m_ * 1000.0;
      z_mm = target_z_m_ * 1000.0 + kLiftZMm;
    }

    const std::string path_name = makeSafeTcpBaseMmPath(x_mm, y_mm, z_mm);

    RCLCPP_WARN(
      get_logger(),
      "[FSM] Preset unhook step 1: lift z by %.1f mm",
      kLiftZMm);

    sendRunRobotPathGoal(
      path_name,
      [this](bool success, const std::string & message)
      {
        if (!success) {
          RCLCPP_ERROR(
            get_logger(),
            "[FSM] Preset unhook step 1 failed: %s",
            message.c_str());
          setState(State::ERROR);
          return;
        }

        RCLCPP_WARN(
          get_logger(),
          "[FSM] Preset unhook step 1 succeeded: %s",
          message.c_str());

        executePresetUnhookPathStep2();
      });
  }

  void executePresetUnhookPathStep2()
  {
    double x_mm;
    double y_mm;
    double z_mm;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      x_mm = target_x_m_ * 1000.0 + kRetreatXMm;
      y_mm = target_y_m_ * 1000.0;
      z_mm = target_z_m_ * 1000.0 + kLiftZMm;
    }

    const std::string path_name = makeSafeTcpBaseMmPath(x_mm, y_mm, z_mm);

    RCLCPP_WARN(
      get_logger(),
      "[FSM] Preset unhook step 2: retreat x by %.1f mm",
      kRetreatXMm);

    sendRunRobotPathGoal(
      path_name,
      [this](bool success, const std::string & message)
      {
        if (!success) {
          RCLCPP_ERROR(
            get_logger(),
            "[FSM] Preset unhook step 2 failed: %s",
            message.c_str());
          setState(State::ERROR);
          return;
        }

        RCLCPP_WARN(
          get_logger(),
          "[FSM] Preset unhook path finished: %s",
          message.c_str());

        setState(State::WAIT_MANUAL_VERIFY);
      });
  }

  void onUnhookSuccess()
  {
    const auto s = getState();

    if (s != State::WAIT_MANUAL_VERIFY) {
      RCLCPP_WARN(
        get_logger(),
        "[FSM] Ignore unhook_success because current state is %s",
        stateName(s).c_str());
      return;
    }

    RCLCPP_WARN(get_logger(), "[FSM] Operator confirmed unhook success.");
    setState(State::WAIT_RETURN_HOME);
  }

  void onUnhookFailed()
  {
    const auto s = getState();

    if (s != State::WAIT_MANUAL_VERIFY) {
      RCLCPP_WARN(
        get_logger(),
        "[FSM] Ignore unhook_failed because current state is %s",
        stateName(s).c_str());
      return;
    }

    RCLCPP_ERROR(get_logger(), "[FSM] Operator confirmed unhook failed. Waiting new manual target.");
    setState(State::WAIT_MANUAL_TARGET);
  }

  void onReturnHome()
  {
    const auto s = getState();

    if (s != State::WAIT_RETURN_HOME) {
      RCLCPP_WARN(
        get_logger(),
        "[FSM] Ignore return_home because current state is %s",
        stateName(s).c_str());
      return;
    }

    setState(State::RETURNING_HOME);

    sendRunRobotPathGoal(
      "return_home",
      [this](bool success, const std::string & message)
      {
        if (!success) {
          RCLCPP_ERROR(
            get_logger(),
            "[FSM] return_home failed: %s",
            message.c_str());
          setState(State::ERROR);
          return;
        }

        RCLCPP_WARN(
          get_logger(),
          "[FSM] return_home succeeded: %s",
          message.c_str());
        setState(State::DONE);
      });
  }

  void onSkipReturnHome()
  {
    const auto s = getState();

    if (s != State::WAIT_RETURN_HOME) {
      RCLCPP_WARN(
        get_logger(),
        "[FSM] Ignore skip_return_home because current state is %s",
        stateName(s).c_str());
      return;
    }

    RCLCPP_WARN(get_logger(), "[FSM] Skip return_home. Task done.");
    setState(State::DONE);
  }

  void onCancel()
  {
    RCLCPP_ERROR(get_logger(), "[FSM] Cancel received. Go ERROR.");
    setState(State::ERROR);
  }

  void onReset()
  {
    RCLCPP_WARN(get_logger(), "[FSM] Reset received. Go IDLE.");
    setState(State::IDLE);
  }
};

}  // namespace sm_abb4600_unhook

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<sm_abb4600_unhook::ManualTargetUnhookStateMachine>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
