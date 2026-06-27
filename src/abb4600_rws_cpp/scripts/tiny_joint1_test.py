#!/usr/bin/env python3
"""Visible, feedback-checked joint_1 motion test through joint_trajectory_controller."""

import math
import sys
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
DEFAULT_DELTA_DEG = 10.0
MAX_DELTA_DEG = 15.0
MOVE_OUT_SEC = 12
MOVE_BACK_SEC = 24
MIN_OBSERVED_DELTA_DEG = 2.0


class TinyJoint1Test(Node):
    def __init__(self, delta_deg: float) -> None:
        super().__init__("abb_tiny_joint1_test")
        self.delta_rad = math.radians(delta_deg)
        self.current_positions = None
        self.start_joint_1 = None
        self.max_observed_delta = 0.0
        self._lock = threading.Lock()

        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

    def joint_state_cb(self, msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))
        if not all(joint in positions_by_name for joint in JOINTS):
            return

        positions = [positions_by_name[joint] for joint in JOINTS]
        with self._lock:
            self.current_positions = positions
            if self.start_joint_1 is not None:
                self.max_observed_delta = max(
                    self.max_observed_delta,
                    abs(positions[0] - self.start_joint_1),
                )

    def wait_for_joint_states(self) -> None:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and self.current_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_positions is None:
            raise RuntimeError("No complete /joint_states message received; aborting.")

    def send_test_trajectory(self) -> int:
        self.wait_for_joint_states()

        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("joint_trajectory_controller action server is not available.")

        with self._lock:
            start = list(self.current_positions)
            self.start_joint_1 = start[0]
            self.max_observed_delta = 0.0

        target = list(start)
        target[0] += self.delta_rad

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINTS

        point_1 = JointTrajectoryPoint()
        point_1.positions = target
        point_1.velocities = [0.0] * len(JOINTS)
        point_1.time_from_start = Duration(sec=MOVE_OUT_SEC)

        point_2 = JointTrajectoryPoint()
        point_2.positions = start
        point_2.velocities = [0.0] * len(JOINTS)
        point_2.time_from_start = Duration(sec=MOVE_BACK_SEC)

        trajectory.points = [point_1, point_2]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            "Sending visible test trajectory: joint_1 +%.3f deg in %d s, then return by %d s."
            % (math.degrees(self.delta_rad), MOVE_OUT_SEC, MOVE_BACK_SEC)
        )

        send_future = self.client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal was rejected by the controller.")

        result_future = goal_handle.get_result_async()
        last_log_time = 0.0
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_log_time >= 1.0:
                last_log_time = now
                with self._lock:
                    if self.current_positions is not None:
                        feedback_deg = math.degrees(self.current_positions[0])
                        observed_deg = math.degrees(self.max_observed_delta)
                        self.get_logger().info(
                            "joint_1 feedback %.2f deg, observed max delta %.2f deg"
                            % (feedback_deg, observed_deg)
                        )

        result = result_future.result().result
        observed_deg = math.degrees(self.max_observed_delta)
        self.get_logger().info("Controller result code: %d" % result.error_code)
        self.get_logger().info("Observed joint_1 max delta: %.2f deg" % observed_deg)

        if observed_deg < MIN_OBSERVED_DELTA_DEG:
            raise RuntimeError(
                "Robot feedback did not follow the trajectory. "
                "This indicates EGM/RobotStudio did not execute the command."
            )

        return result.error_code


def parse_delta_deg() -> float:
    if len(sys.argv) <= 1:
        return DEFAULT_DELTA_DEG
    try:
        return float(sys.argv[1])
    except ValueError as exc:
        raise RuntimeError(f"Invalid delta degree value: {sys.argv[1]!r}") from exc


def main() -> int:
    try:
        delta_deg = parse_delta_deg()
        if delta_deg <= 0.0 or delta_deg > MAX_DELTA_DEG:
            raise RuntimeError(f"Delta must be > 0 and <= {MAX_DELTA_DEG:.1f} degrees.")

        rclpy.init()
        node = TinyJoint1Test(delta_deg)
        try:
            result_code = node.send_test_trajectory()
        finally:
            node.destroy_node()
            rclpy.shutdown()

        return 0 if result_code == 0 else 2
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1


if __name__ == "__main__":
    sys.exit(main())
