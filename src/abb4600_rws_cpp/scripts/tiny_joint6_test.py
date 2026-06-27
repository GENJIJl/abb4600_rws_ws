#!/usr/bin/env python3
"""Tiny, low-speed joint_6 motion test for ABB EGM ros2_control."""

import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
DEFAULT_DELTA_DEG = 30.0
MAX_DELTA_DEG = 30.0
MOVE_OUT_SEC = 20
MOVE_BACK_SEC = 40


class TinyJoint6Test(Node):
    def __init__(self, delta_deg: float) -> None:
        super().__init__("abb_tiny_joint6_test")
        self.delta_rad = math.radians(delta_deg)
        self.current_positions = None
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

    def joint_state_cb(self, msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))
        if all(joint in positions_by_name for joint in JOINTS):
            self.current_positions = [positions_by_name[joint] for joint in JOINTS]

    def wait_for_joint_states(self) -> None:
        deadline = self.get_clock().now().nanoseconds + 5_000_000_000
        while (
            rclpy.ok()
            and self.current_positions is None
            and self.get_clock().now().nanoseconds < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_positions is None:
            raise RuntimeError("No complete /joint_states message received; aborting.")

    def send_test_trajectory(self) -> int:
        self.wait_for_joint_states()

        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("joint_trajectory_controller action server is not available.")

        start = list(self.current_positions)
        target = list(start)
        target[5] += self.delta_rad

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

        delta_deg = math.degrees(self.delta_rad)
        self.get_logger().info(
            f"Sending test trajectory: joint_6 +{delta_deg:.3f} deg in {MOVE_OUT_SEC} s, "
            f"then return by {MOVE_BACK_SEC} s."
        )

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal was rejected by the controller.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"Test finished. Controller result code: {result.error_code}")
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
            raise RuntimeError(f"Delta must be > 0 and <= {MAX_DELTA_DEG:.1f} degrees for this field test.")

        rclpy.init()
        node = TinyJoint6Test(delta_deg)
        try:
            result_code = node.send_test_trajectory()
        finally:
            node.destroy_node()
            rclpy.shutdown()

        return 0 if result_code == 0 else 2
    except Exception as exc:  # noqa: BLE001 - keep field failure message direct.
        print(f"ERROR: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1


if __name__ == "__main__":
    sys.exit(main())
