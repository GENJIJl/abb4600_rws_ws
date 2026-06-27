#!/usr/bin/env python3
"""Visible joint_6 sweep test for ABB EGM through forward position commands."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
AMPLITUDE_DEG = 20.0
SEGMENT_SEC = 12.0
RATE_HZ = 25.0


class EGMJoint6Sweep(Node):
    def __init__(self) -> None:
        super().__init__("abb_egm_joint6_sweep_test")
        self.current_positions = None
        self.start_positions = None
        self.max_observed_delta = 0.0
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 20)
        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/forward_command_controller_position/commands",
            10,
        )

    def joint_state_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(joint in positions for joint in JOINTS):
            self.current_positions = [positions[joint] for joint in JOINTS]
            if self.start_positions is not None:
                self.max_observed_delta = max(
                    self.max_observed_delta,
                    abs(self.current_positions[5] - self.start_positions[5]),
                )

    def wait_for_joint_states(self) -> None:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and self.current_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_positions is None:
            raise RuntimeError("No complete /joint_states message received.")

    def wait_for_forward_controller(self) -> None:
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            "No subscriber on /forward_command_controller_position/commands. "
            "Activate forward_command_controller_position first."
        )

    def publish_segment(self, target_joint6: float, duration_sec: float) -> None:
        start = list(self.current_positions)
        target = list(start)
        target[5] = target_joint6
        start_time = time.monotonic()
        next_log = start_time
        period = 1.0 / RATE_HZ

        while rclpy.ok():
            now = time.monotonic()
            if now >= start_time + duration_sec:
                break

            ratio = (now - start_time) / duration_sec
            ratio = max(0.0, min(1.0, ratio))
            cmd = [start[i] + (target[i] - start[i]) * ratio for i in range(len(JOINTS))]
            self.publisher.publish(Float64MultiArray(data=cmd))
            rclpy.spin_once(self, timeout_sec=0.0)

            if now >= next_log and self.current_positions is not None:
                self.get_logger().info(
                    "joint_6 target %.2f deg, feedback %.2f deg"
                    % (math.degrees(cmd[5]), math.degrees(self.current_positions[5]))
                )
                next_log = now + 1.0

            time.sleep(period)

        self.publisher.publish(Float64MultiArray(data=target))

    def run(self) -> None:
        self.wait_for_joint_states()
        self.wait_for_forward_controller()
        self.start_positions = list(self.current_positions)
        self.max_observed_delta = 0.0

        start_j6 = self.start_positions[5]
        amplitude = math.radians(AMPLITUDE_DEG)
        self.get_logger().info(
            "Starting EGM joint_6 sweep: +%.1f deg, -%.1f deg, then back to start."
            % (AMPLITUDE_DEG, AMPLITUDE_DEG)
        )
        self.publish_segment(start_j6 + amplitude, SEGMENT_SEC)
        self.publish_segment(start_j6 - amplitude, SEGMENT_SEC * 2.0)
        self.publish_segment(start_j6, SEGMENT_SEC)

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        observed = math.degrees(self.max_observed_delta)
        self.get_logger().info("Observed joint_6 max delta: %.2f deg" % observed)
        if observed < AMPLITUDE_DEG * 0.5:
            raise RuntimeError("RobotStudio feedback did not follow the EGM sweep.")


def main() -> int:
    rclpy.init()
    node = EGMJoint6Sweep()
    try:
        node.run()
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
