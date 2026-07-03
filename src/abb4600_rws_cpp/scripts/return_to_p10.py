#!/usr/bin/env python3
import json
import sys
from pathlib import Path

import rclpy
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_ORDER = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


class ReturnToP10JointClient:
    def __init__(self):
        self.node = rclpy.create_node("return_to_p10_joint_client")

        self.client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory"
        )

        self.p10_file = (
            Path.home() / "abb4600_rws_ws" / "config" / "p10_joints.json"
        )

    def load_p10(self):
        if not self.p10_file.exists():
            raise FileNotFoundError(
                f"p10 joint file not found: {self.p10_file}\n"
                "Please move robot to p10 first, then run:\n"
                "ros2 run abb4600_rws_cpp record_p10_joints.py"
            )

        with open(self.p10_file, "r", encoding="utf-8") as f:
            data = json.load(f)

        missing = [name for name in JOINT_ORDER if name not in data]
        if missing:
            raise RuntimeError(f"p10 file missing joints: {missing}")

        positions = [float(data[name]) for name in JOINT_ORDER]

        self.node.get_logger().info("Loaded p10 joint target:")
        for name, value in zip(JOINT_ORDER, positions):
            self.node.get_logger().info(f"  {name}: {value:.9f}")

        return positions

    def send_goal(self):
        positions = self.load_p10()

        self.node.get_logger().info(
            "Waiting for /joint_trajectory_controller/follow_joint_trajectory..."
        )

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(
                "Action server /joint_trajectory_controller/follow_joint_trajectory not available."
            )
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_ORDER

        point = JointTrajectoryPoint()
        point.positions = positions

        # 真实机械臂归位要慢，先给 30 秒。
        # 如果距离很远，可以改成 40 或 60。
        point.time_from_start.sec = 30
        point.time_from_start.nanosec = 0

        goal_msg.trajectory.points.append(point)

        self.node.get_logger().warn(
            "Sending joint-space return-to-p10 trajectory. "
            "Make sure teach pendant is in safe mode and speed override is low."
        )

        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.node.get_logger().error("Failed to send goal.")
            return False

        if not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected by controller.")
            return False

        self.node.get_logger().info("Goal accepted. Waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        wrapped_result = result_future.result()
        result = wrapped_result.result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.node.get_logger().info("Return to p10 succeeded.")
            return True

        self.node.get_logger().error(
            f"Return to p10 failed. error_code={result.error_code}, "
            f"error_string={result.error_string}"
        )
        return False


def main():
    rclpy.init(args=sys.argv)

    client = ReturnToP10JointClient()

    try:
        ok = client.send_goal()
    except Exception as e:
        client.node.get_logger().error(str(e))
        ok = False

    client.node.destroy_node()
    rclpy.shutdown()

    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
