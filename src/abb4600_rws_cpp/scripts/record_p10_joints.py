#!/usr/bin/env python3
import json
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState


JOINT_ORDER = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


def main():
    rclpy.init()

    node = rclpy.create_node("record_p10_joints")

    output_path = Path.home() / "abb4600_rws_ws" / "config" / "p10_joints.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    saved = {"done": False}

    def callback(msg: JointState):
        name_to_position = {}

        for name, position in zip(msg.name, msg.position):
            name_to_position[name] = float(position)

        missing = [name for name in JOINT_ORDER if name not in name_to_position]

        if missing:
            node.get_logger().error(f"Missing joints in /joint_states: {missing}")
            return

        p10 = {
            name: name_to_position[name]
            for name in JOINT_ORDER
        }

        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(p10, f, indent=2)

        node.get_logger().info("Saved p10 joint target:")
        for name in JOINT_ORDER:
            node.get_logger().info(f"  {name}: {p10[name]:.9f}")

        node.get_logger().info(f"File: {output_path}")
        saved["done"] = True

    node.create_subscription(
        JointState,
        "/joint_states",
        callback,
        10
    )

    node.get_logger().info("Waiting for one /joint_states message...")

    while rclpy.ok() and not saved["done"]:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
