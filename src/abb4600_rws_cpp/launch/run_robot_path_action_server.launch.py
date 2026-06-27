import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    support_package = "abb_irb4600_support"
    moveit_config_package = "abb_irb4600_60_205_moveit_config"

    moveit_config = (
        MoveItConfigsBuilder("abb_bringup", package_name=moveit_config_package)
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory(support_package),
                "urdf",
                "irb4600_60_205.xacro",
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory(moveit_config_package),
                "config",
                "abb_irb4600_60_205.srdf.xacro",
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory(moveit_config_package),
                "config",
                "kinematics.yaml",
            )
        )
        .joint_limits(
            file_path=os.path.join(
                get_package_share_directory(moveit_config_package),
                "config",
                "joint_limits.yaml",
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory(moveit_config_package),
                "config",
                "moveit_controllers.yaml",
            ),
            moveit_manage_controllers=False,
        )
        .planning_pipelines()
        .to_moveit_configs()
    )

    return_home_params = {
        "return_home.configured": True,

        # 你的 Home 基本就是全 0 位
        "return_home.joints": [
            -0.9733208633465544,
            0.9750485225247161,
            -0.7990781561269128,
            -4.167562414053478,
            1.2223738961153938,
            0.21063526956876513,
        ],

        # 先不启用中间点。后面如果还报动态负载过高，再改成 True。
        "return_home.use_middle": False,

        "return_home.middle_joints": [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
    }

    return LaunchDescription(
        [
            Node(
                package="abb4600_rws_cpp",
                executable="run_robot_path_action_server",
                name="run_robot_path_action_server",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    return_home_params,
                ],
            )
        ]
    )
