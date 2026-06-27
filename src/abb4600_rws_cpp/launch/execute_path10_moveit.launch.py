import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    velocity_scale = LaunchConfiguration("velocity_scale")
    acceleration_scale = LaunchConfiguration("acceleration_scale")
    planning_time = LaunchConfiguration("planning_time")
    eef_step = LaunchConfiguration("eef_step")
    minimum_cartesian_fraction = LaunchConfiguration("minimum_cartesian_fraction")
    return_to_target_210 = LaunchConfiguration("return_to_target_210")

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

    return [
        Node(
            package="abb4600_rws_cpp",
            executable="execute_path10_moveit_node",
            name="execute_path10_moveit_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "velocity_scale": velocity_scale,
                    "acceleration_scale": acceleration_scale,
                    "planning_time": planning_time,
                    "eef_step": eef_step,
                    "minimum_cartesian_fraction": minimum_cartesian_fraction,
                    "return_to_target_210": return_to_target_210,
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("velocity_scale", default_value="0.02"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.02"),
            DeclareLaunchArgument("planning_time", default_value="10.0"),
            DeclareLaunchArgument("eef_step", default_value="0.02"),
            DeclareLaunchArgument("minimum_cartesian_fraction", default_value="0.90"),
            DeclareLaunchArgument("return_to_target_210", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
