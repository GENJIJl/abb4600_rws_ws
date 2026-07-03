import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    velocity_scale = LaunchConfiguration("velocity_scale")
    acceleration_scale = LaunchConfiguration("acceleration_scale")
    planning_time = LaunchConfiguration("planning_time")
    eef_step = LaunchConfiguration("eef_step")
    minimum_cartesian_fraction = LaunchConfiguration("minimum_cartesian_fraction")
    return_to_target_210 = LaunchConfiguration("return_to_target_210")
    skip_start_execute = LaunchConfiguration("skip_start_execute")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "velocity_scale": ParameterValue(velocity_scale, value_type=float),
                    "acceleration_scale": ParameterValue(acceleration_scale, value_type=float),
                    "planning_time": ParameterValue(planning_time, value_type=float),
                    "eef_step": ParameterValue(eef_step, value_type=float),
                    "minimum_cartesian_fraction": ParameterValue(minimum_cartesian_fraction, value_type=float),
                    "return_to_target_210": ParameterValue(return_to_target_210, value_type=bool),
                    "skip_start_execute": ParameterValue(skip_start_execute, value_type=bool),
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("velocity_scale", default_value="0.02"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.02"),
            DeclareLaunchArgument("planning_time", default_value="10.0"),
            DeclareLaunchArgument("eef_step", default_value="0.02"),
            DeclareLaunchArgument("minimum_cartesian_fraction", default_value="0.90"),
            DeclareLaunchArgument("return_to_target_210", default_value="true"),
            DeclareLaunchArgument("skip_start_execute", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
