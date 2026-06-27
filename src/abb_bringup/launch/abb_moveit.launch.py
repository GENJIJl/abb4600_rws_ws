import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    # 根据包名找到 share 目录，再读取其中的 YAML 文件。
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    # Command-line arguments
    # 这些参数必须在运行 ros2 launch 时传入，用来选择机器人和 MoveIt 配置。
    robot_xacro_file = LaunchConfiguration("robot_xacro_file")
    support_package = LaunchConfiguration("support_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")

    # MoveIt 配置
    # MoveItConfigsBuilder 会把 URDF、SRDF、kinematics、joint limits 和 controller 配置合成参数。
    moveit_config = (
        MoveItConfigsBuilder(
            "abb_bringup", package_name=f"{moveit_config_package.perform(context)}"
        )
        # robot_description 来自机器人 support 包中的 Xacro。
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory(f"{support_package.perform(context)}"),
                "urdf",
                f"{robot_xacro_file.perform(context)}",
            )
        )
        # robot_description_semantic 来自 MoveIt 配置包中的 SRDF。
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory(
                    f"{moveit_config_package.perform(context)}"
                ),
                "config",
                f"{moveit_config_file.perform(context)}",
            )
        )
        # planning_pipelines 加载 MoveIt 的规划流水线配置。
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        # robot_description_kinematics 加载运动学求解器参数。
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory(
                    f"{moveit_config_package.perform(context)}"
                ),
                "config",
                "kinematics.yaml",
            )
        )
        # MoveIt 不会自动处理控制器切换
        # trajectory_execution 指定 MoveIt 通过哪个控制器执行轨迹。
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory(
                    f"{moveit_config_package.perform(context)}"
                ),
                "config",
                "moveit_controllers.yaml",
            ),
            moveit_manage_controllers=False,
        )
        # planning_scene_monitor 控制规划场景、几何、状态和 TF 的发布行为。
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        # joint_limits 加载 MoveIt 使用的关节限制。
        .joint_limits(
            file_path=os.path.join(
                get_package_share_directory(
                    f"{moveit_config_package.perform(context)}"
                ),
                "config",
                "joint_limits.yaml",
            )
        )
        .to_moveit_configs()
    )

    # 启动实际的 move_group 节点/action server
    # move_group 是 MoveIt 的核心节点，负责规划和执行接口。
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # RViz
    # RViz 读取 MoveIt 参数，用于交互式规划和可视化。
    rviz_base = os.path.join(
        get_package_share_directory(f"{moveit_config_package.perform(context)}"), "rviz"
    )
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Static TF
    # 静态 TF 将 world 坐标系连接到机器人 base_link。
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    # robot_state_publisher 根据 robot_description 和 joint_states 发布机器人 TF。
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # MoveIt 启动时需要 move_group、RViz、静态 TF 和 robot_state_publisher。
    nodes_to_start = [move_group_node, rviz_node, static_tf_node, robot_state_pub_node]
    return nodes_to_start


def generate_launch_description():
    # 这里声明 MoveIt launch 必需的命令行参数。
    declared_arguments = []

    # TODO(andyz): add other options
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_xacro_file",
            description="Xacro describing the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            description="Name of the SRDF file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
