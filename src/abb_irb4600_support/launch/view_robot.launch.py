# 在 RViz 中查看 URDF 机器人模型。

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def load_file(package_name, file_path):
    # 根据包名找到 share 目录，再读取指定文本文件。
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # IOError、OSError 以及部分平台上的 WindowsError 的父类。
        return None


def load_yaml(package_name, file_path):
    # 根据包名找到 share 目录，再读取指定 YAML 文件。
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # IOError、OSError 以及部分平台上的 WindowsError 的父类。
        return None


def generate_launch_description():
    # description_file 用来选择要查看的 IRB4600 型号 Xacro。
    description_file_arg = DeclareLaunchArgument(
        "description_file",
        default_value=TextSubstitution(text="irb4600_60_205.xacro"),
        description="support 包中受支持的机器人描述文件名。",
    )
    description_file = LaunchConfiguration("description_file")

    # robot_description_content 通过 xacro 命令把 Xacro 展开成完整 URDF。
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("abb_irb4600_support"), "urdf", description_file]
            ),
        ]
    )

    # robot_description 是 robot_state_publisher 读取的标准参数名。
    robot_description = {"robot_description": robot_description_content}

    # robot_state_publisher 根据 robot_description 和 joint_states 发布 TF。
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # joint_state_publisher_gui 提供滑块，方便手动改变关节角查看模型。
    joint_state_sliders = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # RViz 读取预设配置文件显示机器人模型和 TF。
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("abb_irb4600_support"),
                    "rviz",
                    "urdf_description.rviz",
                ]
            ),
        ],
        output="screen",
    )

    # 这个 launch 只用于看模型，不启动 ros2_control。
    return LaunchDescription(
        [description_file_arg, robot_state_publisher_node, joint_state_sliders, rviz]
    )
