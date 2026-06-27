# ABB4600 ros2_control + RobotStudio 配置说明


abb4600_rws_ws/
  src/
    abb4600_rws_cpp/              你自己当前主要开发的包
    abb_bringup/                  启动和控制器配置
    abb_hardware_interface/       ros2_control 到 ABB EGM 的硬件接口
    abb_egm_rws_managers/         管理 EGM/RWS 通信的 C++ 库
    abb_libegm/                   ABB EGM 底层通信库
    abb_librws/                   ABB RWS 底层通信库
    abb_rws_client/               RWS 客户端工具包
    abb_irb4600_support/          ABB4600 机器人模型
    abb_irb4600_60_205_moveit_config/  MoveIt 配置
    abb_resources/                ABB 通用机器人资源
    abb_ros2/                     ABB ROS2 元包
    abb_ros2_msgs/                ABB ROS2 消息包




当前工作空间中保留了两条 ABB 控制路径：

- `abb4600_rws_cpp`：轻量级 RWS 辅助节点，用于 `resetpp`、启动 RAPID 等管理动作。
- `abb_hardware_interface`：面向 ros2_control 的硬件接口插件，通过 RWS 和 EGM 连接 ABB 控制器。

通过 ros2_control 驱动 RobotStudio 仿真机械臂时，目标链路是：

```text
joint_trajectory_controller
  -> controller_manager
  -> abb_hardware_interface/ABBSystemHardware
  -> RWS + EGM
  -> RobotStudio 虚拟控制器
```

其中，RWS 是管理通道，负责控制器信息、状态、程序启动等操作；EGM 是运动命令和关节状态反馈通道。

## 必需系统依赖

`abb_librws` 需要 Poco 开发头文件：

```bash
sudo apt-get install -y libpoco-dev
```

安装完成后，构建工作空间：

```bash
cd /home/cjx/dev/abb4600_rws_ws
colcon build --symlink-install
source install/setup.bash
```

## 可直接复制执行的完整启动流程

下面按终端窗口拆分。每个代码块都包含 `cd` 和 `source`，可以直接复制执行。

启动前确认 RobotStudio 侧已经设置好 EGM：

```text
Name: ROB_1
Type: UDPUC
Remote Address: 120.55.45.142
Remote port number: 14601
Local port: 0
```


1. 启动 frpc

cd /home/cjx/Downloads/frp_0.61.0_linux_amd64
./frpc -c /home/cjx/dev/abb4600_rws_ws/src/abb4600_rws_cpp/frp/frpc_egm_ubuntu.toml
2. 启动 ros2_control，但先不激活轨迹控制器

不要先用 abb4600_robotstudio_control.launch.py，这里直接用底层 abb_control.launch.py，因为要传：

start_initial_joint_controller:=false
命令如下：

cd /home/cjx/dev/abb4600_rws_ws
source install/setup.bash

ros2 launch abb_bringup abb_control.launch.py \
  description_package:=abb_irb4600_support \
  description_file:=irb4600_60_205.xacro \
  moveit_config_package:=abb_irb4600_60_205_moveit_config \
  runtime_config_package:=abb_bringup \
  controllers_file:=abb_controllers.yaml \
  initial_joint_controller:=joint_trajectory_controller \
  start_initial_joint_controller:=false \
  rws_ip:=120.55.45.142 \
  rws_port:=15555 \
  egm_port:=6511 \
  use_fake_hardware:=false \
  launch_rviz:=false
3. RobotStudio 启动 EGM RAPID 程序

这一步运行你的 EGMTest.MOD 或现有 EGM 程序。

注意：如果 TRob0Main.mod 里面有类似：

MoveJ Target_210, ...
那它启动后一定会先动。要验证“EGM 打开后保持原位”，最好用没有 MoveJ/MoveL 的 EGM 程序。

4. 检查 EGM 和 joint_states

cd /home/cjx/dev/abb4600_rws_ws
source install/setup.bash

ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
确认 /joint_states 正常刷新，并且这一步机械臂不应该动。

需要抓 UDP 的话：

sudo tcpdump -ni any udp port 6511
5. 再激活轨迹控制器

ros2 control list_controllers

cd /home/cjx/dev/abb4600_rws_ws
source install/setup.bash

ros2 control load_controller --set-state active joint_trajectory_controller
检查：

ros2 control list_controllers
ros2 control list_hardware_interfaces

7. 启动 MoveIt

另开终端：

cd /home/cjx/dev/abb4600_rws_ws
source install/setup.bash

ros2 launch abb_bringup abb_moveit.launch.py \
  robot_xacro_file:=irb4600_60_205.xacro \
  support_package:=abb_irb4600_support \
  moveit_config_package:=abb_irb4600_60_205_moveit_config \
  moveit_config_file:=abb_irb4600_60_205.srdf.xacro
8. 运行 Path10

你原来的命令可以用，但我建议仿真中速度稍微快一点：

cd /home/cjx/dev/abb4600_rws_ws
source install/setup.bash

ros2 launch abb4600_rws_cpp execute_path10_moveit.launch.py \
  velocity_scale:=0.08 \
  acceleration_scale:=0.04 \
  planning_time:=10.0 \
  eef_step:=0.02 \
  minimum_cartesian_fraction:=0.90
如果 RobotStudio 报动态负载或 EGM 输出边界，再降回：

velocity_scale:=0.08
acceleration_scale:=0.04
关键判断点：

启动 ros2_control 后，机器人不动
启动 EGM RAPID 后，机器人仍不动
激活 joint_trajectory_controller 后，机器人仍不动
只有运行 tiny_joint6_test.py / MoveIt 轨迹后，机器人才动
如果在第 2 或第 3 步机器人就动了，那问题还在 EGM 初始回包或 RAPID 程序本身，不要继续跑 MoveIt。


cd /home/cjx/dev/abb4600_rws_ws
source install/setup.bash


ros2 launch abb4600_rws_cpp execute_train_hook_moveit.launch.py \
  procedure:=guiji_1 \
  use_exported_movej_jointtargets:=true \
  velocity_scale:=0.05 \
  acceleration_scale:=0.03 \
  trajectory_duration_scale:=1.5 \
  planning_time:=60.0 \
  eef_step:=0.01 \
  minimum_cartesian_fraction:=0.90 \
  validate_trajectory:=true \
  max_joint_step_deg:=120.0




