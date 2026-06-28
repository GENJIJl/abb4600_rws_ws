#!/usr/bin/env bash
set -e

cd ~/abb4600_rws_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

URDF=~/abb4600_rws_ws/generated_urdf/irb4600_60_205_gazebo.urdf

echo "Killing old Gazebo / ROS simulation processes..."
pkill -9 gzserver || true
pkill -9 gzclient || true
pkill -9 gazebo || true
pkill -9 robot_state_publisher || true
pkill -9 spawner || true

ros2 daemon stop || true
ros2 daemon start || true

echo "Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(cat $URDF)" \
  -r __node:=robot_state_publisher &

sleep 3

echo "Starting Gazebo..."
ros2 launch gazebo_ros gazebo.launch.py verbose:=true &

sleep 8

echo "Pausing physics..."
ros2 service call /pause_physics std_srvs/srv/Empty "{}" || true

echo "Spawning ABB4600..."
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity abb4600 \
  -x 0 -y 0 -z 0

sleep 3

echo "Unpausing physics..."
ros2 service call /unpause_physics std_srvs/srv/Empty "{}" || true

sleep 2

echo "Spawning joint_state_broadcaster..."
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager \
  --controller-manager-timeout 120 \
  --service-call-timeout 60 || true

echo "Trying to activate joint_state_broadcaster..."
ros2 control switch_controllers \
  --activate joint_state_broadcaster \
  --strict || true

echo "Spawning joint_trajectory_controller..."
ros2 run controller_manager spawner joint_trajectory_controller \
  --controller-manager /controller_manager \
  --controller-manager-timeout 120 \
  --service-call-timeout 60

echo "Sending initial pose..."
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
  points: [
    {
      positions: [0.0, -0.8, 1.2, 0.0, 1.0, 0.0],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}"

echo "Gazebo ABB4600 simulation started."
echo "Press Ctrl+C to stop."
wait
