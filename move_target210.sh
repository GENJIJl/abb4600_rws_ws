#!/usr/bin/env bash

cd ~/abb4600_rws_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint_1','joint_2','joint_4','joint_5','joint_3','joint_6'],
  points: [
    {
      positions: [
        -1.6445714816283807,
         0.46604889641660896,
         1.4781786414588294,
         1.588014195874477,
         0.25116145740385515,
         2.393441461198193
      ],
      time_from_start: {sec: 8, nanosec: 0}
    }
  ]
}"
