// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* 所有声明 rclcpp 库中定义符号的 rclcpp 头文件都必须包含此头文件。
 * 当没有构建 rclcpp 库时，例如在其他包代码中使用这些头文件时，
 * 此头文件会调整某些符号的可见性，以便使用方代码能够正确链接。
 */

#pragma once

// 该逻辑借鉴自 gcc wiki 示例，并增加了命名空间前缀：
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROS2_CONTROL_DRIVER_EXPORT __attribute__((dllexport))
#define ROS2_CONTROL_DRIVER_IMPORT __attribute__((dllimport))
#else
#define ROS2_CONTROL_DRIVER_EXPORT __declspec(dllexport)
#define ROS2_CONTROL_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef ROS2_CONTROL_DRIVER_BUILDING_DLL
#define ROS2_CONTROL_DRIVER_PUBLIC ROS2_CONTROL_DRIVER_EXPORT
#else
#define ROS2_CONTROL_DRIVER_PUBLIC ROS2_CONTROL_DRIVER_IMPORT
#endif
#define ROS2_CONTROL_DRIVER_PUBLIC_TYPE ROS2_CONTROL_DRIVER_PUBLIC
#define ROS2_CONTROL_DRIVER_LOCAL
#else
#define ROS2_CONTROL_DRIVER_EXPORT __attribute__((visibility("default")))
#define ROS2_CONTROL_DRIVER_IMPORT
#if __GNUC__ >= 4
#define ROS2_CONTROL_DRIVER_PUBLIC __attribute__((visibility("default")))
#define ROS2_CONTROL_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define ROS2_CONTROL_DRIVER_PUBLIC
#define ROS2_CONTROL_DRIVER_LOCAL
#endif
#define ROS2_CONTROL_DRIVER_PUBLIC_TYPE
#endif
