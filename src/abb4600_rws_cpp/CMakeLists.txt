cmake_minimum_required(VERSION 3.8)
project(abb4600_rws_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(abb4600_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(start_fixed_path_node src/start_fixed_path_node.cpp)
ament_target_dependencies(start_fixed_path_node rclcpp std_msgs)
target_link_libraries(start_fixed_path_node CURL::libcurl)

add_executable(execute_path10_moveit_node src/execute_path10_moveit_node.cpp)
ament_target_dependencies(
  execute_path10_moveit_node
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  tf2
  tf2_geometry_msgs
)

add_executable(execute_train_hook_moveit_node src/execute_train_hook_moveit_node.cpp)
ament_target_dependencies(
  execute_train_hook_moveit_node
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  tf2
  tf2_geometry_msgs
)


add_executable(run_robot_path_action_server src/run_robot_path_action_server.cpp)
ament_target_dependencies(
  run_robot_path_action_server
  abb4600_interfaces
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  trajectory_msgs
)

install(TARGETS
  run_robot_path_action_server
  execute_path10_moveit_node
  execute_train_hook_moveit_node
  start_fixed_path_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/egm_joint6_sweep_test.py
  scripts/tiny_joint1_test.py
  scripts/tiny_joint6_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README_ros2_control_robotstudio.md
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  robotstudio
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  frp
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(vision_target_pose_planner_node
  src/vision_target_pose_planner_node.cpp
)

ament_target_dependencies(vision_target_pose_planner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  trajectory_msgs
)

install(TARGETS
  vision_target_pose_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


add_executable(path10_sparse_ompl_planner_node
  src/path10_sparse_ompl_planner_node.cpp
)

ament_target_dependencies(path10_sparse_ompl_planner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  trajectory_msgs
  moveit_msgs
)

install(TARGETS
  path10_sparse_ompl_planner_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(singularity_aware_target_demo_node
  src/singularity_aware_target_demo_node.cpp
)

ament_target_dependencies(singularity_aware_target_demo_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  singularity_aware_target_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
