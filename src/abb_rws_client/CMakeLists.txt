cmake_minimum_required(VERSION 3.8)
project(abb_rws_client)

# 默认使用 C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra
      -Wwrite-strings -Wunreachable-code -Wpointer-arith
    -Winit-self -Wredundant-decls
      -Wno-unused-parameter -Wno-unused-function)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    abb_egm_rws_managers
    abb_egm_msgs
    abb_robot_msgs
    abb_rapid_msgs
    abb_rapid_sm_addin_msgs
    abb_hardware_interface
    rclcpp
    sensor_msgs
)

# 查找依赖
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## 构建 ##
###########

add_library(rws_client_lib
  src/rws_service_provider_ros.cpp
  src/rws_state_publisher_ros.cpp
  src/mapping.cpp
)
ament_target_dependencies(rws_client_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  rws_client_lib
  PRIVATE
  include
)

add_executable(rws_client src/rws_client_node.cpp)
target_link_libraries(rws_client rws_client_lib)
ament_target_dependencies(rws_client "rclcpp")
target_include_directories(
  rws_client
  PRIVATE
  include
)

#############
## 安装 ##
#############

install(
  TARGETS rws_client
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
  rws_client_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#############
## 测试 ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
