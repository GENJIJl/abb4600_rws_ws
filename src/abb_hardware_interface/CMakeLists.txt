cmake_minimum_required(VERSION 3.5)
project(abb_hardware_interface)

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
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

# 查找依赖
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## 构建 ##
###########

add_library(
  ${PROJECT_NAME}
  SHARED
  src/abb_hardware_interface.cpp
  src/utilities.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

pluginlib_export_plugin_description_file(hardware_interface abb_hardware_interface.xml)

#############
## 安装 ##
#############

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

#############
## 测试 ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 关闭 ament_copyright 检查
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
