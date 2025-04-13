cmake_minimum_required(VERSION 3.8)
project(tarkbot_driver)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# 生成自定义接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LightSet.srv"
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${PROJECT_NAME}__rosidl_generator_cpp

)

# 编译可执行文件
add_executable(tarkbot_driver_node
  src/tarkbot_ros_node.cpp
  src/tarkbot_chasis_driver.cpp
)

target_link_libraries(tarkbot_driver_node
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 添加依赖库
ament_target_dependencies(tarkbot_driver_node
  rclcpp
  std_srvs
  std_msgs
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  rosidl_typesupport_cpp
)

ament_export_dependencies(
  rosidl_default_runtime
  rosidl_typesupport_cpp
)

ament_package()

# 安装可执行文件
install(
  TARGETS tarkbot_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本
install(
  PROGRAMS scripts/twist_to_ackermann.py
  DESTINATION lib/${PROJECT_NAME}
)