cmake_minimum_required(VERSION 3.16)
project(custom_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(custom_interfaces
    "msg/Path.msg"
    "srv/GetWaypoints.srv"
    "srv/GetUnitNum.srv"
    "srv/TaskRecord.srv"
    "action/FindUnit.action"
    DEPENDENCIES geometry_msgs
    )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
