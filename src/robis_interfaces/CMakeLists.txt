cmake_minimum_required(VERSION 3.8)
project(robis_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GotoObject.action"
  "srv/DetectObject.srv"
  "msg/ExternalObject.msg"
  "msg/ExternalObjects.msg"
  DEPENDENCIES geometry_msgs vision_msgs std_msgs
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
