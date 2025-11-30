cmake_minimum_required(VERSION 3.8)
project(s1_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Heartbeat.msg"
  "msg/HealthReport.msg"
  "msg/Telemetry.msg"
  "msg/LogEntry.msg"
  "msg/EstopCommand.msg"
  "msg/S1Command.msg"
  "msg/Command.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
