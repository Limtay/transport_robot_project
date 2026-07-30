cmake_minimum_required(VERSION 3.8)
project(mgs_tp_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodeStatus.msg"
  "msg/MotorStatus.msg"
  "msg/ControlFeedback.msg"
  "msg/CommDiag.msg"
  "msg/CmdMotor.msg"
  "srv/ControlConfig.srv"
  "srv/CommandSet.srv"
  "action/RunProfile.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
