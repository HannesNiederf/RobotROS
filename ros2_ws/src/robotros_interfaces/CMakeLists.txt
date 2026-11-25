cmake_minimum_required(VERSION 3.8)
project(robotros_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages (softwarespezifikation.md section 5.1, 5.3)
  "msg/SafetyState.msg"
  "msg/BumperState.msg"
  "msg/RangeState.msg"
  "msg/PowerState.msg"
  "msg/HubState.msg"
  "msg/HubCommand.msg"
  "msg/LedCommand.msg"
  "msg/BuzzerCommand.msg"
  "msg/Station.msg"
  "msg/Order.msg"
  "msg/MissionStatus.msg"
  "msg/OperatorAlert.msg"
  "msg/KeepoutZone.msg"
  "msg/ContactPoint.msg"
  "msg/OrderList.msg"
  "msg/StationList.msg"
  "msg/KeepoutZoneList.msg"
  "msg/ContactPointList.msg"
  "msg/MapMeta.msg"
  "msg/ActivityState.msg"
  # Services (section 5.2)
  "srv/SetArm.srv"
  "srv/SetRelay.srv"
  "srv/CreateOrder.srv"
  "srv/OrderAction.srv"
  "srv/AckSafety.srv"
  "srv/ResolveAlert.srv"
  "srv/SetMode.srv"
  "srv/SetCartLocation.srv"
  "srv/SaveMap.srv"
  "srv/ListMaps.srv"
  "srv/ActivateMap.srv"
  "srv/UpsertStation.srv"
  "srv/DeleteStation.srv"
  "srv/UpsertKeepoutZone.srv"
  "srv/DeleteKeepoutZone.srv"
  "srv/DeleteContactPoints.srv"
  "srv/GetSnapshot.srv"
  "srv/Nudge.srv"
  # Actions (section 5.2, spez_aenderung_01_rueckwaerts.md)
  "action/EngageCart.action"
  "action/ReleaseCart.action"
  "action/DockCharger.action"
  "action/Undock.action"
  "action/Localize.action"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
