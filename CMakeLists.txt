cmake_minimum_required(VERSION 3.0.2)
project(mqtt_bridge)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rospy std_msgs)
catkin_python_setup()

add_service_files(
  FILES Trigger.srv
)

add_message_files(
  FILES MirPosition.msg
  )

install(PROGRAMS
  scripts/mqtt_bridge_node.py scripts/mir_trigger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
