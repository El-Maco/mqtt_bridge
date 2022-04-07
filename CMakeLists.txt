cmake_minimum_required(VERSION 3.0.2)
project(mqtt_bridge)

find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs)
catkin_python_setup()

add_service_files(
  FILES Trigger.srv
)

install(PROGRAMS
  scripts/mqtt_bridge_node.py scripts/mir_trigger_client.py scripts/mir_trigger_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
