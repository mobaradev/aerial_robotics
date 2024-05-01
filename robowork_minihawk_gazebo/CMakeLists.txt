cmake_minimum_required(VERSION 2.8.3)
project(robowork_minihawk_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch roscpp rospy std_msgs message_generation)



add_message_files(FILES Num.msg)
add_service_files(
  FILES
  AddTwoInts.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(CATKIN_DEPENDS message_runtime)

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch worlds urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py scripts/add_two_ints_server.py scripts/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
