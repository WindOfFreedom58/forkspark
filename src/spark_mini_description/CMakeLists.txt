cmake_minimum_required(VERSION 3.0.2)
project(spark_mini_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslaunch
)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
