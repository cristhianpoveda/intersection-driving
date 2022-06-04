cmake_minimum_required(VERSION 2.8.3)
project(stop_finder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs # Every duckietown packages should use this.
)

catkin_package()