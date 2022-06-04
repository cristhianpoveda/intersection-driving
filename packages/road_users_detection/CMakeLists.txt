cmake_minimum_required(VERSION 2.8.3)
project(road_users_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs # Every duckietown packages
)

catkin_package()