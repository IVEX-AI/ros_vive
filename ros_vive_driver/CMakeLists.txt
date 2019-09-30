cmake_minimum_required(VERSION 2.8.3)

project(ros_vive_driver)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    ros_vive_driver_msgs
)

catkin_python_setup()


catkin_package(
    DEPENDS rospy geometry_msgs ros_vive_driver_msgs
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)