cmake_minimum_required(VERSION 2.8.3)
project(assign1_comp417)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_ros
        rospy
        sensor_msgs
        std_msgs
        tf
        )

catkin_package(
        CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs gazebo_ros
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
