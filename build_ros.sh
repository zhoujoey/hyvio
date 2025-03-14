#!/bin/bash

if [ "$1" = "ROS1" ] || [ "$1" = "ros1" ] || [ "$1" = "1" ]; then
    ROS_VERSION=${VERSION_ROS1}
    cp cmake_ros/CMakeLists_ros1.txt CMakeLists.txt
    cp cmake_ros/package_ros1.xml package.xml
    cp cmake_ros/nodelets.xml nodelets.xml
    echo "build in ROS1"
else
    cp cmake_ros/CMakeLists_ros2.txt CMakeLists.txt
    cp cmake_ros/package_ros2.xml package.xml
    echo "build in ROS2"
fi

