#!/bin/bash

install_ros(){
    sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $ROS_CI_DESKTOP main\" > /etc/apt/sources.list.d/ros-latest.list"
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update -qq
    sudo apt-get install -y python-catkin-pkg python-catkin-tools python-rosdep python-wstool ros-$ROS_DISTRO-catkin -y
}

install_ros
