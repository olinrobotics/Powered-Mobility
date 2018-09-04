#!/bin/bash
install_realsense(){
    sudo sh -c "echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' > /etc/apt/sources.list.d/realsense-public.list"
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE
    sudo apt-get update -qq
    sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y
}
install_realsense
