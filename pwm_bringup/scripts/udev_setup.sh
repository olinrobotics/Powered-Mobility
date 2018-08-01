#!/bin/bash

# make sure that the system recognize pwm_bringup package

udev_setup(){
	if [ $(rospack find pwm_bringup) ]; then
		echo 'Found pwm_bringup package'
	else
		echo 'pwm_bringup package could not be found; attempt to build the package'
		catkin build pwm_bringup -w ~/catkin_ws
		source ~/catkin_ws/devel/setup.bash && rospack profile
	fi

	if [ -f /etc/udev/rules.d/10-pwm.rules ]; then
		echo '/etc/udev/rules.d/10-pwm.rules already exists!'
	else
		echo 'Installing 10-pwm.rules at /etc/udev/rules.d'
		sudo cp $(rospack find pwm_bringup)/config/10-pwm.rules /etc/udev/rules.d
		sudo udevadm control --reload
		sudo udevadm trigger
	fi
}

udev_setup
