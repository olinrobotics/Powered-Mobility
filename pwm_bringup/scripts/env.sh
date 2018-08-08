#!/bin/bash

function ros_setup(){
	case "$#" in
		1)
			export ROS_IP="$1"
			export ROS_MASTER_URI="http://$1:11311"
			;;
		2)
			export ROS_IP="$1"
			export ROS_MASTER_URI="http://$2:11311"
			;;
		*)
			echo "Invalid # of Parameters"
			;;
		esac
}

function finddev(){
	for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
		(
			syspath="${sysdevpath%/dev}"
			devname="$(udevadm info -q name -p $syspath)"
			[[ "$devname" == "bus/"* ]] && continue
			eval "$(udevadm info -q property --export -p $syspath)"
			[[ -z "$ID_SERIAL" ]] && continue
			echo "/dev/$devname - $ID_SERIAL"
		)
	done
}

function pwm-list-rosdeps(){
	# useful for discovering un-documented exec-dependencies (i.e. not in package.xml)
	# note that since this only enumerates packages in launch files, it cannot catch build deps.
	echo "$(grep -r pkg $(rospack find pwm_bringup)/.. --include='*.launch' | perl -pe 's|.*pkg="(.*?)".*|\1|')" | sort | uniq
}

function pwm-install-deps(){
	# install from package.xml; assume workspace in ${HOME}/catkin_ws
	rosdep install --from-path ${HOME}/catkin_ws/src/Powered-Mobility --ignore-src --skip-keys='librealsense2' -y
}

function pwm-build(){
	# build catkin workspace; assume workspace in ${HOME}/catkin_ws
	catkin build -w ${HOME}/catkin_ws pwm_robot -DCMAKE_BUILD_TYPE=Release
}

function pwm-test(){
	catkin run_tests -w ${HOME}/catkin_ws pwm_tests && catkin_test_results ~/catkin_ws
}

function pwm-cd(){
	roscd pwm_robot/..
}

function pwm-search-ip(){
	# assume ethernet configured @ 10.42.0.X
	nmap -sP 10.42.0.0/24
}

function pwm-setup(){
	read -p 'Setup URG Network Configuration? [y/N]' urg
	case $urg in
		[Yy]* ) 
			rosrun pwm_bringup urg_setup.sh
			;;
		* ) ;;
	esac

	read -p 'Setup Udev Rules? [y/N]' udev
	case $udev in
		[Yy]* )
			rosrun pwm_bringup udev_setup.sh
			;;
		* ) ;;
	esac

	read -p 'Setup UART Configuration? (Only valid for Pi3+UART Configuration)' uart
	case $uart in
		[Yy]* )
			rosrun pwm_bringup uart_setup.sh
			;;
		* ) ;;
	esac

	read -p 'Setup CAN Configuration? (Only valid for Pi3+PiCan2)' can
	case $can in
		[Yy]* )
			rosrun pwm_bringup can_setup.sh
			;;
		* ) ;;
	esac
}
