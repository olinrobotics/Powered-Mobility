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

function rospkg-deps(){
	echo "$(grep -r pkg $(rospack find pwm_bringup)/.. --include='*.launch' | perl -pe 's|.*pkg="(.*?)".*|\1|')" | sort | uniq
}
