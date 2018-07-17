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
