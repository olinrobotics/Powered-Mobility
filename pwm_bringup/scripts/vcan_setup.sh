#!/bin/sh

# WARNING : the following script is only for testing with a virtual CAN port,
# possibly interfacing with Gazebo or testing other features related to CAN-bus.

vcan_setup(){
	sudo modprobe vcan
	sudo ip link add dev vcan0 type vcan
	sudo ip link set up vcan0
}

vcan_setup
