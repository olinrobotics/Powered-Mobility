#!/bin/bash

urg_setup()
{
	local ifname=$(nmcli dev status | grep ethernet | head -n 1 | awk '{print $1}')
	echo "Using interface : $ifname"
	sudo nmcli con add type ethernet con-name URG ifname $ifname ip4 192.168.0.1/24 gw4 192.168.0.0
}
urg_setup
