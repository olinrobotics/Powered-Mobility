#!/bin/sh

# WARNING : the following script may not work for certain configurations.
# The lines below have only been tested on a Raspberry Pi 3 with a PiCan2 Hat board.

can_setup(){
	# modify /boot/config.txt
	sudo sh -c 'echo "dtparam=spi=on\ndtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25\ndtoverlay=spi-bcm2835" >> /boot/config.txt'

	# modify /etc/network/interfaces
	sudo sh -c 'echo "allow-hotplug can0\niface can0 can static\n\tbitrate 125000\n\tup /sbin/ip link set \$IFACE DOWN\n\tup /sbin/ip link set \$IFACE up" >> /etc/network/interfaces'
}

can_setup
