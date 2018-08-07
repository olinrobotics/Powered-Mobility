#!/bin/sh

# WARNING : the following script is only applicable to the
# Raspberry Pi 3 Interface with UART-based communication.

uart_setup(){
	# modify /boot/config.txt
	sudo sh -c 'echo "dtoverlay=pi3-miniuart-bt" >> /boot/config.txt'
}

uart_setup
