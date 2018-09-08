#!/usr/bin/env bash
# refer to https://docs.docker.com/install/linux/docker-ce/ubuntu/

install_docker(){
	sudo apt update
	sudo apt install apt-transport-https ca-certificates curl software-properties-common
	curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
	sudo apt-key fingerprint 0EBFCD88
	sudo add-apt-repository \
		"deb [arch=amd64] https://download.docker.com/linux/ubuntu \
		$(lsb_release -cs) \
		stable"
	sudo apt update
	sudo apt install docker-ce

}

install_docker
