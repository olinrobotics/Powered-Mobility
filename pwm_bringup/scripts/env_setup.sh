#!/bin/bash

env_setup(){
	echo 'source $(rospack find pwm_bringup)/scripts/env.sh' >> ~/.bashrc
}

env_setup
