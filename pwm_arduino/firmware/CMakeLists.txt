cmake_minimum_required(VERSION 2.8.3)
include_directories(${ROS_LIB_DIR})
add_definitions(-DUSB_CON)
generate_arduino_firmware(
    pwm_arduino_driver
    SRCS main.cpp ${ROS_LIB_DIR}/time.cpp
    BOARD due
    PORT /dev/arduino
    )
