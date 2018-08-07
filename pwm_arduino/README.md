# PWM\_Arduino

Arduino related developments for the Powered Mobility Project.

Currently, the arduino acts as the interface for the following components:

- 2 x Ultrasonic Rangefinders ([MaxSonar-EZ3](https://www.maxbotix.com/Ultrasonic_Sensors/MB1030.htm))
- 2 x MPU9250 IMU ([Sparkfun SEN-13762](https://www.sparkfun.com/products/13762))
- 1 x RNET(CAN) interface for the [Permobil M300 Wheelchair](https://permobilus.com/product/m300-corpus-hd/)


## Installation

For Arduino setup, follow the guide [Here](https://www.arduino.cc/en/Guide/Linux).

For `rosserial_arduino` setup, follow the tutorial [Here](http://wiki.ros.org/rosserial_arduino/Tutorials)

The following setup procedure is necessary in order to use the Arduino Due:

```bash
arduino --install-boards "arduino:sam"
```

Obtain port permissions as follows:

```bash
sudo usermod -a -G dialout $USER
```

and Line `ArduinoHardware.h#75` should be modified as:

```
#if defined(USBCON) and !(defined(USE_USBCON))
      /* Leonardo support */
      iostream = &Serial;
```

For persistent port assignments, configure udev rules as follows:

```
rosrun pwm_bringup udev_setup.sh
```

Note that the package uses the arduino CLI interface to update the firmware onboard:

```bash
catkin build pwm_arduino --make-args pwm_arduino_update
```

For vanilla upload, use the following command:

```bash
export ARDUINO_PORT=/dev/ttyACM0 #/dev/arduino
roscd pwm_arduino && cd firmware
arduino --board arduino:sam:arduino_due_x_dbg --port ${ARDUINO_PORT} --upload firmware.ino
```
