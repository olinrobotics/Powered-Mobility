# Powered-Mobility

Development of Modular "Guardian Angel" Technology for Powered Mobility

Contact [Jeff Dusek](mailto:Jeff.Dusek@olin.edu) For more information.

Olin LAIR(Laboratory for Adaptation, Inclusion and Robotics)

## Requirements / Dependencies

- [ROS Kinetic Kame](http://wiki.ros.org/kinetic)
- [Python 2.7](https://www.python.org/download/releases/2.7/)

## Setup Udev Rules

```
sudo cp $(rospack find pwm_bringup)/config/10-pwm.rules /etc/udev/rules.d
sudo udevadm control --reload
sudo udevadm trigger
```
