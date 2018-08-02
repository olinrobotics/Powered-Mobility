# Powered Mobility

[![Build Status](https://travis-ci.org/olinrobotics/Powered-Mobility.svg?branch=master)](https://travis-ci.org/olinrobotics/Powered-Mobility)

Development of Modular "Guardian Angel" Technology for Powered Mobility

Contact [Jeff Dusek](mailto:Jeff.Dusek@olin.edu) For more information.

Olin LAIR(Laboratory for Adaptation, Inclusion and Robotics)

## Requirements / Dependencies

- [ROS Kinetic Kame](http://wiki.ros.org/kinetic)
- [Python 2.7](https://www.python.org/download/releases/2.7/)
- [Catkin Tools](http://catkin-tools.readthedocs.io/en/latest/)

The following command will install most system/ros dependencies in the machine:

```bash
rosdep install --from-path ~/catkin_ws/src/Powered-Mobility --ignore-src
```

### Generate Python Pip Requirements

```bash
sudo pip install pipreqs
roscd pwm_robot/..
pipreqs .
mv requirements.txt pwm_bringup/config
```
