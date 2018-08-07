# Powered Mobility

[![Build Status](https://travis-ci.org/olinrobotics/Powered-Mobility.svg?branch=master)](https://travis-ci.org/olinrobotics/Powered-Mobility)

Development of Modular "Guardian Angel" Technology for Powered Mobility

Contact [Jeff Dusek](mailto:Jeff.Dusek@olin.edu) For more information.

Olin LAIR(Laboratory for Adaptation, Inclusion and Robotics)

## General Requirements / Dependencies

- [ROS Kinetic Kame](http://wiki.ros.org/kinetic)
- [Python 2.7](https://www.python.org/download/releases/2.7/)
- [Catkin Tools](http://catkin-tools.readthedocs.io/en/latest/)

Refer to [travis configuration](.travis.yml) for the breakdown of general installation workflow.

## Working with the package

To streamline the workflow with the package, it is recommended to configure the environment as follows:

```bash
rosrun pwm_bringup env_setup.sh
# note: command requires functional workspace + package configuration beforehand.
```

This simply adds functional aliases to the `~/.bashrc`.

Afterwards, the procedure is relatively straightforward. The following commands are exported:

```bash
ros_setup # shortcut to setup tcp/ip remote ros operations
finddev # list most serial devices
pwm-install-deps # install most system/ros dependencies in the machine
pwm-cd # navigate to package root
pwm-build # build pwm-related package only
pwm-list-rosdeps # list un-documented package exec dependencies
pwm-setup # runs a series of initial setup scripts for deployment, requires supervision
```

### Generate Python Pip Requirements

```bash
sudo pip install pipreqs
pwm-cd && pipreqs . --print
```
