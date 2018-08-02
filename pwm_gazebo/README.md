# PWM\_Gazebo

Gazebo simulation for the Powered Mobility project.

## Full Demo

The script below runs a demo including localization and navigation capabilities on the simulation:

```bash
roscore
roslaunch pwm_gazebo gazebo.launch paused:=false
roslaunch pwm_bringup bringup.launch arbiter:=false transform:=false hardware:=false navigation:=true
roslaunch pwm_navigation rviz.launch
```
