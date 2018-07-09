# PWM\_Gazebo

Gazebo simulation for the Powered Mobility project.

## Testing SLAM

```bash
roslaunch pwm_gazebo gazebo.launch paused:=false

rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/wheelchair/velocity_controller/cmd_vel

roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/kinect/depth/image_raw rgb_topic:=/kinect/rgb/image_raw camera_info_topic:=/kinect/rgb/camera_info frame_id:=base_footprint rtabmapviz:=false

rosrun rviz rviz -d $(rospack find pwm_description)/config/slam.rviz


```
