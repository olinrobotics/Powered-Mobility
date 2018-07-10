
# Launch the control interface

```bash
rosrun joy joy_node
rosrun teleop_twist_joy teleop_node cmd_vel:=/manual_cmd_vel
rosrun topic_tools mux cmd_vel auto_cmd_vel manual_cmd_vel estop_cmd_vel mux:=mux_cmd_vel
python select_mode.py
```

# Launch the Realsense

```bash
roslaunch realsense2_camera rs_rgbd.launch
```

```bash
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info rtabmapviz:=false
```

```bash
rostopic pub /estop_cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 100
```

```bash
rostopic pub /auto_cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 100
```

```bash
rosbag record
```
