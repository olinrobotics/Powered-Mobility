## Pivoting Lidar Investigation

### Run:

```bash
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun urg_node urg_node _ip_address:="192.168.0.10"
rosrun lidar_pivot pub_as_tf.py
rosrun lidar_pivot swivel.py
rosrun tf static_transform_publisher 0 0 0.02 0 0 0 odom base_link 100
roslaunch pomo_description laser_terrain.launch 
```
