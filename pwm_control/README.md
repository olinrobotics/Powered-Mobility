## Pivoting Lidar Investigation

### Run:

```bash
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun urg_node urg_node _ip_address:="192.168.0.10"
rosrun pwm_control pub_as_tf.py
rosrun pwm_control swivel.py
rosrun tf static_transform_publisher 0 0 0.02 0 0 0 odom base_link 100
roslaunch pwm_description laser_terrain.launch 
```
