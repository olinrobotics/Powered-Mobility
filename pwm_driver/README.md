# PWM\_Driver

This package implements R-Net ROS Interfaces; namely, it connects the CAN-based R-NET protocol on standard wheelchairs to the ROS network to issue motion commands and report basic chair status (such as battery level).

## Install / Build

For instructions on installing and building the package, follow the package-wide [setup instruction](TODO); the package follows standard ros setup with properly configured dependencies under [package.xml](package.xml), so reviewing the `ros` and `catkin_tools` documentation should be sufficient. As such, the instructions will focus on operations.

## Running R-NET Client

Two R-NET Drivers are available under `pwm_driver` : `rnet_teleop_node.py` and `rnet_node`.

While poorly named (TODO : rename executables), these drivers support two different configurations:

- `rnet_teleop_node.py` : standard ROS interface, (as such, it can also be run remotely over TCP/IP)
- `rnet_node` : minimalistic `rosserial_embeddedlinux` based driver (supports Serial/TCP/IP)

For guidelines on running each of the interfaces, see the instructions below.

### Running (Standard TCP/IP Client)

```bash
roslaunch pwm_driver permobil_m300.launch
```

### Running (Minimal Serial/TCP/IP Client)

Note that this version will be mostly useful for serial connections,
as TCP/IP protocol is natively supported by the standard ROS protocol.

To avoid confusion, do not run the serial client inside roslaunch.

For details, refer to [serial\_client.launch](launch/serial_client.launch)

```bash
# for help, run as: rosrun pwm_driver rnet_node -h
rosrun pwm_driver rnet_node -c can0 -p 10.27.92.14 -t 0.2 -r 50
```

## Testing CAN Without Hardware Interfaces:

1. Setup Virtual Can Interface:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

2. Start Serial Server:

```bash
rosrun rosserial_python serial_node.py tcp
```

3. Start Serial Client:

```bash
screen -S rnet_node
rosrun pwm_driver rnet_node -c vcan0 -p localhost -t 0.2 -r 50
# press <C-A>D to detach from screen
```

4. Observe Output:

In one terminal:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

In another terminal:

```bash
candump -L vcan0
```
