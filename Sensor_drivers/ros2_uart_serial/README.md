# ros2_uart_serial

A set of ROS2 drivers to support the USB-TTL communication between the gokart laptop and the nucleo main controller. The repository is developed based on the [transport_drivers](https://github.com/ros-drivers/transport_drivers) pacakges developed by the Autoware Foundation

## Supported Drivers:
    
**Serial Driver**

A package which which encapsulates basic receiving and sending of serial data.

Provided within this package is the following executabe:
- serial_bridge: combined both receiver and sender nodes into one

Provided within this package also is a `serial_driver` library without the ROS2 dependencies which could be used elsewhere.

**IO Context**

A library to write synchronous and asynchronous networking applications.

## Quick start

Clone the repo into your workspace, normally with the structure `workspace/src/<clone-repo-here>`:

```
git clone https://github.com/mlab-upenn/gokart-sensor.git
```

Switch to the pure pursuit branch
```
git checkout ros2_foxy_purepursuit
```

Install dependencies using `rosdep` from your top-level workspace directory:

```
rosdep install --from-paths src --ignore-src -r -y
```

Once you have the repository cloned and dependencies installed, you can now go ahead and compile:

```
colcon build --symlink-install
```

After successful compilation, you should be able to source your newly built packages:

```
source install/setup.bash
```

...and now you should be able to run your newly built executables. Here is how you would launch the `serial_driver` bridge node:

```
ros2 launch serial_driver serial_driver_bridge_node.launch.py
```

## Topics published by the simulation

`/drive_info_from_nucleo`: gokart's current speed (m/s) and steering (radians) information published as AckermannDriveStamped Message

`/effective_command_to_nucleo`: current driving command gokart is executing published as AckermannDriveStamped Message. Three sources of driving command happening at the same time: manual, teleop, autonomous. The nucleo will select the command based on the current control mode and filter out other commands.

## Topics subscribed by the simulation

`/automous_command_to_nucleo`: the serial_bridge_node subscribes to this topic as AckermannDriveStamped Message and transmit the autonomous command to the nucleo through usb-ttl.

## Touble Shooting
If you encounter an error like below, there might be several causes.
`[serial_bridge-1] [ERROR] [1682451189.016799529] [serial_bridge_node]: Error creating serial port: /dev/ttyUSB0 - open: No such file or directory`

1. Type ls /dev/ttyUSB0 in a terminal and see if the port is available. If the port has a different name, go to the params file and change the it.
2. The permission to use the serial port is denied. Type chmod 666 /dev/ttyUSB0 (or your port number) to change permission.
