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

# Topics published by the simulation

`/drive_info_from_nucleo`: Gokart's current speed (m/s) and steering (radians) information published as ackermannstamped message

`/effective_command_to_nucleo`: Current driving command gokart is executing published as ackermannstamped message. Three sources of driving command happening at the same time: manual, teleop, autonomous. The nucleo will select the command based on the current control mode and filter out other commands.

# Topics subscribed by the simulation

`/automous_command_to_nucleo`: the serial_bridge_node subscribes to this topic as ackermannstamped message and transmit the autonomous control command to the nucleo through usb-ttl.
