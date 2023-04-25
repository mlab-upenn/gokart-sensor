# ros2_uart_serial

A set of ROS2 drivers to support the USB-TTL communication between the gokart laptop and the nucleo main controller. The repository is developed based on the [transport_drivers](https://github.com/ros-drivers/transport_drivers) pacakges developed by the Autoware Foundation

## Supported Drivers:
    
* **Serial Driver**

A package which which encapsulates basic receiving and sending of serial data.

Provided within this package is the following executabe:
- serial_bridge: combined both receiver and sender nodes into one

Provided within this package also is a `serial_driver` library without the ROS2 dependencies which could be used elsewhere.

* **IO Context**

A library to write synchronous and asynchronous networking applications.

## Quick start

Clone the repo into your workspace, normally with the structure `workspace/src/<clone-repo-here>`:

```
git clone https://github.com/ros-drivers/transport_drivers.git
```

Install dependencies using `rosdep` from your top-level workspace directory:

```
rosdep install --from-paths src --ignore-src -r -y
```

Once you have the repository cloned and dependencies installed, you can now go ahead and compile:

```
colcon build
```

After successful compilation, you should be able to source your newly built packages:

```
source install/setup.bash
```

...and now you should be able to run your newly built executables. Here is how you would launch the `udp_driver` bridge node:

```
ros2 run udp_driver udp_bridge_node_exe --ros-args --params-file ./src/transport_drivers/udp_driver/params/example_udp_params.yaml
```

## Testing

Comprehensive unit tests have been written for every package within this repository.

To run them yourself, use the normal command from your top-level workspace directory:

```
colcon test
```
