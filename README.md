# GoKart-Sensor-Ros2

This is the latest ros2 Humble repo for gokart applications.



## Overview

### Sensor Drivers

(1) Senpentrio Mosaic-H Dev Kit ([Manual](https://www.septentrio.com/en/products/gps/gnss-receiver-modules/mosaichdevkit)) + Swift Navigation RTK ([Link](https://www.swiftnav.com/skylark))

(2) Ouster OS1 LiDAR ([Manual](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf) + [GitHub](https://github.com/ouster-lidar/ouster-ros/tree/ros2))

(3) OAK-D camera ([Manual](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK.html) + [GitHub](https://github.com/luxonis/depthai-ros))

(4) BNO055 IMU ([Manual](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)) + LC231X UART to Serial Module ([Manual](https://www.digikey.com/en/products/detail/ftdi-future-technology-devices-international-ltd/LC231X/6823712))

### Perception

We use yolov8 ([ultralytics](https://docs.ultralytics.com/)) for camera object detection. Apart from the standard VOC mode, we also have a cone-detect model.

> ├── Object_detection
>
> │   └── yolov8_pkg



### Mapping & Localization

We use Lidarslam package ([link](https://github.com/rsasaki0109/lidarslam_ros2)) to perform the 3D slam. 

> ├── Localization **[under dev]**
>
> │   └── ros2_python_pkg
>
> ├── Slam
>
> │   └── lidarslam_ros2



## Install

Install dependency

```
# for yolov8
pip install ultralytics

# for OAK-D camera:
sudo apt install ros-humble-depthai-ros

# for IMU driver:
pip install pyserial
```

Make a ros2 workspace under your home directory and git clone the source code to /src

```bash
mkdir -p gokart_ws/src
cd gokart_ws/src
# clone the repo, make sure to use --recursive to fetch the submodules
git clone -b ros2_humble_purepursuit --recursive git@github.com:mlab-upenn/gokart-sensor.git
# go back to the workspace dir
cd ..
# use rosdep to install all the dependency needed by the ros2 packages
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
# build, the Release flag is required by the ouster driver
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```



## Usage

### Hardware Setting

**Ethernet**

The Ouster is connected to the laptop through ethernet cable. After connecting, choose manual configuration and under IPv4, set Address to 192.0.2.200 and Netmask to 255.255.255.0. Restart your system and ping 192.0.2.100 to verify succesful connection.



### Sensor Drivers

Run ros2 launch/run in different terminals for each driver(see [tmux](#tmux-intro))

```bash
# under the workspace folder, source
source /opt/ros/humble/setup.bash && source install/setup.bash

# launch Ouster driver
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.0.2.100 timestamp_mode:=TIME_FROM_ROS_TIME

# launch gnss driver(default config is set to gnss.yaml)
ros2 launch septentrio_gnss_driver rover.py

# launch imu driver(default mode is set to NDOF)
ros2 launch ros_imu_bno055 imu_launch.py

# launch OAK-D camera driver
ros2 launch depthai_examples rgb_publisher.launch.py
```

### Udev Rules

BNO055 IMU
```bash
create the udev rule file
sudo gedit /etc/udev/rules.d/bno055.rules

# Copy & paste the following rules, save and exit.
ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE="0666", SYMLINK+="sensors/bno055"
```

Nucleo Main Board
```bash
create the udev rule file
sudo gedit /etc/udev/rules.d/usb_uart.rules

# Copy & paste the following rules, save and exit.
ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", SYMLINK+="sensors/usb_uart"
```

Reload and trigger the rules
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change
```

Unplug and replug in the devices. and you should find the devices by running
```bash
ls /dev/sensors
```

### Perception (yolov8)

Run the following commands in three different terminals.

```bash
ros2 launch yolov8_pkg yolov8_node.launch.py
ros2 bag play <your_bag_name>
rviz2
```

### LiDAR SLAM

```
ros2 launch lidarslam lidarslam.launch.py
```


### Checking
You can show all the topic list and echo some specific topics to see if the drivers and applications are launched successfully.
```
ros2 topic list
ros2 topic echo <topic name> --no-arr
```



## Reference Links

### Tmux intro

A cheatsheet for the original tmux shortcut keys can be found [here](https://tmuxcheatsheet.com/). To know about how to change the configuration of tmux to make it more  useable (for example, if you want to toggle the mouse mode on when you  start a tmux bash session or change the shortcut keys), you can find a  tutorial [here](https://www.hamvocke.com/blog/a-guide-to-customizing-your-tmux-conf/).

