# GoKart-Sensor-Ros2

This repo is the source code for ros2 go-kart application.



## Overview

There are several different modules in the GoKart application. It is still under development, so the structure  might be changed time to time.

### Sensor drivers

Currently the sensor system of GoKart includes:

(1) GNSS+RTK with Septentrio receiver board ([internel resource link](https://drive.google.com/drive/folders/1cNY-N6_Q-Gh8M3gbirDXrJNdaiElLbfh?usp=share_link))

(2) Ouster OS1 LiDAR ([manual](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf))

(3) Velodyne M1600 solid-state LiDAR ([internal resource link](https://drive.google.com/drive/folders/1519wXv9WTFysgB52GzT6PCQ7rEGvB3O7?usp=share_link))

(4) OAK-D camera ([manual](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK.html))

(5) Bno055 IMU ([datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf))

> ├── Sensor_drivers
>
> │   ├── ouster-ros ([link](https://github.com/ouster-lidar/ouster-ros/tree/ros2-foxy))
>
> │   ├── ros2_velodyne_driver_debs (download from official debs)
>
> │   ├── ros_imu_bno055 (ros2 wrapper of [link]())
>
> │   └── septentrio_gnss_driver([link]())

Note:  

Velodyne Lidar ros2 driver and OAK-D camera ros2 driver([github link](https://github.com/luxonis/depthai-ros)) can be installed directly in CLI as ros2 internal driver packages. So they are not listed inside the Sensor_drivers, but in the install dependency part.



### Object_detection

We use yolov8([ultralytics](https://docs.ultralytics.com/)) for camera object detection. Apart from the standard VOC mode, we also have a cone-detect model.

> ├── Object_detection
>
> │   └── yolov8_pkg



### Mapping & Localization

We use Lidarslam package from this repo([link](https://github.com/rsasaki0109/lidarslam_ros2)) to do the 3D slam. 

> ├── Localization **[under dev]**
>
> │   └── ros2_python_pkg
>
> ├── Slam
>
> │   └── lidarslam_ros2(link)
>
> 
>
> 

## Install

- Install dependency

```
# common libs
sudo apt install camke libboost-all-dev libpcap-dev libssl-dev

# for yolov8(ultralytics)
pip install ultralytics

# for OAK-D camera:
sudo apt install ros-foxy-depthai-ros

# for GNSS drivers:
sudo apt install ros-foxy-nmea-msgs ros-foxy-gps-msgs
sudo apt install libgeographic-dev

# for Ouster driver:
sudo apt install -y         \
	ros-foxy-pcl-ros        \
	ros-foxy-tf2-eigen      \
	ros-foxy-rviz2          \
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions

# for Velodyne driver:
cd Sensor_drivers/ros2_velodyne, then do as the README said.
```



- Make a ros2 workspace under your home directory and git clone the source code to /src

```bash
mkdir -p gokart_ws/src
cd gokart_ws/src
# clone the repo, make sure to use --recursive to fetch the submodules
git clone -b ros2_foxy --recursive git@github.com:mlab-upenn/gokart-sensor.git
# go back to the workspace dir
cd ..
# use rosdep to install all the dependency needed by the ros2 packages
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
# build, the Release flag is required by the ouster driver
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```



## Usage

### Hardware Setting

**Ethernet**

The Ouster and Velodyne LiDARs are connected to an Ethernet switcher, which keeps them under the same network gate. The Wired setting of the host ubuntu should be:

> Choose Manual under IPv4 Method, set Address to 192.0.2.200, set Netmask to 255.255.255.0





### Sensor drivers

Run ros2 launch/run in different terminals for each driver(can use **tmux**, see [section link](#tmux-intro))

```bash
# launch Ouster driver
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.0.2.100 timestamp_mode:=TIME_FROM_ROS_TIME

# source Velodyne driver
source /opt/velodyne/velodyne-lidar-driver-ros2/setup.bash
# launch Velodyne driver
ros2 run lidar_driver_ros2 lidar_driver_ros2 --ros-args -p config_file:=/opt/velodyne/velodyne-lidar-driver/config/lidar_driver_velarray_m1600.cfg

# launch gnss driver(default config is set to gnss.yaml)
ros2 launch septentrio_gnss_driver rover.py

# launch imu driver(default mode is set to NDOF)
ros2 launch ros_imu_bno055 imu_launch.py

# launch OAK-D camera driver
ros2 launch depthai_examples rgb_publisher.launch.py
```



### Object detection(yolov8)

Run the following commands in three different terminals.

```bash
ros2 launch yolov8_pkg yolov8_node.launch.py
ros2 bag play <your_bag_name>
rviz2
```



### Slam(lidarSlam)

```
ros2 launch lidarslam lidarslam.launch.py
```





## Reference Links

### Tmux intro

A cheatsheet for the original tmux shortcut keys can be found [here](https://tmuxcheatsheet.com/). To know about how to change the configuration of tmux to make it more  useable (for example, if you want to toggle the mouse mode on when you  start a tmux bash session or change the shortcut keys), you can find a  tutorial [here](https://www.hamvocke.com/blog/a-guide-to-customizing-your-tmux-conf/).

