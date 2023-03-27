# GoKart-Sensor-Ros2

This repo is the source code for ros2 go-kart application.



## Overview

There are several different modules in the GoKart application. It is still under development, so the structure  might be changed time to time.

### Sensor drivers

Currently the sensor system of GoKart includes:

(1) GNSS+RTK with Septentrio receiver board

(2) Ouster OS1 LiDAR

(3) Velodyne M1600 solid-state LiDAR

(4) OAK-D camera

(5) Bno055 IMU 



├── Sensor_drivers
│   ├── ros2_imu_bno055
│   ├── ros2_ouster
│   └── ros2_velodyne

Note:  Septentrio GNSS ros2 driver, Velodyne Lidar ros2 driver and OAK-D camera ros2 driver can be installed directly in CLI as ros2 internal driver packages. So they are not listed here, but in the install dependency part.



### Object_detection

We use yolov8(ultralytics) for camera object detection. Apart from the standard VOC mode, we also have a cone-detect model.

├── Object_detection
│   └── yolov8_pkg



### Mapping & Localization

We use Lidarslam package from this repo([link](https://github.com/rsasaki0109/lidarslam_ros2)) to do the 3D slam. 

├── Localization **[under dev]**
│   └── ros2_python_pkg

├── Slam
│   └── lidarslam_ros2



### ThirdParty library

└── Thirdparty
    └── libtins (required for the ouster driver)



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
cd Thirdparty/libtins
mkdir build
cd build
cmake ../
make
sudo make install
sudo ldconfig

# for Velodyne driver:
cd Sensor_drivers/ros2_velodyne, then do as the README said.
```



- Make a ros2 workspace under your home directory and git clone the source code to /src

```bash
mkdir -p gokart_ws/src
cd gokart_ws
# use rosdep to install all the dependency needed by the ros2 packages
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
# build
colcon build
source install/setup.bash
```



## Usage

### Object detection(yolov8)

Run the following commands in three different terminals(can use tmux, see [section link](#tmux-intro) )

```bash
ros2 launch yolov8_pkg yolov8_node.launch.py
ros2 bag play <your_bag_name>
rviz2
```





## Reference Links

### Tmux intro

A cheatsheet for the original tmux shortcut keys can be found [here](https://tmuxcheatsheet.com/). To know about how to change the configuration of tmux to make it more  useable (for example, if you want to toggle the mouse mode on when you  start a tmux bash session or change the shortcut keys), you can find a  tutorial [here](https://www.hamvocke.com/blog/a-guide-to-customizing-your-tmux-conf/).

