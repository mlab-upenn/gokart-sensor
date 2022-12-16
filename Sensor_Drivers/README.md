Depthai Camera Driver:
https://github.com/luxonis/depthai-ros
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

sudo apt install ros-foxy-depthai-ros
source /opt/ros/foxy/setup.bash
ros2 launch depthai_examples rgb_publisher.launch.py


Ouster LiDAR driver:
under setting go to the network tab
click on the gear next to the wired setting
click on IPv4
switch the IPv4 method to Manual
set Address to 192.0.2.200 and Netmask to 255.255.255.0
click Apply
check in a terminal with “ip a” that the IP address is correct
if the IP address is not correct a reboot can help → check again after

in home directory

git clone https://github.com/mfontanini/libtins.git
apt-get install libpcap-dev libssl-dev cmake
cd libtins
mkdir build
cd build
cmake ../
make
sudo make install
sudo ldconfig

in home directory
create workspace for OusterDriver
mkdir -p ros2_OusterDriver/src && cd ros2_OusterDriver/src
git clone https://github.com/ros-drivers/ros2_ouster_drivers.git

under ros2_OusterDriver/src/ros_ouster_driver/ros2_ouster/params open driver_config.yaml
change lidar_ip to 192.0.2.100
change computer_ip to 192.0.2.200

cd ~/ros2_OusterDriver
colcon build
. install/setup.bash
ros2 launch ros2_ouster driver_launch.py

to visualize in rviz2
rviz2
select “laser_data_frame” as “Fixed Frame”
add by topic “/points” the PointCloud2
one the left hand side extend the “Topic” Drop-Down menu
under “Reliability Policy” change to “Best Effort” (see picture)



Velodyne M1600:
under setting go to the network tab
click on the gear next to the wired setting
click on IPv4
switch the IPv4 method to Manual
set Address to 192.168.1.200 and Netmask to 255.255.255.0
click Apply
check in a terminal with “ip a” that the IP address is correct
if the IP address is not correct a reboot can help → check again after

Download deb files from google drive or github

Install Vella-Go deb files (temporary method until Vella-Go installation scripts completed)

cd to the directory where the ROS2 Vella-Go deb files are located.

$ sudo dpkg -i velodyne-lidar-driver_2.4.0-22
.05.11.1422_focal_amd64.deb
$ sudo dpkg -i ros-foxy-velodyne-lidar-driver_2.4.0-22.05.11.1422_focal_amd64.deb

(note: velodyne-lidar-driver deb file is the same for ROS1 and ROS2)

Source the ROS2 environment for foxy and the environment for velodyne ros2 driver

$ source /opt/ros/foxy/setup.bash
$ source /opt/velodyne/velodyne-lidar-driver-ros2/setup.bash

To start the driver use the following command (substitute the appropriate config file for the sensor being used)

The following commands are a single line
$ ros2 run lidar_driver_ros2 lidar_driver_ros2 --ros-args -p config_file:=/opt/velodyne/velodyne-lidar-driver/config/lidar_driver_velarray_m1600.cfg

If you run into issues, suggest running with the error logging to the console window:

$ GLOG_logtostderr=1 ros2 run lidar_driver_ros2 lidar_driver_ros2 --ros-args -p config_file:=/opt/velodyne/velodyne-lidar-driver/config/lidar_driver_velarray_m1600.cfg
