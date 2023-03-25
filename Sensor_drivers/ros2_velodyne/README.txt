This official driver is provided by the vendor
run the following commands to install the drivers:

$ sudo dpkg -i velodyne-lidar-driver_2.4.0-22.05.11.1422_focal_amd64.deb
$ sudo dpkg -i ros-foxy-velodyne-lidar-driver_2.4.0-22.05.11.1422_focal_amd64.deb

To use the driver, you need to run the following commands:

$ source /opt/ros/foxy/setup.bash
$ source /opt/velodyne/velodyne-lidar-driver-ros2/setup.bash
$ ros2 run lidar_driver_ros2 lidar_driver_ros2 --ros-args -p config_file:=/opt/velodyne/velodyne-lidar-driver/config/lidar_driver_velarray_m1600.cfg
