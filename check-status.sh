
echo "check-status.sh started" > log.txt
source /opt/ros/foxy/setup.bash

echo "ros2 run sub_status start" >> log.txt
ros2 run sub_status subscribe >> log.txt
echo "ros2 run sub_status ended" >> log.txt
