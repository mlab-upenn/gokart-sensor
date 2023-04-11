FROM ros:foxy
SHELL ["/bin/bash", "-c"]

#RUN apt update
#RUN apt install -y python3-pip git
#RUN apt install -y openssh-client
#RUN apt install -y ros-foxy-rviz2

## common libs
#RUN apt install -y cmake libboost-all-dev libpcap-dev libssl-dev

ENV DEBIAN_FRONTEND=noninteractive
# FROM ros:foxy
# SHELL ["/bin/bash", "-c"]
# ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y \ 
    python3-pip \
# common libs
    cmake libboost-all-dev libpcap-dev libssl-dev \ 
# for OAK-D camera:
#    ros-foxy-depthai-ros \
# for GNSS drivers:
    ros-foxy-nmea-msgs ros-foxy-gps-msgs \ 
    libgeographic-dev \ 
# for Ouster driver:
    ros-foxy-pcl-ros        \
    ros-foxy-tf2-eigen      \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake \
    python3-colcon-common-extensions \ 
# for Velodyne driver:
    libgoogle-glog0v5 \ 
    libconfig++9v5 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
# for yolov8(ultralytics)
RUN pip install ultralytics
# for OAK-D camera:
RUN apt update \
    && apt install -y ros-foxy-depthai-ros \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/gokart_ws/src
COPY data_aug_ros2 /home/gokart_ws/src/data_aug_ros2
COPY Localization /home/gokart_ws/src/Localization
COPY Object_detection /home/gokart_ws/src/Obejct_detection
COPY pics /home/gokart_ws/src/pics
COPY robot_localization /home/gokart_ws/src/robot_localization
COPY Sensor_drivers /home/gokart_ws/src/Sensor_drivers
COPY Slam /home/gokart_ws/src/Slam

ENV DEBIAN_FRONTEND=noninteractive
RUN cd /home/gokart_ws && sudo apt update && rosdep install --from-paths src --ignore-src -r -y \ 
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/foxy/setup.bash && cd /home/gokart_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /home/gokart_ws/install/setup.bash

#RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
#RUN --mount=type=ssh git clone --recursive -b ros2_dev git@github.com:mlab-upenn/gokart-sensor.git
#RUN mkdir -p /home/gokart_ws/src
#RUN cp -r /gokart-sensor /home/gokart_ws/src

#RUN apt update && cd /home/gokart_ws && rosdep install --from-paths src --ignore-src -r -y \
    #&& apt clean \
    #&& rm -rf /var/lib/apt/lists/*
#RUN source /opt/ros/foxy/setup.bash && cd /home/gokart_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

#RUN source /opt/velodyne/velodyne-lidar-driver-ros2/setup.bash
#COPY --from=0 /home/gokart_ws/install /home/gokart_ws/install
#COPY --from=0 /home/gokart_ws/build /home/gokart_ws/build
#RUN source /opt/ros/foxy/setup.bash
#RUN source /home/gokart_ws/install/setup.bash
ENV LIBGL_ALWAYS_SOFTWARE=true
ENV QT_QPA_PLATFORM=xcb
#ENV XDG_RUNTIME_DIR=/tmp/runtime-root
#RUN pip install opencv-python-headless
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
# GUI Tool
#RUN apt update \
    #&& apt install -y ros-foxy-rviz2 \
    #&& apt clean \
    #&& rm -rf /var/lib/apt/lists/*
#ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
CMD source /opt/ros/foxy/setup.bash && source ./install/setup.bash && ros2 launch yolov8_pkg yolov8_node.launch.py
CMD source /opt/ros/foxy/setup.bash && source /home/gokart_ws/install/setup.bash && ros2 launch yolov8_pkg yolov8_node.launch.py
