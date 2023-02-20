#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ros/testimu/src/ros_imu_bno055"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ros/testimu/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ros/testimu/install/lib/python3/dist-packages:/home/ros/testimu/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ros/testimu/build" \
    "/usr/bin/python3" \
    "/home/ros/testimu/src/ros_imu_bno055/setup.py" \
     \
    build --build-base "/home/ros/testimu/build/ros_imu_bno055" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ros/testimu/install" --install-scripts="/home/ros/testimu/install/bin"
