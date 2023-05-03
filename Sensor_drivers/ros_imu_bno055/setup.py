# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup
#
# # fetch values from package.xml
# setup_args = generate_distutils_setup(
#     packages=['ros_imu_bno055'],
#     package_dir={'': 'include'},
# )
#
# setup(**setup_args)

from setuptools import setup
import os
from glob import glob

package_name = 'ros_imu_bno055'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhijun Zhuang',
    maintainer_email='zhijunz@seas.upenn.edu',
    description='ros_imu_bno055 driver in ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_ros = ros_imu_bno055.imu_ros:main',
            'imu_calibration = ros_imu_bno055.imu_calibration:main'
        ],
    },
)
