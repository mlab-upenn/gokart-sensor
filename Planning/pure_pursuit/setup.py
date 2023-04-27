import os
from glob import glob
from setuptools import setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # Copy files from the src to the share directory
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzjun',
    maintainer_email='zhijunz@seas.upenn.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # Add new executable here
    entry_points={
        'console_scripts': [
            f'python_node = {package_name}.python_node:main',
            f'visualize_node = {package_name}.visualize_node:main'
        ],
    },
)
