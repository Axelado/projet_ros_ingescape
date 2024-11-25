from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'ingescape_ros2_interface'

launch_files = glob(os.path.join('launch', '*.launch.py'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools', 'cv_bridge', 'base64', 'cv2',],
    zip_safe=True,
    maintainer='Axel Niato',
    maintainer_email='axelniato@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_on_whiteboard = ingescape_ros2_interface.image_on_whiteboard:main",
            "tf_listener_node = ingescape_ros2_interface.tf_listener_node:main", 
            "joy_publisher_node = ingescape_ros2_interface.joy_publisher_node:main",
            "pose_publish_from_room_number = ingescape_ros2_interface.pose_publish_from_room_number:main",
            ],
    },
)
