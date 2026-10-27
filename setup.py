from setuptools import setup
import os
from glob import glob

package_name = 'vision_guided_bin_picking'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mjakinin',
    maintainer_email='maxim.mjakinin@gmail.com',
    description='Vision-Guided Industrial Bin-Picking and Assembly Stack using Franka Emika Panda, MoveIt 2, and ROS 2 Humble',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_detector_node = vision_guided_bin_picking.vision_detector_node:main',
            'planning_scene_node = vision_guided_bin_picking.planning_scene_node:main',
            'bin_picking_orchestrator_node = vision_guided_bin_picking.bin_picking_orchestrator_node:main',
        ],
    },
)
