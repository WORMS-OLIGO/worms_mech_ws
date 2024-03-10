from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'worms_mech'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacobdr',
    maintainer_email='jacobrod@mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_interface = worms_mech.hardware_interface:main',
            'simple_hardware_interface = worms_mech.simple_hardware_interface:main',
            'gait_parser = worms_mech.gait_parser:main',
            'camera_scanner = worms_mech.camera_scanner:main',
            'gait_action_node = worms_mech.gait_action_node:main',
            'gait_manager = worms_mech.gait_manager:main',
            'controller_node = worms_mech.controller_node:main',
            'testbed_interface = worms_mech.testbed_interface:main',
            'joint_command_publisher = worms_mech.joint_command_publisher:main',
            'group_joint_controller = worms_mech.group_joint_controller:main',
            'joystick_control = worms_mech.joystick_control:main',
            'stand_node = worms_mech.stand_node:main'
        ],
    },
)
