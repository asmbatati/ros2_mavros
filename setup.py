from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('rviz/*.rviz')),
        (os.path.join('share', package_name), glob('config/mavros/*.yaml')),
        (os.path.join('share', package_name), glob('config/kf/*.yaml')),
        (os.path.join('share', package_name), glob('config/geometric_controller/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asmbatati',
    maintainer_email='asmalbatati@hotmail.com',
    description='ROS 2 simulation packge of x500 UAV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = px4_offboard_control.offboard_control:main',
            'arm = px4_offboard_control.arm:main',
            'takeoff = px4_offboard_control.takeoff:main',
            'arm_takeoff = px4_offboard_control.arm_takeoff:main',
            'circle = px4_offboard_control.circular_traj:main'
        ],
    },
)
