import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cmd_servo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sushant',
    maintainer_email='mail@sushant.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "cmd_servo_py = cmd_servo_py.cmd_servo_py:main",
            "twist_publisher = cmd_servo_py.twist_publisher:main",
            "joint_trajectory_pub = cmd_servo_py.joint_trajectory_pub:main",
			"gripper_traj_pub = cmd_servo_py.gripper_traj_pub:main",
			"gripper_param = cmd_servo_py.gripper_param:main",
        ],
    },
)
