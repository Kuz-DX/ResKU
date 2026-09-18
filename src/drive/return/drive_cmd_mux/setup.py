import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drive_cmd_mux'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rim',
    maintainer_email='harim9340@gmail.com',
    description=(
        'Arbitrates between /cmd_vel_manual and /cmd_vel_return based on '
        '/mission/return/state, publishing /cmd_vel.'
    ),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_cmd_mux_node = drive_cmd_mux.drive_cmd_mux_node:main',
        ],
    },
)
