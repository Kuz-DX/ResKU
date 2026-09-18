import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'manual_return_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rim',
    maintainer_email='harim9340@gmail.com',
    description=(
        'Pre-flight virtual test rig (vcan0 fake motors + IMU + RViz) for '
        'the manual+return mission -- runs the real production stack '
        'against simulated hardware.'
    ),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_rmd_x8_hardware = manual_return_sim.sim_rmd_x8_hardware:main',
            'sim_debug_viz = manual_return_sim.sim_debug_viz:main',
        ],
    },
)
