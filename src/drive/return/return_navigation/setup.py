import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'return_navigation'

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
        'Manual-drive path recording and closed-loop autonomous return '
        '(180-degree turn + fixed mission-frame path following).'
    ),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'manual_path_recorder_node = return_navigation.manual_path_recorder_node:main',
            'return_state_machine_node = return_navigation.return_state_machine_node:main',
            'return_path_follower_node = return_navigation.return_path_follower_node:main',
        ],
    },
)
