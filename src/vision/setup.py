import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'vision'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('camera/launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config', 'camera'),
            glob('camera/config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'models'),
            glob('models/*.xml') + glob('models/*.bin'),
        ),
    ],
    install_requires=[
        'setuptools',
        'pupil-apriltags',
        'openvino',
    ],
    zip_safe=True,
    maintainer='dolbat',
    maintainer_email='dolbat@example.com',
    description='ROS 2 vision nodes for AprilTag and OpenVINO person detection.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'apriltag = vision.apriltag:main',
            'person_detection = vision.person_detection:main',
            'export_rfdetr_openvino = vision.export_rfdetr_openvino:main',
        ],
    },
)
