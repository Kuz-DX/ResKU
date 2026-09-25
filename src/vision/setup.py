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
    ],
    install_requires=[
        'setuptools',
        'pupil-apriltags',
    ],
    zip_safe=True,
    maintainer='dolbat',
    maintainer_email='dolbat@example.com',
    description='ROS 2 vision nodes for AprilTag detection.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'apriltag = vision.apriltag:main',
        ],
    },
)
