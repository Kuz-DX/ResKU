from setuptools import find_packages, setup

package_name = 'visual_landmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rim',
    maintainer_email='harim9340@gmail.com',
    description='Back-to-Base visual landmark keyframe capture (stop-and-capture).',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'visual_landmark_node = visual_landmark.visual_landmark_node:main',
        ],
    },
)
