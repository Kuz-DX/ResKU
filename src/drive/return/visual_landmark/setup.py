from setuptools import find_packages, setup

package_name = 'visual_landmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'HARDWARE_VALIDATION.md', 'CAMERA_VPR_TODO.md', 'ADAPTIVE_VISUAL.md', 'MOUNTED_CAMERA_SETUP.md', 'REAR_EXTRINSIC_CHECKERBOARD.md', 'VPR_RUN_COMMANDS.md', 'STATIONARY_VPR_DEBUG.md']),
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
            'validate_camera_calibration = visual_landmark.calibration_tool:main',
            'evaluate_landmark_matching = visual_landmark.evaluate_matching:main',
            'rear_extrinsic = visual_landmark.extrinsic_tool:main',
            'vpr_viewer = visual_landmark.viewer:main',
            'vpr_preflight = visual_landmark.preflight:main',
            'visual_check_scheduler_node = visual_landmark.visual_check_scheduler_node:main',
            'visual_localization_node = visual_landmark.visual_localization_node:main',
            'visual_landmark_node = visual_landmark.visual_landmark_node:main',
        ],
    },
)
