import os
from glob import glob

from setuptools import find_packages, setup


package_name = "dolbotz"


def _config_data_files():
    """config/ 전체를 재귀적으로 share/dolbotz/config/ 아래에 설치되도록
    (하위 디렉토리별 (dest, [files]) 튜플) 나열한다. .pt/.bin/.xml 등 모델
    가중치 파일도 확장자 구분 없이 전부 포함된다 — dolbotz.utils.paths가
    colcon install 환경에서 이 경로를 찾는다."""
    entries = []
    config_root = "config"
    for dirpath, _dirnames, filenames in os.walk(config_root):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, config_root)
        dest = os.path.join("share", package_name, "config") if rel == "." \
            else os.path.join("share", package_name, "config", rel)
        entries.append((dest, [os.path.join(dirpath, f) for f in filenames]))
    return entries


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        *_config_data_files(),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "pyserial",  # led_bridge_node (봄 미션 LED 시리얼 브릿지) 전용
    ],
    zip_safe=True,
    maintainer="j",
    maintainer_email="j@example.com",
    description="ROS2 nodes for terrain side-slope detection and related robot utilities.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "slope_decision = dolbotz.drive_area.slope_decision:main",
            "flat_drive = dolbotz.drive_area.flat_drive:main",
            "gradient_map = dolbotz.drive_area.gradient_map:main",
            "elevation_map = dolbotz.drive_area.elevation_map:main",
            "segmentation = dolbotz.drive_area.segmentation:main",
            "arm_visualizer = dolbotz.arm_visualizer:main",
            "purepursuit = dolbotz.purepursuit:main",
            "summer_supply = dolbotz.missions.summer_supply:main",
            "drive_supply_detector = dolbotz.missions.drive_supply_detector:main",
            "spring_ifof = dolbotz.missions.spring_ifof:main",
            "led_relay = dolbotz.missions.led_relay:main",
            "led_bridge_node = dolbotz.missions.led_bridge_node:main",
            "summer_traffic = dolbotz.missions.summer_traffic:main",
            "fall_marker = dolbotz.missions.fall_marker:main",
            "escort_follow = dolbotz.missions.escort_follow:main",
            "slope_visualizer = dolbotz.utils.slope_visualizer:main",
            "terrain_viz_relay = dolbotz.utils.terrain_viz_relay:main",
            "path_camera_overlay_relay = dolbotz.utils.path_camera_overlay_relay:main",
        ],
    },
)
