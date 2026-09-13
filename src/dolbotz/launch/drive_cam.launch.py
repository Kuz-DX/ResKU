"""공용 YAML의 drive_cam 설정으로 주행 카메라를 실행한다."""

from dolbotz.utils.realsense_camera_launch import generate_camera_launch_description


def generate_launch_description():
    return generate_camera_launch_description('drive_cam')
