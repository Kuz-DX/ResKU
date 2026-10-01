#!/usr/bin/env python3
# 이 파일을 직접 실행할 때 Python 3 인터프리터를 사용합니다.
"""Publish the calibrated, static cam_link -> arm_camera_link transform.

The RealSense driver owns arm_camera_link -> arm_camera_color_optical_frame.
This node owns only the mechanical mounting calibration, so it must run once
per robot and must not be included by individual perception launches.
"""
# 이 노드는 기계적 장착 보정인 cam_link -> arm_camera_link 변환만 발행합니다.
# 카메라 드라이버가 소유한 optical frame 변환과 중복되지 않도록 로봇당 한 번만 실행합니다.

# 라디안 입력 검증과 RPY 각도의 삼각 함수 계산에 사용합니다.
import math

# ROS 2 Python 클라이언트의 초기화, 이벤트 루프 및 종료 기능입니다.
import rclpy
# 좌표계 간 위치와 회전을 표현하는 TF 메시지 형식입니다.
from geometry_msgs.msg import TransformStamped
# ROS 2 노드의 파라미터, 시계, 로그 기능을 제공합니다.
from rclpy.node import Node
# 변하지 않는 좌표 변환을 TF에 발행하는 브로드캐스터입니다.
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# Roll, Pitch, Yaw 회전을 XYZW 순서의 쿼터니언으로 변환합니다.
def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    """Return an XYZW quaternion for intrinsic fixed-axis RPY angles."""
    # roll 각도의 절반을 쿼터니언 변환에 사용합니다.
    half_roll = roll * 0.5
    # pitch 각도의 절반을 쿼터니언 변환에 사용합니다.
    half_pitch = pitch * 0.5
    # yaw 각도의 절반을 쿼터니언 변환에 사용합니다.
    half_yaw = yaw * 0.5
    # roll 절반각의 코사인과 사인 값을 각각 구합니다.
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    # pitch 절반각의 코사인과 사인 값을 각각 구합니다.
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    # yaw 절반각의 코사인과 사인 값을 각각 구합니다.
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    # 고정축 RPY 회전의 쿼터니언 성분을 x, y, z, w 순서로 반환합니다.
    return (
        # x축 회전 성분을 계산합니다.
        sr * cp * cy - cr * sp * sy,
        # y축 회전 성분을 계산합니다.
        cr * sp * cy + sr * cp * sy,
        # z축 회전 성분을 계산합니다.
        cr * cp * sy - sr * sp * cy,
        # 쿼터니언의 스칼라 성분 w를 계산합니다.
        cr * cp * cy + sr * sp * sy,
    )


# 카메라 장착 외부 파라미터를 검증하고 정적 TF로 발행하는 ROS 노드입니다.
class ArmCameraExtrinsicsBroadcaster(Node):
    # 노드 이름과 파라미터를 초기화하고 유효한 변환을 발행합니다.
    def __init__(self):
        # ROS 그래프에서 사용할 노드 이름을 설정합니다.
        super().__init__('arm_camera_extrinsics')
        # 명시적으로 활성화하기 전에는 보정 TF를 발행하지 않도록 기본값을 둡니다.
        self.declare_parameter('enabled', False)
        # 변환의 부모 좌표계 기본값을 지정합니다.
        self.declare_parameter('parent_frame', 'cam_link')
        # 변환의 자식 좌표계 기본값을 지정합니다.
        self.declare_parameter('child_frame', 'arm_camera_link')
        # 부모 기준 자식 좌표계의 XYZ 이동량 기본값을 선언합니다.
        self.declare_parameter('translation_xyz', [0.0, 0.0, 0.0])
        # 부모 기준 자식 좌표계의 RPY 회전량 기본값을 라디안으로 선언합니다.
        self.declare_parameter('rotation_rpy', [0.0, 0.0, 0.0])

        # 보정 발행이 비활성화되어 있으면 좌표 변환을 만들지 않습니다.
        if not bool(self.get_parameter('enabled').value):
            # 실제 측정값 입력과 활성화가 필요함을 경고 로그로 알립니다.
            self.get_logger().warn(
                'Camera extrinsics are disabled; no cam_link TF is published. '
                'Enter measured values in arm_camera_extrinsics.yaml and set '
                'enabled:=true before using arm-camera spatial control.')
            # 이후 검증과 발행 로직을 건너뛰고 노드 초기화를 마칩니다.
            return

        # 프레임 이름 파라미터를 문자열로 읽고 앞뒤 슬래시를 제거합니다.
        parent = str(self.get_parameter('parent_frame').value).strip('/')
        # 자식 프레임 이름도 문자열로 읽어 앞뒤 슬래시를 제거합니다.
        child = str(self.get_parameter('child_frame').value).strip('/')
        # XYZ 이동량 파라미터를 순회 가능한 리스트로 읽습니다.
        translation = list(self.get_parameter('translation_xyz').value)
        # RPY 회전량 파라미터를 순회 가능한 리스트로 읽습니다.
        rotation = list(self.get_parameter('rotation_rpy').value)
        # 빈 프레임 이름이나 부모와 자식이 같은 잘못된 변환을 거부합니다.
        if not parent or not child or parent == child:
            # 설정 오류가 TF 트리의 잘못된 구조로 이어지지 않도록 원인을 보고합니다.
            raise ValueError('parent_frame and child_frame must be distinct, non-empty frame IDs.')
        # 이동과 회전 모두 정확히 세 축 성분을 갖는지 확인합니다.
        if len(translation) != 3 or len(rotation) != 3:
            # 벡터 길이가 맞지 않으면 변환 메시지를 만들기 전에 중단합니다.
            raise ValueError('translation_xyz and rotation_rpy must each contain exactly 3 values.')
        # 여섯 값이 숫자로 변환 가능하고 NaN 또는 무한대가 아닌지 검사합니다.
        if not all(math.isfinite(float(value)) for value in translation + rotation):
            # 유한하지 않은 보정값이 TF에 발행되는 것을 방지합니다.
            raise ValueError('Camera extrinsics must contain only finite numeric values.')

        # 부모-자식 좌표계 변환을 담을 ROS 메시지를 생성합니다.
        transform = TransformStamped()
        # 현재 ROS 시각을 변환 헤더의 타임스탬프로 기록합니다.
        transform.header.stamp = self.get_clock().now().to_msg()
        # 변환이 기준으로 삼는 부모 좌표계 이름을 지정합니다.
        transform.header.frame_id = parent
        # 변환으로 위치가 정의되는 자식 좌표계 이름을 지정합니다.
        transform.child_frame_id = child
        # 보정 이동 벡터의 x 성분을 메시지에 기록합니다.
        transform.transform.translation.x = float(translation[0])
        # 보정 이동 벡터의 y 성분을 메시지에 기록합니다.
        transform.transform.translation.y = float(translation[1])
        # 보정 이동 벡터의 z 성분을 메시지에 기록합니다.
        transform.transform.translation.z = float(translation[2])
        # RPY 라디안 값을 쿼터니언으로 바꾸고 XYZW 성분을 분리합니다.
        qx, qy, qz, qw = _quaternion_from_rpy(*(float(value) for value in rotation))
        # 회전 쿼터니언의 x 성분을 메시지에 기록합니다.
        transform.transform.rotation.x = qx
        # 회전 쿼터니언의 y 성분을 메시지에 기록합니다.
        transform.transform.rotation.y = qy
        # 회전 쿼터니언의 z 성분을 메시지에 기록합니다.
        transform.transform.rotation.z = qz
        # 회전 쿼터니언의 스칼라 w 성분을 메시지에 기록합니다.
        transform.transform.rotation.w = qw

        # 이 노드가 살아 있는 동안 고정 변환을 발행할 브로드캐스터를 생성합니다.
        self.broadcaster = StaticTransformBroadcaster(self)
        # 검증과 변환 구성이 끝난 정적 TF를 발행합니다.
        self.broadcaster.sendTransform(transform)
        # 발행된 프레임 쌍과 중복 발행 방지 주의사항을 로그에 남깁니다.
        self.get_logger().info(
            f'Publishing calibrated static TF {parent} -> {child}; '
            'do not launch another broadcaster for this child frame.')


# ROS 클라이언트를 초기화하고 TF 발행 노드를 계속 실행합니다.
def main(args=None):
    # 명령행 인자를 ROS 초기화에 전달합니다.
    rclpy.init(args=args)
    # 파라미터를 읽고 보정 TF 발행 노드를 생성합니다.
    node = ArmCameraExtrinsicsBroadcaster()
    # ROS 콜백과 노드 이벤트를 처리하는 실행 루프를 시작합니다.
    try:
        rclpy.spin(node)
    # 정상 종료나 실행 중 예외가 있어도 생성한 자원을 해제합니다.
    finally:
        # 노드의 ROS 자원과 그래프 등록을 정리합니다.
        node.destroy_node()
        # ROS 클라이언트 라이브러리를 종료합니다.
        rclpy.shutdown()


# 이 파일을 직접 실행할 때만 진입점을 호출하고 import 시에는 실행하지 않습니다.
if __name__ == '__main__':
    # ROS 2 노드 초기화와 실행 루프를 시작합니다.
    main()
