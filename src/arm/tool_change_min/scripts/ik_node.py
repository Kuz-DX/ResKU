#!/usr/bin/env python3
# 이 파일을 Python 3 인터프리터로 직접 실행할 때 사용할 실행 파일을 지정합니다.
"""Numerical URDF IK service for safe Cartesian lines.

FK is the ordered product of the unmodified Xacro joint-origin transforms and
axis rotations.  The 6x6 Jacobian is central finite difference; each IK update
uses damped least squares, ``dq=J^T(JJ^T+lambda^2 I)^-1 e``.  A requested line
given as direction/distance uses ``ceil(distance/ik_step_m)`` points with
fixed orientation. A pose target uses the larger position-distance and
SO(3)-rotation step counts; position is linear and orientation follows the
shortest rotation. Each point is solved and checked against the equal
URDF/hardware soft limits before one complete JointTrajectory is returned.
"""
# 타입 표기 평가 시점과 전방 참조 처리를 위해 미래의 어노테이션 동작을 활성화합니다.
from __future__ import annotations

# 거리·속도 입력의 유한성 검사와 관련 수학 계산에 사용합니다.
import math
import time

# 관절 벡터 및 변환 행렬 계산을 위한 수치 배열 라이브러리입니다.
import numpy as np
# ROS 2 Python 클라이언트를 초기화하고 실행하는 데 사용합니다.
import rclpy
# 설치된 ROS 패키지의 공유 디렉터리 경로를 찾습니다.
from ament_index_python.packages import get_package_share_directory
# ROS 2 노드의 파라미터, 구독, 서비스를 관리하는 기본 클래스입니다.
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# 현재 로봇 관절 위치가 담긴 토픽 메시지 형식입니다.
from sensor_msgs.msg import JointState
# IK 결과를 이동 궤적 점으로 반환할 때 사용하는 메시지 형식입니다.
from trajectory_msgs.msg import JointTrajectoryPoint

# 정지 자세 구성 파일을 읽고 필수 설정이 있는지 확인합니다.
from tool_change_min.config import require_for_stop_state
# URDF와 하드웨어 설정을 사용해 순기구학·역기구학을 계산합니다.
from tool_change_min.kinematics import ArmKinematics
# 직선 이동 및 단일 점 풀이 ROS 서비스의 요청·응답 형식입니다.
from tool_change_min.srv import LinearMove, LinearMoveToPose, SolvePoint


# 초 단위 실수를 ROS Duration 메시지의 초·나노초 필드로 변환합니다.
def _duration(seconds: float):
    # Duration 메시지는 이 함수에서만 필요하므로 호출 시점에 가져옵니다.
    from builtin_interfaces.msg import Duration
    # ROS가 요구하는 시간 메시지 객체를 새로 만듭니다.
    result = Duration()
    # 정수 초 부분을 분리해 저장합니다.
    result.sec = int(seconds)
    # 남은 소수 초를 나노초로 바꾸어 저장합니다.
    result.nanosec = int(round((seconds - result.sec) * 1.0e9))
    # 변환한 ROS 시간 메시지를 호출자에게 돌려줍니다.
    return result


def _rotation_from_xyzw(xyzw: list[float]) -> np.ndarray:
    """Return a rotation matrix from a normalized ROS quaternion [x, y, z, w]."""
    quaternion = np.asarray(xyzw, dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.all(np.isfinite(quaternion)) or norm < 1.0e-12:
        raise ValueError("target orientation_xyzw must be a finite non-zero quaternion")
    x, y, z, w = quaternion / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


# IK 요청을 받고 현재 관절 상태를 바탕으로 궤적을 만드는 ROS 2 노드입니다.
class IkNode(Node):
    # 노드 생성 시 설정·기구학 모델과 ROS 통신 경로를 초기화합니다.
    def __init__(self):
        # ROS 그래프에 등록될 노드 이름을 지정합니다.
        super().__init__("ik_node")
        # 이 패키지의 기본 설정 파일이 위치한 설치 경로를 구합니다.
        share = get_package_share_directory("tool_change_min")
        # 로봇 설명 패키지의 URDF/Xacro 기본 경로를 구합니다.
        description = get_package_share_directory("tool_manipulator_description")
        # 하드웨어 설정 패키지의 기본 경로를 구합니다.
        legacy = get_package_share_directory("tool_manipulator_bringup")
        # 자세 설정 YAML 경로를 파라미터로 노출하고 기본값을 지정합니다.
        self.declare_parameter("poses_yaml", f"{share}/config/poses.yaml")
        # 기구학 계산에 사용할 로봇 URDF/Xacro 경로를 파라미터로 노출합니다.
        self.declare_parameter("urdf_xacro", f"{description}/urdf/tool_manipulator.urdf.xacro")
        # 관절 하드웨어 한계 설정 파일 경로를 파라미터로 노출합니다.
        self.declare_parameter("hardware_yaml", f"{legacy}/config/hardware.yaml")
        # 정지 자세 설정을 읽고 필수 항목이 빠졌으면 초기화를 실패시킵니다.
        self.config = require_for_stop_state(self.get_parameter("poses_yaml").value, tool_id=None)
        # 설정에 적힌 순서를 IK 벡터와 응답 관절 이름의 기준으로 보관합니다.
        self.joint_names = tuple(self.config["joint_names"])
        # URDF 기구학과 하드웨어 관절 한계를 결합한 계산 모델을 만듭니다.
        self.model = ArmKinematics(self.get_parameter("urdf_xacro").value,
                                   self.get_parameter("hardware_yaml").value)
        # 설정과 URDF의 관절 순서가 다르면 벡터 대응이 잘못되므로 즉시 중단합니다.
        if self.joint_names != self.model.joint_names:
            # 두 순서를 함께 보여 주어 설정 불일치를 진단할 수 있게 합니다.
            raise ValueError(f"poses joint order {self.joint_names} differs from URDF {self.model.joint_names}")
        # 단일 JointState 메시지에서 검증된 여섯 축만 저장합니다. raw/deg 토픽은
        # 사용하지 않으며 서로 다른 메시지의 관절 값을 합치지도 않습니다.
        self.joint_state: np.ndarray | None = None
        self.joint_state_received_at: float | None = None
        self.joint_state_error = "no /joint_states received"
        self.joint_state_sequence = 0
        self.joint_state_max_age_s = float(self.config["feedback"]["joint_state_max_age_s"])
        if not math.isfinite(self.joint_state_max_age_s) or self.joint_state_max_age_s <= 0.0:
            raise ValueError("feedback.joint_state_max_age_s must be finite and > 0")
        # /joint_states를 구독해 한 메시지 단위의 완전한 관절 스냅샷만 수락합니다.
        self.create_subscription(JointState, "/joint_states", self._joint_state, 20)
        # 직선 Cartesian 이동 요청을 처리하는 서비스를 등록합니다.
        self.create_service(LinearMove, "linear_move", self._linear_move)
        # 목표 pose까지 위치 직선·자세 최단 회전으로 이동하는 서비스를 등록합니다.
        self.create_service(LinearMoveToPose, "linear_move_to_pose", self._linear_move_to_pose)
        # 목표 점 하나의 IK 계산 요청을 처리하는 서비스를 등록합니다.
        self.create_service(SolvePoint, "solve_point", self._solve_point)

    # 하나의 JointState 메시지에서 검증된 여섯 축 스냅샷을 수락합니다.
    def _joint_state(self, message: JointState):
        try:
            if len(message.name) != len(message.position):
                raise ValueError("name/position length mismatch")
            values = {}
            for name, position in zip(message.name, message.position):
                if name in self.joint_names:
                    if name in values:
                        raise ValueError(f"duplicate joint {name}")
                    values[name] = float(position)
            missing = [name for name in self.joint_names if name not in values]
            if missing:
                raise ValueError("missing joints in one message: " + ", ".join(missing))
            q = np.asarray([values[name] for name in self.joint_names], dtype=float)
            if not np.all(np.isfinite(q)):
                raise ValueError("non-finite joint position")
            if not self.model.within_limits(q, float(self.config["motion"]["ik_limit_margin_rad"])):
                raise ValueError("joint state violates hardware/URDF soft limits")
            self.joint_state = q
            self.joint_state_received_at = time.monotonic()
            self.joint_state_sequence += 1
            self.joint_state_error = ""
        except ValueError as exc:
            self.joint_state_error = str(exc)
            self.get_logger().warning(f"rejected /joint_states snapshot: {exc}")

    # 내부 상태를 설정된 관절 순서의 NumPy 벡터로 반환합니다.
    def _current(self) -> np.ndarray:
        if self.joint_state is None or self.joint_state_received_at is None:
            raise ValueError("no valid /joint_states snapshot: " + self.joint_state_error)
        age = time.monotonic() - self.joint_state_received_at
        if age > self.joint_state_max_age_s:
            raise ValueError(f"stale /joint_states age={age:.3f}s exceeds "
                             f"feedback.joint_state_max_age_s={self.joint_state_max_age_s:.3f}s")
        return self.joint_state.copy()

    # 직선 이동 서비스 요청을 Cartesian 경로와 관절 궤적으로 변환합니다.
    def _linear_move(self, request, response):
        # 잘못된 입력이나 IK 실패를 서비스 실패 응답으로 변환하기 위해 감쌉니다.
        try:
            # 기준 좌표계에서 지정된 방향의 x, y, z 성분을 실수 배열로 읽습니다.
            direction = np.array([request.direction_base.x, request.direction_base.y,
                                  request.direction_base.z], dtype=float)
            # 속도는 유한한 양수여야 시간 계산이 유효합니다.
            if not math.isfinite(request.speed_m_s) or request.speed_m_s <= 0.0:
                # 유효하지 않은 속도 입력을 아래 예외 처리로 전달합니다.
                raise ValueError("speed_m_s must be finite and > 0")
            # 이동 간격과 관절 한계 여유값이 포함된 설정을 가져옵니다.
            motion = self.config["motion"]
            # 현재 자세에서 출발하는 직선 경로를 IK로 풀고 한계값도 검사합니다.
            path = self.model.linear_path(self._current(), direction, request.distance_m,
                                          float(motion["ik_step_m"]),
                                          float(motion["ik_limit_margin_rad"]))
            # 궤적의 각 위치 값이 어느 관절에 대응하는지 응답에 기록합니다.
            response.trajectory.joint_names = list(self.joint_names)
            # 생성된 경로 점 개수를 이후 시간 배분에 사용합니다.
            steps = len(path)
            # 각 IK 해를 순회하며 ROS 궤적 점으로 변환합니다.
            for index, q in enumerate(path, start=1):
                # 궤적의 한 시점을 나타내는 메시지를 만듭니다.
                point = JointTrajectoryPoint()
                # NumPy 관절 벡터를 ROS 메시지가 요구하는 리스트로 바꿉니다.
                point.positions = q.tolist()
                # 거리 비율과 요청 속도를 이용해 시작 후 누적 시간을 지정합니다.
                point.time_from_start = _duration(request.distance_m * index / steps / request.speed_m_s)
                # 완성한 점을 응답 궤적의 끝에 추가합니다.
                response.trajectory.points.append(point)
            # 모든 점이 성공적으로 만들어졌음을 응답에 표시합니다.
            response.success, response.message = True, f"generated {steps} Cartesian IK points"
        # 한 점이라도 실패하면 부분 궤적을 반환하지 않도록 전체를 실패 처리합니다.
        except Exception as exc:  # A single failed point invalidates all points.
            # 실패 상태와 원인 메시지를 서비스 응답에 기록합니다.
            response.success, response.message = False, str(exc)
            # 혹시 일부 점이 추가되었더라도 안전하지 않은 부분 궤적을 비웁니다.
            response.trajectory.points.clear()
            # ROS 로그에도 요청 거부 사유를 남깁니다.
            self.get_logger().error(f"linear_move rejected: {exc}")
        # 성공 또는 실패 내용이 채워진 응답 객체를 반환합니다.
        return response

    # base_actuator 목표 pose까지 모든 중간점을 IK로 풀어 다점 궤적을 만듭니다.
    def _linear_move_to_pose(self, request, response):
        try:
            if request.target_pose.header.frame_id != "base_actuator":
                raise ValueError("linear_move_to_pose accepts only base_actuator targets")
            if not math.isfinite(request.speed_m_s) or request.speed_m_s <= 0.0:
                raise ValueError("speed_m_s must be finite and > 0")

            target = np.eye(4)
            target[:3, 3] = [request.target_pose.pose.position.x,
                              request.target_pose.pose.position.y,
                              request.target_pose.pose.position.z]
            target[:3, :3] = _rotation_from_xyzw([
                request.target_pose.pose.orientation.x,
                request.target_pose.pose.orientation.y,
                request.target_pose.pose.orientation.z,
                request.target_pose.pose.orientation.w,
            ])
            if not np.all(np.isfinite(target[:3, 3])):
                raise ValueError("target position must be finite")

            motion = self.config["motion"]
            current = self._current()
            start_pose = self.model.fk(current)
            distance = float(np.linalg.norm(target[:3, 3] - start_pose[:3, 3]))
            if distance <= 1.0e-12:
                raise ValueError("linear speed cannot time a zero-distance target")
            path = self.model.linear_path_to_pose(
                current, target, float(motion["ik_step_m"]),
                float(motion["ik_orientation_step_rad"]),
                float(motion["ik_limit_margin_rad"]))

            response.trajectory.joint_names = list(self.joint_names)
            total_time = distance / request.speed_m_s
            steps = len(path)
            for index, q in enumerate(path, start=1):
                point = JointTrajectoryPoint()
                point.positions = q.tolist()
                point.time_from_start = _duration(total_time * index / steps)
                response.trajectory.points.append(point)
            response.success = True
            response.message = (
                f"generated {steps} Cartesian pose-line IK points over {distance:.6f} m")
        except Exception as exc:
            response.success, response.message = False, str(exc)
            response.trajectory.points.clear()
            self.get_logger().error(f"linear_move_to_pose rejected: {exc}")
        return response

    # 기준 좌표계의 목표 pose 하나를 위치와 orientation 모두 만족하도록 IK로 풉니다.
    def _solve_point(self, request, response):
        # 요청 검증과 기구학 계산 오류를 실패 응답으로 바꾸기 위해 감쌉니다.
        try:
            # 이 서비스는 좌표계 변환을 수행하지 않으므로 기준 프레임을 제한합니다.
            if request.target_pose.header.frame_id != "base_actuator":
                # 지원하지 않는 프레임이면 잘못된 요청으로 처리합니다.
                raise ValueError("solve_point accepts only base_actuator targets")
            # Cartesian position과 quaternion orientation을 목표 강체변환으로 구성합니다.
            target = np.eye(4)
            target[:3, 3] = [request.target_pose.pose.position.x,
                              request.target_pose.pose.position.y,
                              request.target_pose.pose.position.z]
            target[:3, :3] = _rotation_from_xyzw([
                request.target_pose.pose.orientation.x,
                request.target_pose.pose.orientation.y,
                request.target_pose.pose.orientation.z,
                request.target_pose.pose.orientation.w,
            ])
            if not np.all(np.isfinite(target[:3, 3])):
                raise ValueError("target position must be finite")
            # 현재 관절 자세를 seed로 삼아 목표 위치와 방향을 동시에 풉니다.
            q = self.model.solve(target, self._current(), float(self.config["motion"]["ik_limit_margin_rad"]))
            # 계산 성공과 관절 이름·해를 서비스 응답에 기록합니다.
            response.success, response.joint_names, response.positions = True, list(self.joint_names), q.tolist()
            response.message = "IK solved for base_actuator Cartesian pose"
        # 프레임 검증이나 IK 계산이 실패하면 성공으로 오인되지 않게 응답을 채웁니다.
        except Exception as exc:
            # 실패 상태와 예외 설명을 응답으로 돌려줍니다.
            response.success, response.message = False, str(exc)
            # 로그에서 서비스 실패 원인을 확인할 수 있도록 기록합니다.
            self.get_logger().error(f"solve_point rejected: {exc}")
        # 처리 결과가 기록된 서비스 응답을 반환합니다.
        return response


# ROS 2를 초기화하고 노드를 실행하는 프로그램 진입점입니다.
def main():
    # ROS 클라이언트 라이브러리의 전역 상태를 초기화합니다.
    rclpy.init()
    # 필수 설정과 기구학 모델을 사용해 노드를 생성합니다.
    try:
        node = IkNode()
    # 설정 또는 모델 초기화가 실패하면 노드를 실행하지 않습니다.
    except Exception as exc:
        # There is intentionally no fallback configuration for robot motion.
        # 로봇 이동 설정 오류를 명시하고 대체 설정 없이 안전하게 종료합니다.
        rclpy.logging.get_logger("ik_node").error(f"configuration invalid; exiting: {exc}")
        # 초기화에 실패했으므로 ROS 클라이언트 상태를 정리합니다.
        rclpy.try_shutdown()
        # 노드가 없는 상태에서 함수가 계속 진행하지 않도록 반환합니다.
        return
    # 노드의 구독과 서비스 콜백을 ROS 이벤트 루프에서 처리합니다.
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 또는 launch에 의한 컨텍스트 종료는 정상 종료로 처리합니다.
        pass
    # 정상 종료나 실행 중 예외가 발생해도 노드와 ROS를 정리합니다.
    finally:
        # 노드가 만든 ROS 자원을 해제합니다.
        node.destroy_node()
        # 시그널 처리에서 이미 종료된 컨텍스트도 안전하게 정리합니다.
        rclpy.try_shutdown()


# 다른 모듈에서 가져다 쓸 때는 실행하지 않고 직접 실행할 때만 진입점을 호출합니다.
if __name__ == "__main__":
    # ROS 노드의 초기화 및 이벤트 루프를 시작합니다.
    main()
