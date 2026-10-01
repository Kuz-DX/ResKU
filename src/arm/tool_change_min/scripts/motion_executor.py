#!/usr/bin/env python3
# 이 파일을 직접 실행할 때 사용할 Python 3 인터프리터를 지정합니다.
"""Fail-closed FollowJointTrajectory adapter for the six-joint arm.

Named poses are converted to one controller trajectory point.  Cartesian IK
paths retain their ``ceil(distance/step)`` interpolation from ``ik_node`` and
are forwarded unchanged.  Wrist lock reads the current yaw as a baseline,
commands ``baseline + delta_rad`` while preserving the other five current
positions, then computes and logs ``joint_states_after - baseline``; it never
uses an absolute yaw target or retries a failed controller goal.
"""
# 이 노드는 검증된 궤적만 6축 팔 컨트롤러 액션으로 전달합니다.
# 제어기 실패나 제한 시간 초과를 자동 재시도하지 않고 호출자에게 실패로 반환합니다.
# 손목 회전량은 명령값이 아니라 실제 JointState 피드백 변화량으로 계산합니다.
# 타입 어노테이션을 지연 평가해 전방 참조를 지원합니다.
from __future__ import annotations

# 시간 제한과 응답 대기 시각을 계산합니다.
import math
# Future 완료 대기를 짧은 간격으로 양보하며 단조 시계를 사용합니다.
import time

# ROS 2 클라이언트의 초기화, 이벤트 루프 및 종료 기능을 제공합니다.
import rclpy
# FollowJointTrajectory 액션 결과 상태 상수를 가져옵니다.
from action_msgs.msg import GoalStatus
# ROS 패키지가 설치된 공유 디렉터리 경로를 찾습니다.
from ament_index_python.packages import get_package_share_directory
# 관절 궤적 실행 액션 메시지 형식입니다.
from control_msgs.action import FollowJointTrajectory
# ROS 액션 서버에 목표를 보내고 결과를 받는 클라이언트입니다.
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
# 여러 스레드에서 ROS 콜백을 처리하는 실행기입니다.
from rclpy.executors import MultiThreadedExecutor
# ROS 2 노드의 파라미터, 로거, 구독 및 서비스 기능을 제공합니다.
from rclpy.node import Node
# 현재 관절 위치 피드백 토픽의 메시지 형식입니다.
from sensor_msgs.msg import JointState
# 관절 궤적과 궤적의 개별 시점을 표현하는 메시지 형식입니다.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 정지 자세 설정을 로드하고 필수 항목을 검증합니다.
from tool_change_min.config import require_for_stop_state
# URDF와 하드웨어 설정을 이용해 관절 순서와 제한을 확인합니다.
from tool_change_min.kinematics import ArmKinematics
# 자세 이동, 궤적 실행, 상대 손목 회전 서비스 형식입니다.
from tool_change_min.srv import ExecuteTrajectory, MoveNamedPose, RotateWristYaw


# 실수 초 단위를 ROS Duration 메시지의 정수 초와 나노초로 변환합니다.
def _duration(seconds: float):
    # 시간 메시지 형식은 변환 함수가 호출될 때 가져옵니다.
    from builtin_interfaces.msg import Duration
    # 반환할 ROS 시간 메시지를 만듭니다.
    result = Duration()
    # 전체 초와 나머지 소수 초를 초·나노초 필드로 나누어 저장합니다.
    result.sec, result.nanosec = int(seconds), int(round((seconds % 1.0) * 1.0e9))
    # ROS 궤적 시점에 사용할 Duration 객체를 반환합니다.
    return result


# 팔 컨트롤러 액션에 궤적을 전달하는 ROS 2 노드입니다.
class MotionExecutor(Node):
    # 설정, 기구학 모델, 피드백 구독, 액션 및 서비스 클라이언트를 초기화합니다.
    def __init__(self):
        # ROS 그래프에 등록할 노드 이름을 설정합니다.
        super().__init__("motion_executor")
        # tool_change_min 패키지의 설정 기본 경로를 찾습니다.
        share = get_package_share_directory("tool_change_min")
        # 로봇 URDF/Xacro가 설치된 설명 패키지 경로를 찾습니다.
        description = get_package_share_directory("tool_manipulator_description")
        # 하드웨어 설정이 들어 있는 bringup 패키지 경로를 찾습니다.
        bringup = get_package_share_directory("tool_manipulator_bringup")
        # 자세와 동작 설정 파일의 경로를 ROS 파라미터로 선언합니다.
        self.declare_parameter("poses_yaml", f"{share}/config/poses.yaml")
        # 기구학 계산에 사용할 URDF/Xacro 파일 경로를 선언합니다.
        self.declare_parameter("urdf_xacro", f"{description}/urdf/tool_manipulator.urdf.xacro")
        # 하드웨어 관절 제한 설정 파일의 경로를 선언합니다.
        self.declare_parameter("hardware_yaml", f"{bringup}/config/hardware.yaml")
        # FollowJointTrajectory 액션 서버의 이름을 기본 경로와 함께 선언합니다.
        self.declare_parameter("arm_action", "/arm_controller/follow_joint_trajectory")
        # 필수 자세 및 정지 상태 설정을 로드하고 검증합니다.
        self.config = require_for_stop_state(self.get_parameter("poses_yaml").value)
        # 서비스 응답 및 궤적 벡터의 기준 관절 순서를 보관합니다.
        self.joint_names = tuple(self.config["joint_names"])
        # URDF와 하드웨어 설정을 바탕으로 팔 기구학 및 제한 모델을 생성합니다.
        self.model = ArmKinematics(self.get_parameter("urdf_xacro").value,
                                   self.get_parameter("hardware_yaml").value)
        # YAML과 URDF의 관절 순서가 다르면 위치값 대응이 어긋나므로 시작을 거부합니다.
        if self.joint_names != self.model.joint_names:
            # 잘못된 관절 순서 설정을 식별할 수 있도록 예외를 발생시킵니다.
            raise ValueError("poses.yaml joint_names does not match the shared URDF")
        # 제어 입력은 한 개의 완전하고 최근인 /joint_states 메시지뿐입니다.
        # raw/degree 진단 토픽은 구독하지 않고, 서로 다른 메시지를 합치지 않습니다.
        self.joint_state: list[float] | None = None
        self.joint_state_received_at: float | None = None
        self.joint_state_error = "no /joint_states received"
        self.joint_state_sequence = 0
        self.joint_state_max_age_s = float(self.config["feedback"]["joint_state_max_age_s"])
        if not math.isfinite(self.joint_state_max_age_s) or self.joint_state_max_age_s <= 0.0:
            raise ValueError("feedback.joint_state_max_age_s must be finite and > 0")
        # JointState 피드백을 구독해 한 메시지 단위의 여섯 축 상태만 수락합니다.
        # Service callbacks wait for action/encoder feedback. Those callbacks
        # must not share the services' mutually-exclusive default group.
        self.feedback_group = ReentrantCallbackGroup()
        self.motion_fault = ""
        self.create_subscription(JointState, "/joint_states", self._joint_state, 20,
                                 callback_group=self.feedback_group)
        # 지정된 FollowJointTrajectory 서버에 목표 궤적을 보낼 액션 클라이언트를 만듭니다.
        self.client = ActionClient(self, FollowJointTrajectory, self.get_parameter("arm_action").value,
                                   callback_group=self.feedback_group)
        # 이름 있는 자세 이동 요청을 처리하는 서비스를 등록합니다.
        self.create_service(MoveNamedPose, "move_to_named_pose", self._move_named)
        # 전달된 관절 궤적의 검증 및 실행 요청을 처리하는 서비스를 등록합니다.
        self.create_service(ExecuteTrajectory, "execute_trajectory", self._execute)
        # 상대 손목 yaw 회전 요청을 처리하는 서비스를 등록합니다.
        self.create_service(RotateWristYaw, "rotate_wrist_yaw_relative", self._rotate_yaw)

    # 한 메시지에서 완전하고 안전한 여섯 축 관절 상태만 저장합니다.
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
            q = [values[name] for name in self.joint_names]
            if not all(math.isfinite(value) for value in q):
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

    # 현재 관절 위치를 설정된 관절 순서의 실수 리스트로 반환합니다.
    def _current(self) -> list[float]:
        if self.joint_state is None or self.joint_state_received_at is None:
            raise ValueError("no valid /joint_states snapshot: " + self.joint_state_error)
        age = time.monotonic() - self.joint_state_received_at
        if age > self.joint_state_max_age_s:
            raise ValueError(f"stale /joint_states age={age:.3f}s exceeds "
                             f"feedback.joint_state_max_age_s={self.joint_state_max_age_s:.3f}s")
        return self.joint_state.copy()

    # Future가 완료되거나 마감 시각에 도달할 때까지 기다리고 결과를 가져옵니다.
    def _await(self, future, deadline: float):
        # 작업 완료 전이고 단조 시계 기준 마감 전인 동안 대기합니다.
        while not future.done() and time.monotonic() < deadline:
            # 짧게 쉬며 다른 콜백과 스레드가 실행될 기회를 줍니다.
            time.sleep(0.01)
        # 마감까지 Future가 완료되지 않으면 제한 시간 초과로 처리합니다.
        if not future.done():
            # 호출자가 현재 작업을 실패로 처리하도록 예외를 전달합니다.
            raise TimeoutError("controller action response/result timed out")
        # 완료된 Future의 결과 또는 예외를 호출자에게 돌려줍니다.
        return future.result()

    # 궤적을 검증하고 FollowJointTrajectory 액션으로 전송해 최종 상태를 확인합니다.
    def _send(self, trajectory: JointTrajectory, timeout_s: float) -> str:
        if self.motion_fault:
            raise RuntimeError("motion blocked until restart and physical stop check: " + self.motion_fault)
        # 제한 시간은 유한한 양수여야 합니다.
        if timeout_s <= 0.0 or not math.isfinite(timeout_s):
            # 유효하지 않은 제한 시간으로 액션을 보내지 않습니다.
            raise ValueError("controller timeout must be finite and > 0")
        # 궤적은 설정된 모든 관절을 정확한 순서로 포함하고 최소 한 점이 있어야 합니다.
        if list(trajectory.joint_names) != list(self.joint_names) or not trajectory.points:
            # 잘못된 관절 목록이나 빈 경로는 컨트롤러에 전달하지 않습니다.
            raise ValueError("trajectory must contain all six configured joints and at least one point")
        # IK 경로에도 적용할 관절 제한 여유값을 설정에서 가져옵니다.
        margin = float(self.config["motion"]["ik_limit_margin_rad"])
        # 궤적의 모든 시점이 유효한지 순서대로 검사합니다.
        for index, point in enumerate(trajectory.points):
            # 각 점의 위치 배열이 설정된 전체 관절 수와 같은지 확인합니다.
            if len(point.positions) != len(self.joint_names):
                # 잘못된 시점 번호를 예외에 포함합니다.
                raise ValueError(f"trajectory point {index} has an invalid joint count")
            # URDF 및 하드웨어 soft limit에 여유값까지 적용해 위치를 검사합니다.
            if not self.model.within_limits(point.positions, margin):
                # 제한을 벗어난 시점이 있으면 전체 경로를 거부합니다.
                raise ValueError(f"trajectory point {index} violates hardware/URDF soft limits")
        # 지정된 제한 시간 동안 액션 서버가 준비되는지 기다립니다.
        if not self.client.wait_for_server(timeout_sec=timeout_s):
            # 액션 서버를 찾지 못하면 궤적 목표를 보낼 수 없습니다.
            raise TimeoutError(f"action server unavailable: {self.get_parameter('arm_action').value}")
        # FollowJointTrajectory 목표 메시지를 생성합니다.
        goal = FollowJointTrajectory.Goal()
        # 검증이 완료된 궤적을 액션 목표에 넣습니다.
        goal.trajectory = trajectory
        # 서버 대기 이후 목표 응답과 실행 결과가 공유할 절대 마감 시각을 계산합니다.
        deadline = time.monotonic() + timeout_s
        # 비동기로 목표를 전송하고 목표 수락 여부가 담긴 응답을 기다립니다.
        acceptance = self.client.send_goal_async(goal)
        try:
            goal_handle = self._await(acceptance, deadline)
        except Exception as exc:
            self.motion_fault = "goal acceptance unresolved; physical stop unconfirmed"
            # An acceptance can arrive after our timeout. Cancel that goal too.
            acceptance.add_done_callback(self._cancel_late_goal)
            raise RuntimeError(self.motion_fault) from exc
        # 액션 서버가 목표를 거부했으면 실행 실패로 처리합니다.
        if not goal_handle.accepted:
            # 거부된 목표에 대해 재시도하지 않고 호출자에게 예외를 전달합니다.
            raise RuntimeError("controller rejected FollowJointTrajectory goal")
        # 수락된 목표의 최종 실행 결과를 같은 마감 시각까지 기다립니다.
        try:
            result = self._await(goal_handle.get_result_async(), deadline)
        except Exception as exc:
            self.motion_fault = "controller result unresolved; physical stop unconfirmed"
            self._request_cancel(goal_handle)
            raise RuntimeError(self.motion_fault) from exc
        # 액션 결과가 성공 상태가 아니면 컨트롤러 상태와 상세 오류를 보고합니다.
        if (result.status != GoalStatus.STATUS_SUCCEEDED or
                result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            self.motion_fault = "controller reported trajectory failure"
            # 컨트롤러 실패 상태와 자체 오류 메시지를 예외에 담습니다.
            raise RuntimeError(f"controller result status={result.status}, error={result.result.error_string}")
        # 액션 성공을 서비스 응답에 전달할 메시지로 반환합니다.
        return "controller trajectory succeeded"

    def _request_cancel(self, handle):
        """Queue cancellation; neither this request nor its ACK proves a stop."""
        try:
            handle.cancel_goal_async()
            self.get_logger().error("Cancel requested; physical stop remains unconfirmed")
        except Exception as exc:
            self.get_logger().error(f"Cannot request cancel; physical stop unconfirmed: {exc}")

    def _cancel_late_goal(self, future):
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                self._request_cancel(handle)
        except Exception as exc:
            self.get_logger().error(f"Late goal response unresolved: {exc}")

    # 이름으로 지정된 자세를 설정에서 찾아 단일점 궤적으로 실행합니다.
    def _move_named(self, request, response):
        # 입력 이름과 설정, 제한값을 확인하고 액션 실행까지 수행합니다.
        try:
            # 설정된 이름 자세 맵에서 요청한 자세를 조회합니다.
            target = self.config["named_poses"].get(request.name)
            # 자세가 없거나 맵 형식이 아니면 알 수 없는 자세로 처리합니다.
            if not isinstance(target, dict):
                # 잘못된 자세 이름을 포함해 요청 오류를 돌려줍니다.
                raise ValueError(f"unknown named pose {request.name!r}")
            # 설정된 관절 순서에 맞춰 목표 위치를 실수 리스트로 만듭니다.
            positions = [float(target[name]) for name in self.joint_names]
            # 목표 위치가 여유값을 포함한 URDF/하드웨어 제한을 만족하는지 확인합니다.
            if not self.model.within_limits(positions, float(self.config["motion"]["ik_limit_margin_rad"])):
                # 제한을 위반한 이름 자세의 실행을 차단합니다.
                raise ValueError(f"named pose {request.name!r} violates hardware/URDF soft limits")
            # 설정된 관절 이름 순서를 사용하는 빈 관절 궤적 메시지를 생성합니다.
            trajectory = JointTrajectory(joint_names=list(self.joint_names))
            # 이름 자세를 하나의 목표점으로 만들고 설정된 지속 시간을 지정합니다.
            trajectory.points = [JointTrajectoryPoint(
                positions=positions, time_from_start=_duration(float(self.config["motion"]["named_pose_duration_s"]))) ]
            # 컨트롤러 실행 결과를 성공 응답에 기록합니다.
            response.success, response.message = True, self._send(trajectory, request.timeout_s)
        # 설정 조회, 검증 또는 액션 실행 중의 오류를 서비스 실패로 바꿉니다.
        except Exception as exc:
            # 실패 여부와 오류 원인을 서비스 응답에 기록합니다.
            response.success, response.message = False, str(exc)
            # 요청된 자세 이름과 오류 내용을 ROS 로그에 남깁니다.
            self.get_logger().error(f"move_to_named_pose({request.name}) failed: {exc}")
        # 성공 또는 실패가 기록된 응답 메시지를 반환합니다.
        return response

    # 외부에서 전달한 궤적을 검증한 뒤 컨트롤러에서 실행합니다.
    def _execute(self, request, response):
        # 궤적 실행 오류를 서비스 실패 응답으로 바꿉니다.
        try:
            # 공통 검증·액션 실행 함수의 성공 메시지를 응답에 넣습니다.
            response.success, response.message = True, self._send(request.trajectory, request.timeout_s)
        # 입력 궤적 또는 컨트롤러 실행에서 발생한 오류를 처리합니다.
        except Exception as exc:
            # 실패 상태와 원인 메시지를 서비스 응답에 기록합니다.
            response.success, response.message = False, str(exc)
            # 서비스 실행 실패의 상세 이유를 로그에 남깁니다.
            self.get_logger().error(f"execute_trajectory failed: {exc}")
        # 실행 결과가 채워진 서비스 응답을 반환합니다.
        return response

    # 현재 yaw를 기준으로 상대 회전 목표를 실행하고 피드백 기반 실제 변화를 반환합니다.
    def _rotate_yaw(self, request, response):
        # 요청값과 현재 상태, 제한, 액션 결과를 검증하는 구간입니다.
        try:
            # 상대 회전량은 유한한 숫자여야 합니다.
            if not math.isfinite(request.delta_rad):
                # NaN이나 무한대 회전 요청을 차단합니다.
                raise ValueError("delta_rad must be finite")
            # 현재 모든 관절 위치를 설정된 순서로 읽습니다.
            current = self._current()
            baseline_sequence = self.joint_state_sequence
            # 설정된 관절 목록에서 손목 yaw 관절이 위치한 인덱스를 구합니다.
            index = self.joint_names.index("wrist_yaw_joint")
            # 현재 yaw를 기준값으로 저장하고 요청한 상대량을 더해 목표를 계산합니다.
            baseline, target = current[index], current[index] + request.delta_rad
            # 기구학 모델이 제공하는 손목 yaw 관절의 허용 하한과 상한을 가져옵니다.
            lower, upper = self.model.limits["wrist_yaw_joint"]
            # 상대 목표가 하드웨어/URDF 관절 한계 안에 있는지 확인합니다.
            if not lower <= target <= upper:
                # 제한을 벗어나는 회전 요청을 컨트롤러에 보내지 않습니다.
                raise ValueError(f"relative yaw target {target:.6f} violates [{lower:.6f}, {upper:.6f}]")
            # 현재 위치 벡터에서 손목 yaw 성분만 상대 목표로 바꿉니다.
            current[index] = target
            # 전체 관절 목록을 유지하는 실행 궤적 메시지를 만듭니다.
            trajectory = JointTrajectory(joint_names=list(self.joint_names))
            # 변경된 yaw와 나머지 현재 위치를 단일 목표점으로 지정합니다.
            trajectory.points = [JointTrajectoryPoint(
                positions=current, time_from_start=_duration(float(self.config["motion"]["lock_duration_s"]))) ]
            # 목표를 컨트롤러에 실행하고 액션 성공 여부를 확인합니다.
            response.message = self._send(trajectory, request.timeout_s)
            # The result is intentionally feedback-derived, never an assumed delta.
            # 피드백 갱신을 기다릴 최대 시간을 요청 제한 시간의 0.5초 이하로 정합니다.
            deadline = time.monotonic() + min(0.5, request.timeout_s)
            # 기존 기준값과 같은 피드백이 유지되는 동안 제한 시간 안에서 기다립니다.
            while time.monotonic() < deadline and self.joint_state_sequence == baseline_sequence:
                # JointState 콜백이 피드백을 갱신할 수 있도록 짧게 대기합니다.
                time.sleep(0.01)
            # 최신 피드백과 명령 전 기준값의 차이를 실제 회전량으로 계산합니다.
            if self.joint_state_sequence == baseline_sequence:
                raise TimeoutError("no post-lock /joint_states snapshot received")
            actual = self._current()[index] - baseline
            # 회전 성공과 피드백에서 측정한 상대 회전량을 응답에 기록합니다.
            response.success, response.actual_delta_rad = True, actual
            # 기준 yaw와 실제 변화량을 진단 로그에 남깁니다.
            self.get_logger().info(f"wrist_yaw baseline={baseline:.6f}, actual_delta_rad={actual:.6f}")
        # 입력 검증 또는 액션 실행에 실패한 경우의 공통 응답 처리입니다.
        except Exception as exc:
            # 실패 상태, 중립적인 실제 변화량, 오류 설명을 응답에 기록합니다.
            response.success, response.actual_delta_rad, response.message = False, 0.0, str(exc)
            # 상대 회전 서비스의 실패 원인을 ROS 로그에 남깁니다.
            self.get_logger().error(f"rotate_wrist_yaw_relative failed: {exc}")
        # 성공 또는 실패 내용이 채워진 응답을 반환합니다.
        return response


# ROS 2를 초기화하고 모션 실행 노드를 구동하는 프로그램 진입점입니다.
def main():
    # ROS 클라이언트 라이브러리를 초기화합니다.
    rclpy.init()
    # 설정과 기구학 모델을 사용해 실행 노드를 생성합니다.
    try:
        node = MotionExecutor()
    # 필수 설정이나 모델 생성이 실패하면 ROS 실행을 시작하지 않습니다.
    except Exception as exc:
        # 설정 오류 원인을 노드 이름이 지정된 ROS 로그에 기록합니다.
        rclpy.logging.get_logger("motion_executor").error(f"configuration invalid; exiting: {exc}")
        # 노드 생성 실패 상태에서 ROS 자원을 정리합니다.
        rclpy.shutdown()
        # 실행 가능한 노드가 없으므로 진입점을 종료합니다.
        return
    # 동시 서비스와 JointState 콜백 처리를 위한 4개 스레드 실행기를 만듭니다.
    executor = MultiThreadedExecutor(num_threads=4)
    # 생성한 모션 실행 노드를 실행기에 등록합니다.
    executor.add_node(node)
    # 서비스 요청과 피드백 콜백을 계속 처리합니다.
    try:
        executor.spin()
    # 실행기 종료 시 콜백 처리와 노드, ROS 상태를 정리합니다.
    finally:
        # 실행기를 종료하고 진행 중인 콜백 작업을 정리합니다.
        executor.shutdown()
        # 노드가 생성한 ROS 통신 자원을 해제합니다.
        node.destroy_node()
        # ROS 클라이언트 라이브러리를 종료합니다.
        rclpy.shutdown()


# 이 파일을 직접 실행할 때만 main을 호출하고 모듈 import 시에는 실행하지 않습니다.
if __name__ == "__main__":
    # ROS 노드 초기화와 실행 루프를 시작합니다.
    main()
