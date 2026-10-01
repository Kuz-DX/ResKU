#!/usr/bin/env python3
# 이 파일을 직접 실행할 때 Python 3 인터프리터를 사용하도록 지정합니다.
"""One-shot, hold-on-failure FSM for the tool-1 Cartesian target routine.

The state order is HOME, DOCKING_WAIT, TOOL1_PRE, TOOL1_TARGET, LOCK,
RETURN_DOCKING_WAIT, RETURN_HOME, DONE.  TOOL1_TARGET reads a fixed
``base_actuator`` XYZ+quaternion from YAML and asks IK for a Cartesian path.
Position waypoint k is ``p0 + (k/N)(pt-p0)`` and orientation follows the
shortest SO(3) rotation; N is the larger position/rotation step count. Every
waypoint must solve within the joint limits before the resulting multi-point
JointTrajectory is sent to the controller.
LOCK then snapshots the actual wrist yaw and commands ``baseline + pi/2`` in
the positive (CCW) ROS joint direction; it is a relative action, not a pose.
Every service/controller failure or timeout enters HOLD without retry or any
speculative return motion.
"""
# 이 모듈은 도구 1 장착 절차를 한 번 실행하고 실패 시 HOLD에 머뭅니다.
# 목표 자세는 base_actuator 기준 XYZ와 quaternion을 모두 IK에 전달합니다.
# 실패 이후 자동 재시도나 보상 이동은 수행하지 않습니다.
# 타입 어노테이션을 지연 평가해 전방 참조와 타입 선언을 지원합니다.
from __future__ import annotations

# 서비스 대기 루프에서 짧은 간격으로 실행을 양보하고 제한 시간을 측정합니다.
import time

# ROS 2 Python 클라이언트를 초기화하고 종료하는 기능을 제공합니다.
import rclpy
# 패키지가 설치된 공유 디렉터리의 기본 설정 경로를 찾습니다.
from ament_index_python.packages import get_package_share_directory
# 여러 스레드에서 ROS 콜백을 처리하는 실행기를 사용합니다.
from rclpy.executors import MultiThreadedExecutor
# ROS 2 노드 기본 기능을 상속하기 위해 가져옵니다.
from rclpy.node import Node
# 요청 번호와 상태 문자열을 토픽으로 주고받는 메시지 형식입니다.
from std_msgs.msg import Int32, String

# 정지 상태 설정 검증과 개발 단계별 정지 지점을 처리합니다.
from tool_change_min.config import require_for_stop_state, stop_after_state
# 이 FSM이 호출하는 자세 이동, IK, 궤적 실행 서비스 형식입니다.
from tool_change_min.srv import ExecuteTrajectory, LinearMoveToPose, MoveNamedPose, RotateWristYaw


# 도구 장착 절차의 상태와 서비스 호출을 관리하는 ROS 2 노드입니다.
class ToolChangeFsm(Node):
    # 노드, 설정, 상태 토픽 및 하위 서비스 클라이언트를 초기화합니다.
    def __init__(self):
        # ROS 그래프에 등록될 노드 이름을 설정합니다.
        super().__init__("tool_change_fsm")
        # 이 패키지의 설치된 공유 디렉터리를 찾아 기본 설정 경로에 사용합니다.
        share = get_package_share_directory("tool_change_min")
        # 자세 및 동작 설정 YAML 경로를 ROS 파라미터로 선언합니다.
        self.declare_parameter("poses_yaml", f"{share}/config/poses.yaml")
        # 정지 자세를 포함한 필수 설정을 읽고 검증합니다.
        self.config = require_for_stop_state(self.get_parameter("poses_yaml").value)
        # 설정된 개발용 정지 단계 이름을 가져옵니다.
        self.stop_after = stop_after_state(self.config)
        # 시작 상태는 어떤 요청도 처리 중이지 않은 IDLE입니다.
        self.state = "IDLE"
        # 외부 관찰자가 현재 절차 상태를 구독할 수 있도록 퍼블리셔를 만듭니다.
        self.status = self.create_publisher(String, "/tool_change/status", 10)
        # 도구 변경 요청 토픽을 구독하고 수신 시 _request 콜백을 호출합니다.
        self.create_subscription(Int32, "/tool_change/request", self._request, 10)
        # 미리 지정한 관절 자세로 이동하는 서비스 클라이언트를 만듭니다.
        self.named = self.create_client(MoveNamedPose, "move_to_named_pose")
        # 고정 Cartesian pose까지 다점 직선 IK 경로를 만드는 서비스 클라이언트입니다.
        self.linear_pose = self.create_client(LinearMoveToPose, "linear_move_to_pose")
        # 계획된 관절 궤적을 컨트롤러에 실행시키는 서비스 클라이언트를 만듭니다.
        self.execute = self.create_client(ExecuteTrajectory, "execute_trajectory")
        # 기준 관절 상태에서 손목 요 회전을 요청하는 서비스 클라이언트를 만듭니다.
        self.rotate = self.create_client(RotateWristYaw, "rotate_wrist_yaw_relative")
        # 초기 상태를 토픽과 로그에 알립니다.
        self._publish("IDLE")

    # 내부 상태를 갱신하고 토픽과 로그에 같은 상태를 알립니다.
    def _publish(self, state: str):
        # 현재 상태를 저장해 요청 처리와 실패 로그에서 사용할 수 있게 합니다.
        self.state = state
        # 상태 문자열을 ROS 메시지로 만들어 상태 토픽에 발행합니다.
        self.status.publish(String(data=state))
        # ROS 로그에도 상태 전이를 남깁니다.
        self.get_logger().info(f"tool-change state: {state}")

    # 서비스를 기다리고 호출한 뒤 결과를 확인하며, 제한 시간 초과 시 예외를 발생시킵니다.
    def _call(self, client, request, timeout_s: float):
        # 0 이하의 제한 시간은 유효한 대기가 아니므로 즉시 거부합니다.
        if timeout_s <= 0.0:
            # 호출한 단계가 실패 처리 경로로 이동하도록 입력 오류를 발생시킵니다.
            raise ValueError("state timeout must be > 0")
        # 지정 시간 안에 해당 서비스 서버가 준비되는지 확인합니다.
        if not client.wait_for_service(timeout_sec=timeout_s):
            # 서비스가 없으면 실행 요청을 보내지 않고 실패로 처리합니다.
            raise TimeoutError("required service unavailable")
        # 비동기 서비스 요청을 보내고 완료 판정에 사용할 절대 마감 시각을 계산합니다.
        future, deadline = client.call_async(request), time.monotonic() + timeout_s
        # 응답이 오거나 마감 시각에 도달할 때까지 짧은 간격으로 확인합니다.
        while not future.done() and time.monotonic() < deadline:
            # 바쁜 대기 대신 10ms 동안 쉬어 CPU 사용량을 낮춥니다.
            time.sleep(0.01)
        # 마감까지 응답이 완료되지 않았다면 시간 초과로 취급합니다.
        if not future.done():
            # 상위 단계의 예외 처리에서 전체 FSM을 HOLD로 전환하게 합니다.
            raise TimeoutError("service call timed out")
        # 완료된 Future에서 서비스 응답 객체를 가져옵니다.
        response = future.result()
        # 서비스가 명시적으로 실패를 반환했으면 응답 메시지를 예외 사유로 사용합니다.
        if not response.success:
            # 거부 원인을 유지해 상위 로깅과 HOLD 상태에 전달합니다.
            raise RuntimeError(response.message)
        # 성공한 서비스 응답을 호출자에게 돌려줍니다.
        return response

    # 설정된 이름 자세로 이동하는 단계를 실행합니다.
    def _named(self, state: str, pose: str, timeout_key: str):
        # 서비스 요청 전에 현재 FSM 단계를 상태 토픽에 알립니다.
        self._publish(state)
        # 이름 자세 이동 서비스의 요청 메시지를 만듭니다.
        request = MoveNamedPose.Request()
        # 이동할 사전 정의 자세의 이름을 지정합니다.
        request.name = pose
        # 이 단계에 지정된 제한 시간을 설정 파일에서 읽어 요청에 넣습니다.
        request.timeout_s = float(self.config["timeouts_s"][timeout_key])
        # 서비스 완료 및 성공 여부를 제한 시간과 함께 확인합니다.
        self._call(self.named, request, request.timeout_s)

    # 고정 base-frame Cartesian pose까지 직선 다점 IK 경로를 생성하고 실행합니다.
    def _cartesian_target(self, state: str, target_name: str, timeout_key: str):
        self._publish(state)
        timeout = float(self.config["timeouts_s"][timeout_key])
        target = self.config["cartesian_targets"][target_name]
        request = LinearMoveToPose.Request()
        request.target_pose.header.frame_id = str(target["frame_id"])
        request.target_pose.pose.position.x, request.target_pose.pose.position.y, request.target_pose.pose.position.z = (
            [float(value) for value in target["position_m"]])
        (request.target_pose.pose.orientation.x, request.target_pose.pose.orientation.y,
         request.target_pose.pose.orientation.z, request.target_pose.pose.orientation.w) = (
             [float(value) for value in target["orientation_xyzw"]])
        request.speed_m_s = float(self.config["motion"]["insert"]["speed_m_s"])
        solved = self._call(self.linear_pose, request, timeout)
        execute = ExecuteTrajectory.Request()
        execute.trajectory, execute.timeout_s = solved.trajectory, timeout
        self._call(self.execute, execute, timeout)

    # 설정에서 지정한 단계에 도달했는지 확인하고 개발용 일시 정지를 수행합니다.
    def _pause_after(self, stage: str) -> bool:
        # 현재 단계가 설정된 정지 지점이 아니면 계속 진행할 수 있도록 False를 돌려줍니다.
        if self.stop_after != stage:
            return False
        # 정지 지점에 도달했음을 PAUSED_ 접두 상태로 외부에 알립니다.
        self._publish(f"PAUSED_{stage.upper()}")
        # 검토 후 다음 상태를 설정하도록 안내하는 로그를 출력합니다.
        self.get_logger().info(
            f"Development stop reached after {stage}; set "
            "development.stop_after_state to the next state after review.")
        # 호출자가 현재 절차를 중단하도록 True를 반환합니다.
        return True

    # 도구 장착 절차의 모든 단계를 정해진 순서로 실행합니다.
    def _run(self):
        # 어느 단계에서든 실패하면 아래 공통 예외 처리로 이동합니다.
        try:
            # 로봇을 기준 HOME 자세로 이동합니다.
            self._named("HOME", "home", "home")
            # HOME 직후 개발 정지 지점이면 이후 단계를 실행하지 않습니다.
            if self._pause_after("home"):
                return
            # 도킹 대기 자세로 이동합니다.
            self._named("DOCKING_WAIT", "docking_wait", "docking_wait")
            # 도킹 대기 단계가 정지 지점이면 절차를 반환합니다.
            if self._pause_after("docking_wait"):
                return
            # tool 1 목표 pose를 풀기 전의 teach pre-pose로 이동합니다.
            self._named("TOOL1_PRE", "tool1_pre", "tool1_pre")
            if self._pause_after("tool1_pre"):
                return
            # 미리 정한 base_actuator Cartesian target pose를 IK로 풀어 실행합니다.
            self._cartesian_target("TOOL1_TARGET", "tool1_target", "tool1_target")
            if self._pause_after("tool1_target"):
                return
            # 목표 도달 후 실제 wrist-yaw 피드백을 baseline으로 CCW +90도 체결합니다.
            self._publish("LOCK")
            lock = RotateWristYaw.Request()
            lock.delta_rad = float(self.config["motion"]["yaw_delta_rad"])
            lock.timeout_s = float(self.config["timeouts_s"]["lock"])
            self._call(self.rotate, lock, lock.timeout_s)
            if self._pause_after("lock"):
                return
            self._named("RETURN_DOCKING_WAIT", "docking_wait", "return_docking_wait")
            if self._pause_after("return_docking_wait"):
                return
            self._named("RETURN_HOME", "home", "return_home")
            if self._pause_after("return_home"):
                return
            # 모든 단계가 성공적으로 끝났음을 완료 상태로 알립니다.
            self._publish("DONE")
        # 어떤 서비스·단계에서 예외가 발생해도 즉시 공통 실패 처리로 모읍니다.
        except Exception as exc:
            # Do not issue any compensating motion after a failed stage.
            # 실패한 단계와 이유를 기록하며, 추가 이동 없이 HOLD 상태로 전환합니다.
            self.get_logger().error(f"tool change failed in {self.state}: {exc}; entering HOLD")
            # 실패 이후 자동 복구를 시도하지 않고 현재 절차를 보류 상태로 둡니다.
            self._publish("HOLD")

    # 도구 변경 요청을 검증하고 허용된 경우 장착 절차를 시작합니다.
    def _request(self, message: Int32):
        # 현재 구현은 도구 1 장착 요청 번호만 지원합니다.
        if message.data != 1:
            # 지원하지 않는 번호를 로그로 남기고 요청을 무시합니다.
            self.get_logger().warning(f"ignored request {message.data}; only tool 1 attach is supported")
            return
        # 실행 중인 단계가 있으면 중복 요청으로 간주해 새 절차를 시작하지 않습니다.
        if self.state not in ("IDLE", "DONE", "HOLD"):
            # 무시한 요청과 현재 바쁜 상태를 로그에 남깁니다.
            self.get_logger().warning(f"ignored request while busy in {self.state}")
            return
        # IDLE, DONE 또는 HOLD 상태에서는 전체 절차를 새로 실행합니다.
        self._run()


# ROS 2를 초기화하고 FSM 노드를 실행하는 프로그램 진입점입니다.
def main():
    # ROS 클라이언트 라이브러리를 초기화합니다.
    rclpy.init()
    # 설정을 읽고 FSM 노드를 생성합니다.
    try:
        node = ToolChangeFsm()
    # 노드 구성이나 설정 검증이 실패하면 실행기를 만들지 않습니다.
    except Exception as exc:
        # 설정 오류를 로그로 남겨 실행이 중단된 이유를 알립니다.
        rclpy.logging.get_logger("tool_change_fsm").error(f"configuration invalid; exiting: {exc}")
        # 노드 생성 실패 시에도 ROS 초기화 자원을 정리합니다.
        rclpy.shutdown()
        # 유효한 노드가 없으므로 프로그램 진입점을 종료합니다.
        return
    # 동시에 여러 ROS 콜백을 처리할 4개 스레드 실행기를 만듭니다.
    executor = MultiThreadedExecutor(num_threads=4)
    # 생성한 FSM 노드를 실행기에 등록합니다.
    executor.add_node(node)
    # ROS 콜백이 들어오는 동안 실행기를 계속 구동합니다.
    try:
        executor.spin()
    # 실행기가 종료될 때 노드와 ROS 자원을 순서대로 정리합니다.
    finally:
        # 실행 중인 콜백 처리를 끝내고 실행기를 종료합니다.
        executor.shutdown()
        # 노드가 생성한 퍼블리셔·구독·클라이언트 등 자원을 해제합니다.
        node.destroy_node()
        # ROS 클라이언트 라이브러리를 종료합니다.
        rclpy.shutdown()


# 이 파일을 모듈로 가져올 때는 자동 실행하지 않고 직접 실행할 때만 main을 호출합니다.
if __name__ == "__main__":
    # 노드 생성과 ROS 이벤트 처리를 시작합니다.
    main()
