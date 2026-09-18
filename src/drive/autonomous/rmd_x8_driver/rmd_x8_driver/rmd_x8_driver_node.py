#!/usr/bin/env python3
"""
rmd_x8_driver_node

Drives two RMD-X8-120 actuators (left/right) of a skid-steer /
tracked vehicle over CAN, using MYACTUATOR's RMD-X protocol
(Speed Closed-loop Control Command, 0xA2).

[manual+return 통합] 이 노드가 CAN 버스(can_drive)/motor ID를 소유하는
유일한 저수준 드라이버다 -- manual 조종(drive_cmd_mux_node를 거쳐
/cmd_vel_manual -> /cmd_vel)과 return 자동복귀(/cmd_vel_return -> /cmd_vel)
모두 이 노드를 공용으로 사용한다. 예전 can_driver_node(manual 전용, CAN
feedback을 안 읽어 wheel odometry가 없었음)는 더 이상 launch되지 않는다 --
같은 CAN 버스/motor ID를 두 노드가 동시에 제어해서는 안 된다.

Subscribes:
    /cmd_vel               (geometry_msgs/Twist)   -- desired body v, w
                                                        (from drive_cmd_mux_node)

Publishes:
    /wheel/odom             (nav_msgs/Odometry)      -- vx only is meaningful;
                                                         see robot_localization
                                                         odom0_config (this
                                                         project fuses vx only,
                                                         wheel yaw rate is
                                                         intentionally NOT
                                                         trusted -- see the
                                                         control guide, 4.1)
    /wheel/joint_states      (sensor_msgs/JointState) -- position(rad),
                                                         velocity(rad/s),
                                                         effort(A, torque
                                                         current -- reused
                                                         field, see README)
    /wheel/motor_status      (diagnostic_msgs/DiagnosticArray)
                                                       -- temp / voltage /
                                                          error flags, for
                                                          the current-monitor
                                                          / stability layers
                                                          described in the
                                                          control guide (3.3)

IMPORTANT SAFETY NOTE:
    The RMD-X drive itself has a 500 ms heartbeat timeout: if it does not
    receive a new control frame within 500 ms it stops the motor on its
    own. This node ALSO enforces a shorter, configurable cmd_vel timeout
    so the robot decelerates smoothly well before that hardware cutoff
    (see control_guide.md section 2.3 "never hard-stop on a slope").
"""

import math
import struct
import threading
import time

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# CMakeLists.txt가 이 파일을 install(PROGRAMS ...)로 그대로 복사해 실행 파일로
# 등록하기 때문에 ros2 run이 이걸 __main__으로 직접 실행한다 (패키지로 import되지
# 않음) -> 상대 임포트(from . import ...)는 __package__가 없어 항상 실패한다.
# ament_python_install_package(rmd_x8_driver)로 패키지 자체는 정상 설치되어
# PYTHONPATH에 잡히므로 절대 임포트를 사용한다.
from rmd_x8_driver import rmd_x8_protocol as proto


class MotorChannel:
    """Per-motor runtime state (one instance for left, one for right)."""

    def __init__(self, name: str, can_id: int, direction_sign: float,
                 wheel_radius_m: float, external_gear_ratio: float):
        self.name = name
        self.can_id = can_id
        self.direction_sign = direction_sign
        self.wheel_radius_m = wheel_radius_m
        self.external_gear_ratio = external_gear_ratio

        self.unwrapper = proto.AngleUnwrapper()
        self.lock = threading.Lock()

        # latest feedback (updated from CAN RX thread)
        self.last_feedback_time = None
        self.wheel_angle_rad = 0.0      # unwrapped, actual wheel frame
        self.wheel_speed_rad_s = 0.0    # actual wheel frame
        self.torque_current_a = 0.0
        self.temperature_c = None
        self.status1 = None            # proto.MotorStatus1 or None
        # None = not confirmed yet (readback reply never arrived). Otherwise
        # the timeout (ms) this motor actually echoed back applying --
        # see RmdX8DriverNode._configure_comm_timeout_protection().
        self.comm_timeout_confirmed_ms = None

    def apply_feedback(self, fb: proto.MotorFeedback):
        with self.lock:
            # Manual values are OUTPUT SHAFT values (internal gearbox
            # already applied). external_gear_ratio only accounts for
            # any *additional* gearing between the actuator's output
            # shaft and the actual wheel/sprocket (1.0 if directly
            # coupled).
            unwrapped_output_deg = self.unwrapper.update(fb.angle_deg)
            wheel_deg = (unwrapped_output_deg / self.external_gear_ratio) * self.direction_sign
            wheel_dps = (fb.speed_dps / self.external_gear_ratio) * self.direction_sign

            self.wheel_angle_rad = math.radians(wheel_deg)
            self.wheel_speed_rad_s = math.radians(wheel_dps)
            self.torque_current_a = fb.torque_current_a * self.direction_sign
            self.temperature_c = fb.temperature_c
            self.last_feedback_time = time.monotonic()

    def apply_status1(self, st: proto.MotorStatus1):
        with self.lock:
            self.status1 = st

    def apply_comm_timeout_ack(self, timeout_ms: int):
        with self.lock:
            self.comm_timeout_confirmed_ms = timeout_ms

    def wheel_linear_speed_m_s(self) -> float:
        with self.lock:
            return self.wheel_speed_rad_s * self.wheel_radius_m

    def snapshot(self):
        with self.lock:
            never_received = self.last_feedback_time is None
            stale_after_having_feedback = (
                not never_received and
                (time.monotonic() - self.last_feedback_time) > 0.5)
            return {
                "angle_rad": self.wheel_angle_rad,
                "speed_rad_s": self.wheel_speed_rad_s,
                "current_a": self.torque_current_a,
                "temperature_c": self.temperature_c,
                "status1": self.status1,
                "comm_timeout_confirmed_ms": self.comm_timeout_confirmed_ms,
                # 안전판단(속도 0 보고/odom 적분 중단 등)용 -- "한 번도 못
                # 받음"과 "받다가 0.5초 넘게 끊김" 둘 다 "신뢰 못 함"으로
                # 묶어서 취급 (기존 동작 그대로 유지).
                "stale": never_received or stale_after_having_feedback,
                # [수정] 바깥(stability_monitor_node의 fault 래치)에 보고할
                # DiagnosticStatus.STALE 판정 전용 -- "한 번도 못 받음"(노드가
                # 막 켜져서 아직 첫 CAN 응답을 못 받은, 누구나 거치는 정상
                # 부팅 구간)은 여기서 제외한다. 안 그러면 위의 "stale"을 그대로
                # STALE로 보고할 때, 노드가 켜질 때마다 무조건 최초 한 번은
                # STALE을 찍고 지나가게 되고, stability_monitor_node의 fault
                # 래치가 그걸 그대로 "통신 두절"로 오인해서 매번 부팅 시점에
                # 영구히 걸려버리는 문제가 실기에서 확인됐다(2026-08-20,
                # autonomous.launch.py를 깨끗하게 재시작했는데도 /cmd_vel_safety가
                # 계속 0으로 나옴). 진짜 "받다가 끊긴" 경우만 STALE로 보고한다.
                "stale_after_having_feedback": stale_after_having_feedback,
            }


class RmdX8DriverNode(Node):
    def __init__(self):
        super().__init__("rmd_x8_driver_node")

        # ---- parameters -------------------------------------------------
        # [하림 수정] 기본값을 can_drive로 변경 (can_driver 패키지와 동일 운영 관례)
        self.declare_parameter("can_interface", "can_drive")
        self.declare_parameter("left_motor_can_id", 1)
        self.declare_parameter("right_motor_can_id", 2)
        self.declare_parameter("left_direction_sign", 1.0)
        self.declare_parameter("right_direction_sign", -1.0)  # mirrored mounting, verify on bench!
        self.declare_parameter("wheel_radius_m", 0.1125)
        self.declare_parameter("external_gear_ratio", 1.0)
        self.declare_parameter("effective_track_width_m", 0.50)  # MUST calibrate, see control_guide 3.1
        # [2026-08-27 실측, 재검증] 제자리 360도 회전(VALIDATION.md Test 5),
        # 좌/우 각각 60초, effective_track_width_m=0.365 정확히 반영된 상태로
        # 측정(최초 측정은 이 값이 실수로 0.4904인 채로 잡혀서 track_width
        # 기구학 오차와 진짜 슬립이 뒤섞여 있었음 -- wheel/odom의 wz 자체가
        # effective_track_width_m으로 계산되기 때문, 아래 참고). 우회전
        # 진짜 슬립률 7.9%, 좌회전 7.59% -> 평균 7.75%. 이미 걸려있던 예전
        # 계수(1.069)를 감안해 역산: 필요 계수 = 1/(1-0.0775) ≈ 1.084.
        # 이 값을 명령 w에 곱해 미리 더 세게 돌라고 명령해서 슬립으로 깎이는
        # 만큼을 feedforward로 보정한다 -- _skid_steer_inverse() 참고.
        # [2026-08-27 속도 의존성 재검증] 위 "고속에서 30%까지 커질 수 있다"는
        # 우려는 오염된 최초 측정을 역산한 값이라 근거가 약했음 -- w=0.267/
        # 0.5/0.7/0.9rad/s(0.9 = our_mppi_params.yaml wz_max, 즉 MPPI가 실제로
        # 낼 수 있는 최대치) 네 지점에서 깨끗하게(0.365 반영된 상태로) 다시
        # 재보니 슬립률이 6.36%~7.9%로 이 전 구간에서 사실상 상수였음(속도가
        # 가장 빠른 0.9rad/s에서 오히려 제일 낮은 6.36%). 30% 가설은 기각.
        #
        # [2026-08-27 실차 재현 테스트에서 1.084 잠정 보류] 제자리 회전으로
        # 검증한 1.084를 실제 MPPI 반원 경로 주행(전진+회전 동시)에 적용해
        # 보니 CTE가 오히려 악화됐다(0.72m -> 1.16m -> 1.42m, 재현할수록
        # 더 나빠짐). 회전 초반 안정 구간(cmd_wz가 wz_max=0.9rad/s로 포화된
        # 구간)만 보면 슬립 4.4%로 제자리 회전 값과 비슷했지만, 그 이후 MPPI가
        # 낸 cmd_wz 자체가 부호까지 뒤집히며 요동치는 발산성 패턴이 나타남 --
        # 이 open-loop feedforward 보정이 MPPI의 자체 closed-loop 재계획과
        # 상호작용해 모델 불일치를 키웠을 가능성이 있음(MPPI의 내부 DiffDrive
        # 모델은 이 보정 배율의 존재를 모른 채 실제 오도메트리로만 재계획함).
        # 원인 격리를 위해 track_width(365mm)만 남기고 이 계수는 1.0(무보정)
        # 으로 되돌려 재테스트 예정 -- 결과 나오기 전까지 1.084를 프로덕션에
        # 쓰지 말 것.
        self.declare_parameter("angular_slip_compensation_factor", 1.0)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.3)
        self.declare_parameter("cmd_vel_safety_timeout_s", 0.5)
        self.declare_parameter("max_wheel_speed_dps", 3000.0)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_odom_tf", False)  # normally False: robot_localization publishes odom->base_link
        # [수정] 이 노드가 (크래시/kill -9 포함 어떤 이유로든) 새 명령을 더
        # 이상 못 보내는 상태가 되면, 모터가 하드웨어 레벨에서 스스로 멈추게
        # 하는 마지막 방어선. 0xA2 등 새 제어 프레임이 이 시간(ms) 안에 안
        # 오면 모터가 알아서 정지한다 -- 지금까지 코드 주석에 "RMD-X8은 500ms
        # 하트비트로 알아서 멈춘다"고 적혀 있었지만 실제로 이 설정을 모터에
        # 걸어준 적이 한 번도 없었다(이 프로토콜 모듈에 0xB3 커맨드 자체가
        # 없었음, arm 쪽 rmd_sdk에는 있는데 여기만 빠짐) -- 노드가 죽었는데도
        # 로봇이 계속 움직인 사고의 유력한 원인. 0으로 주면 이 기능 자체가
        # 꺼지니 절대 0으로 두지 말 것.
        self.declare_parameter("comm_timeout_protection_ms", 500)
        # [manual+return 통합] can_driver_node에서 이관 -- USB-CAN 어댑터(gs_usb)가
        # 물리적으로 잠깐 빠졌다 붙는 사고가 manual 조종 중 실기에서 확인된 적이
        # 있다(can_driver_node._attempt_reconnect() 주석 참고). 이 노드가 이제
        # manual+return 공용 CAN 소유 노드가 되므로 동일한 재연결 로직이 필요하다.
        self.declare_parameter("reconnect_after_failures", 10)
        self.declare_parameter("reconnect_cooldown_sec", 2.0)

        can_interface = self.get_parameter("can_interface").value
        left_id = int(self.get_parameter("left_motor_can_id").value)
        right_id = int(self.get_parameter("right_motor_can_id").value)
        left_dir = float(self.get_parameter("left_direction_sign").value)
        right_dir = float(self.get_parameter("right_direction_sign").value)
        wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        gear_ratio = float(self.get_parameter("external_gear_ratio").value)

        self.track_width = float(self.get_parameter("effective_track_width_m").value)
        self.angular_slip_compensation_factor = float(
            self.get_parameter("angular_slip_compensation_factor").value)
        self.control_period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout_s").value)
        self.cmd_vel_safety_timeout = float(self.get_parameter("cmd_vel_safety_timeout_s").value)
        self.max_wheel_speed_dps = float(self.get_parameter("max_wheel_speed_dps").value)
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.publish_odom_tf = bool(self.get_parameter("publish_odom_tf").value)
        self.comm_timeout_protection_ms = int(self.get_parameter("comm_timeout_protection_ms").value)
        self.reconnect_after_failures = int(self.get_parameter("reconnect_after_failures").value)
        self.reconnect_cooldown_sec = float(self.get_parameter("reconnect_cooldown_sec").value)

        self.left = MotorChannel("left", left_id, left_dir, wheel_radius, gear_ratio)
        self.right = MotorChannel("right", right_id, right_dir, wheel_radius, gear_ratio)
        self.channels = [self.left, self.right]

        self._can_interface = can_interface
        self._reply_id_map = {
            proto.motor_reply_id(left_id): self.left,
            proto.motor_reply_id(right_id): self.right,
        }
        self._consecutive_send_failures = 0
        self._last_reconnect_attempt = None
        self.bus = None
        self._notifier = None

        # ---- CAN bus ------------------------------------------------------
        # [manual+return 통합] can_driver_node._open_bus()와 동일하게, 최초
        # 연결 실패에도 노드를 살려둔다 -- 이 노드는 이제 manual 조종의 유일한
        # 저수준 드라이버이기도 하므로, 어댑터가 아직 안 붙은 채로 launch돼도
        # 프로세스 자체는 죽지 않고 재연결을 계속 시도해야 한다.
        if not self._open_bus():
            self.get_logger().error(
                f"CAN interface '{can_interface}' not available at startup -- "
                f"node stays alive and will keep retrying (see _attempt_reconnect())."
            )

        # [수정] 모터 하드웨어 통신두절 보호(0xB3)를 시작 시 실제로 걸어준다
        # (지금까지 아무도 이걸 호출한 적이 없었음). Notifier가 이미 떠 있으니
        # 여기서 보낸 명령의 reply도 _on_can_message가 정상적으로 받아 처리한다.
        if self.bus is not None:
            self._configure_comm_timeout_protection()

        # ---- ROS interfaces -------------------------------------------------
        best_effort_qos = QoSProfile(depth=10,
                                      reliability=ReliabilityPolicy.BEST_EFFORT,
                                      history=HistoryPolicy.KEEP_LAST)

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # [하림 수정] stability_monitor_node의 CRITICAL 비상 명령을
        # current_ramp_node를 거치지 않고 여기서 직접 받아
        # 최우선 처리한다 (그 체인이 죽어도 안전 개입은 살아있도록, 실제 모터
        # 명령 생성 지점에 가장 가깝게 배치).
        # [manual+return 통합] 이 노드는 이제 manual 조종(경유: drive_cmd_mux_node)과
        # return 자동복귀 모두의 공용 저수준 CAN 드라이버다 -- can_driver_node는
        # CAN feedback을 못 읽어 manual 주행 중 wheel odometry가 아예 안 나오는
        # 구조적 한계 때문에 이 경로에서 제외됐다(더 이상 launch되지 않음, 소스는
        # 보존). manual_stability_node(IMU pitch/roll)도 이제 여기 /cmd_vel_safety로
        # 직접 발행한다.
        self.cmd_vel_safety_sub = self.create_subscription(
            Twist, "/cmd_vel_safety", self._on_cmd_vel_safety, 10)

        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", best_effort_qos)
        self.joint_state_pub = self.create_publisher(JointState, "/wheel/joint_states", best_effort_qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/wheel/motor_status", 10)

        self._tf_broadcaster = None
        if self.publish_odom_tf:
            from tf2_ros import TransformBroadcaster
            self._tf_broadcaster = TransformBroadcaster(self)

        # ---- runtime state --------------------------------------------------
        self._desired_v = 0.0
        self._desired_w = 0.0
        self._last_cmd_vel_time = self.get_clock().now()
        self._safety_v = 0.0
        self._safety_w = 0.0
        self._last_safety_time = None  # None = /cmd_vel_safety로부터 아직 아무 메시지도 못 받음
        self._cmd_lock = threading.Lock()

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._last_odom_time = self.get_clock().now()

        self._status1_poll_counter = 0

        self.control_timer = self.create_timer(self.control_period, self._control_loop)

        self.get_logger().info(
            f"rmd_x8_driver_node started: can={can_interface}, "
            f"left_id={left_id}, right_id={right_id}, "
            f"track_width={self.track_width} m (VERIFY VIA CALIBRATION), "
            f"wheel_radius={wheel_radius} m"
        )

    # ------------------------------------------------------------------ #
    # CAN bus (re)connection -- ported from can_driver_node._open_bus() /
    # _attempt_reconnect(), extended to also (re)bind the CAN RX Notifier
    # and re-arm the comm_timeout_protection watchdog, since this node
    # (unlike can_driver_node) reads feedback and configures the motors'
    # own hardware watchdog on top of just sending commands.
    # ------------------------------------------------------------------ #
    def _open_bus(self) -> bool:
        try:
            self.bus = can.interface.Bus(channel=self._can_interface, bustype="socketcan")
        except Exception as exc:  # noqa: BLE001
            self.bus = None
            self.get_logger().error(
                f"Failed to open CAN interface '{self._can_interface}': {exc} -- "
                f"node stays alive, will keep retrying "
                f"(`ip link show {self._can_interface}` to check interface state)."
            )
            return False

        if self._notifier is not None:
            try:
                self._notifier.stop()
            except Exception:  # noqa: BLE001
                pass
        self._notifier = can.Notifier(self.bus, [self._on_can_message])
        return True

    def _attempt_reconnect(self):
        """CAN 소켓을 다시 연다 -- self.bus가 없는 경우(_send_speed_command의
        가드)와 연속 전송 실패가 임계치를 넘은 경우 둘 다 여기로 온다. USB-CAN
        어댑터 재열거로 인터페이스 자체가 바뀌는 상황을 노린 재시도이며, 재시도
        폭주를 막기 위해 reconnect_cooldown_sec 간격을 둔다 (can_driver_node와
        동일 패턴)."""
        now = self.get_clock().now()
        if self._last_reconnect_attempt is not None:
            age_s = (now - self._last_reconnect_attempt).nanoseconds * 1e-9
            if age_s < self.reconnect_cooldown_sec:
                return
        self._last_reconnect_attempt = now

        self.get_logger().warn(
            f"CAN bus unusable ({self._consecutive_send_failures} consecutive send "
            f"failures) -- attempting to (re)open {self._can_interface}"
        )
        if self.bus is not None:
            try:
                self._notifier.stop()
            except Exception:  # noqa: BLE001
                pass
            try:
                self.bus.shutdown()
            except Exception:  # noqa: BLE001
                pass
            self.bus = None
            self._notifier = None

        # 재연결 성공 시, 재부팅된 모터 쪽 통신두절 보호(0xB3)도 다시 걸어준다 --
        # 이 설정은 모터 자체 상태라 소켓만 다시 여는 것과 무관하게 유지되지만,
        # 그 사이 모터 전원이 같이 나갔다 들어온 경우까지 커버하기 위해 매
        # 재연결마다 재적용한다.
        if self._open_bus():
            self._consecutive_send_failures = 0
            self._configure_comm_timeout_protection()

    # ------------------------------------------------------------------ #
    # CAN RX
    # ------------------------------------------------------------------ #
    def _on_can_message(self, msg: can.Message):
        channel = self._reply_id_map.get(msg.arbitration_id)
        if channel is None:
            return
        if len(msg.data) != 8:
            return

        cmd = msg.data[0]
        try:
            if cmd in (proto.CMD_SPEED_CONTROL, proto.CMD_READ_STATUS_2):
                fb = proto.parse_a2_9c_reply(bytes(msg.data))
                channel.apply_feedback(fb)
            elif cmd == proto.CMD_READ_STATUS_1:
                st = proto.parse_status1_reply(bytes(msg.data))
                channel.apply_status1(st)
                if st.error_flags:
                    self.get_logger().warn(
                        f"[{channel.name}] motor error flags: {st.error_flags}"
                    )
            elif cmd == proto.CMD_SET_TIMEOUT:
                applied_ms = proto.parse_set_timeout_reply(bytes(msg.data))
                channel.apply_comm_timeout_ack(applied_ms)
        except (struct.error, ValueError) as exc:
            self.get_logger().warn(f"[{channel.name}] failed to parse CAN reply: {exc}")

    # ------------------------------------------------------------------ #
    # startup safety configuration
    # ------------------------------------------------------------------ #
    def _configure_comm_timeout_protection(self):
        """0xB3로 통신두절 보호시간을 설정하고, readback(reply echo)으로 모터가
        실제로 그 값을 적용했는지 검증한다. 이 노드가 죽거나(kill -9 포함)
        CAN 자체가 끊겨서 더 이상 명령을 못 보내는 상황에서, 모터가 하드웨어
        레벨에서 스스로 멈추게 하는 유일한 방어선 -- destroy_node()의 정지
        프레임 전송은 정상 종료 경로에서만 동작하므로 이것만으론 부족하다."""
        timeout_ms = self.comm_timeout_protection_ms
        if timeout_ms <= 0:
            self.get_logger().warn(
                "comm_timeout_protection_ms<=0 -- 모터 하드웨어 통신두절 보호가 "
                "비활성화된 채로 실행됩니다. 노드가 죽으면 모터가 마지막 명령을 "
                "계속 유지할 수 있습니다 (권장하지 않음)."
            )
            return

        data = proto.build_set_timeout_command(timeout_ms)
        for channel in self.channels:
            message = can.Message(
                arbitration_id=proto.motor_send_id(channel.can_id),
                data=data, is_extended_id=False)
            try:
                self.bus.send(message)
            except can.CanError as exc:
                self.get_logger().error(
                    f"[{channel.name}] comm_timeout_protection_ms 설정 프레임 전송 실패: {exc}")

        # readback 대기 -- _on_can_message가 별도 Notifier 스레드에서 비동기로
        # 채워주므로 여기서는 짧게 폴링만 한다 (블로킹이지만 노드 생성자
        # 안이라 rclpy 실행기가 아직 안 돌고 있어도 상관없음).
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if all(ch.snapshot()["comm_timeout_confirmed_ms"] is not None for ch in self.channels):
                break
            time.sleep(0.02)

        for channel in self.channels:
            confirmed = channel.snapshot()["comm_timeout_confirmed_ms"]
            if confirmed is None:
                self.get_logger().error(
                    f"[{channel.name}] comm_timeout_protection 설정 응답(readback)을 "
                    f"못 받았습니다 -- 모터 하드웨어 통신두절 보호가 실제로 걸렸는지 "
                    f"확인 불가. CAN 배선/모터 전원을 확인하세요."
                )
            elif confirmed != timeout_ms:
                self.get_logger().error(
                    f"[{channel.name}] comm_timeout_protection 설정값 불일치: "
                    f"요청={timeout_ms}ms, 모터 응답={confirmed}ms."
                )
            else:
                self.get_logger().info(
                    f"[{channel.name}] comm_timeout_protection 확인됨: {confirmed}ms "
                    f"(이 시간 안에 새 명령이 안 오면 모터가 자체적으로 정지)"
                )

    # ------------------------------------------------------------------ #
    # cmd_vel in
    # ------------------------------------------------------------------ #
    def _on_cmd_vel(self, msg: Twist):
        with self._cmd_lock:
            self._desired_v = msg.linear.x
            self._desired_w = msg.angular.z
            self._last_cmd_vel_time = self.get_clock().now()

    def _on_cmd_vel_safety(self, msg: Twist):
        with self._cmd_lock:
            self._safety_v = msg.linear.x
            self._safety_w = msg.angular.z
            self._last_safety_time = self.get_clock().now()

    # ------------------------------------------------------------------ #
    # main control loop
    # ------------------------------------------------------------------ #
    def _control_loop(self):
        now = self.get_clock().now()
        with self._cmd_lock:
            safety_age_s = (
                (now - self._last_safety_time).nanoseconds * 1e-9
                if self._last_safety_time is not None
                else None
            )

            if safety_age_s is not None and safety_age_s <= self.cmd_vel_safety_timeout:
                # /cmd_vel_safety가 최근에 수신됨 -> /cmd_vel 완전히 무시하고
                # 이 값을 그대로 사용. 기존 cmd_vel_timeout 로직과는 별개(이쪽은
                # /cmd_vel의 나이를 아예 보지 않는다).
                v, w = self._safety_v, self._safety_w
            else:
                age_s = (now - self._last_cmd_vel_time).nanoseconds * 1e-9
                v, w = self._desired_v, self._desired_w
                if age_s > self.cmd_vel_timeout:
                    # Safety: no recent cmd_vel -> command zero speed rather than
                    # relying solely on the drive's own 500ms heartbeat cutoff.
                    v, w = 0.0, 0.0

        v_left, v_right = self._skid_steer_inverse(v, w)

        self._send_speed_command(self.left, v_left)
        self._send_speed_command(self.right, v_right)

        # occasionally poll Motor Status 1 (temp/voltage/error flags) --
        # not needed every cycle, ~2 Hz is plenty for diagnostics
        self._status1_poll_counter += 1
        if self._status1_poll_counter >= max(1, int(round(1.0 / self.control_period / 2.0))):
            self._status1_poll_counter = 0
            self._poll_status1(self.left)
            self._poll_status1(self.right)

        self._publish_feedback(now)

    def _skid_steer_inverse(self, v: float, w: float):
        """v [m/s], w [rad/s] -> (v_left, v_right) wheel linear speed [m/s].

        w는 angular_slip_compensation_factor로 미리 부풀려서 바퀴 speed를
        계산한다 -- 실측상 궤도 슬립 때문에 실제 회전은 이 계수만큼 명령보다
        덜 나오므로, 그만큼 더 세게 명령해서 상쇄한다(feedforward 보정,
        피드백 아님 -- 실제 슬립량은 지형/속도에 따라 달라지므로 이건
        평균적인 근사치일 뿐이다)."""
        w_compensated = w * self.angular_slip_compensation_factor
        half_track = self.track_width / 2.0
        v_left = v - w_compensated * half_track
        v_right = v + w_compensated * half_track
        return v_left, v_right

    def _send_speed_command(self, channel: MotorChannel, wheel_linear_speed_m_s: float):
        wheel_angular_dps = math.degrees(wheel_linear_speed_m_s / channel.wheel_radius_m)
        # convert wheel-frame dps to the actuator's OUTPUT SHAFT dps
        # (accounting for any external gear stage) and this motor's
        # physical mounting direction sign
        output_shaft_dps = wheel_angular_dps * channel.external_gear_ratio * channel.direction_sign

        # clamp for safety regardless of what Nav2/EKF asked for
        output_shaft_dps = max(-self.max_wheel_speed_dps,
                                min(self.max_wheel_speed_dps, output_shaft_dps))

        data = proto.build_speed_command(output_shaft_dps)
        message = can.Message(
            arbitration_id=proto.motor_send_id(channel.can_id),
            data=data,
            is_extended_id=False,
        )
        self._send_can_message(channel.name, message)

    def _send_can_message(self, channel_name: str, message: can.Message):
        # [manual+return 통합] can_driver_node._send_frame()과 동일 패턴: bus가
        # 없으면 매 호출마다 재연결을 시도(쿨다운 내부에 있음)하고, 있으면
        # 보내되 연속 실패가 임계치를 넘으면 재연결을 트리거한다.
        if self.bus is None:
            self._attempt_reconnect()
            if self.bus is None:
                self.get_logger().warn(
                    f"[{channel_name}] CAN bus not open on {self._can_interface} -- "
                    f"command not sent.", throttle_duration_sec=1.0)
            return

        try:
            self.bus.send(message)
            self._consecutive_send_failures = 0
        except can.CanError as exc:
            # [2026-08-19] throttle 추가. 이 메서드가 control_rate_hz(기본 50)로
            # 계속 불리는데, 모터 전원이 꺼진 채로 방치되면(배터리 OFF 등)
            # 매 사이클 실패해서 로그가 스팸으로 쏟아짐 -- 2초에 한 번으로 줄임.
            self._consecutive_send_failures += 1
            self.get_logger().warn(
                f"[{channel_name}] CAN send failed: {exc}", throttle_duration_sec=2.0)
            if self._consecutive_send_failures >= self.reconnect_after_failures:
                self._attempt_reconnect()

    def _poll_status1(self, channel: MotorChannel):
        data = proto.build_read_status1_command()
        message = can.Message(
            arbitration_id=proto.motor_send_id(channel.can_id),
            data=data,
            is_extended_id=False,
        )
        self._send_can_message(channel.name, message)

    # ------------------------------------------------------------------ #
    # publishing
    # ------------------------------------------------------------------ #
    def _publish_feedback(self, now):
        left = self.left.snapshot()
        right = self.right.snapshot()
        # [수정] 한쪽이라도 stale이면 차동/스키드 조향 계산(v/w) 자체가 의미가
        # 없다 -- 두 바퀴 속도가 "같은 순간"의 값이어야 성립하는데, 한쪽이
        # 옛날 값이면 그 전제가 깨진다.
        any_stale = left["stale"] or right["stale"]

        # ---- joint states ----
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [left["angle_rad"], right["angle_rad"]]
        # [수정] stale인 쪽 속도는 0으로 보고한다. 피드백이 끊긴 채로 마지막
        # 속도값을 새 timestamp로 계속 내보내면, 구독 측(특히 아래 odom 적분과
        # robot_localization)이 "지금도 그 속도로 계속 움직이는 중"이라고 계속
        # 믿게 된다 -- 실제로는 CAN 응답 자체가 안 오는 상황이라 진짜 속도를
        # 전혀 알 수 없는데도. position(각도)은 그대로 둔다: 마지막으로 확인된
        # 절대 각도를 유지하는 것 자체는 위험한 정보 왜곡이 아니고, 여기서
        # 0으로 만들면 오히려 "원점으로 돌아갔다"는 또 다른 거짓 신호가 된다.
        js.velocity = [
            0.0 if left["stale"] else left["speed_rad_s"],
            0.0 if right["stale"] else right["speed_rad_s"],
        ]
        # NOTE: 'effort' is reused here to carry torque current in Amps
        # (not Nm) for convenience -- see README. The 3.3 current-protection
        # logic in the control guide should subscribe to this topic.
        js.effort = [left["current_a"], right["current_a"]]
        self.joint_state_pub.publish(js)

        # ---- wheel odometry (dead-reckoning for debug/rviz; robot_localization
        # is configured to trust ONLY vx from this topic, see control_guide 4.2) ----
        # [수정] stale이면 v/w를 0으로 보고하고 데드레커닝 적분도 건너뛴다 --
        # 마지막 위치에 그대로 고정. 이전엔 stale 상태에서도 마지막 속도를 계속
        # 적분해서, 바퀴가 실제로는 멈췄거나 응답 자체가 안 오는데 odom이
        # 계속 앞으로 나아가는(존재하지 않는 이동을 쌓는) 결과를 냈다.
        if any_stale:
            v, w = 0.0, 0.0
        else:
            v_left = left["speed_rad_s"] * self.left.wheel_radius_m
            v_right = right["speed_rad_s"] * self.right.wheel_radius_m
            v = (v_left + v_right) / 2.0
            w = (v_right - v_left) / self.track_width if self.track_width > 1e-6 else 0.0

        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now
        if not any_stale and 0.0 < dt < 1.0:  # guard against startup / clock jump artifacts
            self._odom_theta += w * dt
            self._odom_x += v * math.cos(self._odom_theta) * dt
            self._odom_y += v * math.sin(self._odom_theta) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        qz = math.sin(self._odom_theta / 2.0)
        qw = math.cos(self._odom_theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Large covariances on everything except vx: this is a DEBUG /
        # dead-reckoning estimate. robot_localization's odom0_config
        # (control_guide 4.2) is what actually decides what gets trusted --
        # these covariance numbers are not authoritative on their own, but
        # set them sanely in case anything else consumes this topic directly.
        odom.pose.covariance[0] = 1e6
        odom.pose.covariance[7] = 1e6
        odom.pose.covariance[35] = 1e6
        # [수정] stale이면 vx도 못 믿는다고 명시 -- robot_localization의
        # odom0_config(control_guide 4.2)가 실제 신뢰 여부를 최종 결정하지만,
        # 이 필드 자체도 실제 상태를 반영하도록 맞춰둔다 (다른 소비자가 이
        # 토픽을 직접 구독해서 쓸 경우에 대비).
        odom.twist.covariance[0] = 1e6 if any_stale else 0.01  # vx: stale이면 미신뢰, 아니면 신뢰
        odom.twist.covariance[35] = 1e6       # wz: NOT trusted (track slip)
        self.odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self._odom_x
            t.transform.translation.y = self._odom_y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)

        # ---- diagnostics ----
        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()
        for name, ch_snapshot in (("left", left), ("right", right)):
            status = DiagnosticStatus()
            status.name = f"rmd_x8/{name}"
            status.hardware_id = name
            # [수정] "받다가 끊김"(진짜 통신 두절, STALE -- stability_monitor_node의
            # fault 래치가 이걸 보고 걸림)과 "아직 한 번도 못 받음"(부팅 직후
            # 누구나 거치는 정상 구간, WARN -- 래치 안 걸림)을 구분한다.
            if ch_snapshot["stale_after_having_feedback"]:
                status.level = DiagnosticStatus.STALE
                status.message = "no feedback received recently"
            elif ch_snapshot["stale"]:
                status.level = DiagnosticStatus.WARN
                status.message = "no feedback received yet (starting up)"
            elif ch_snapshot["status1"] and ch_snapshot["status1"].error_flags:
                status.level = DiagnosticStatus.ERROR
                status.message = ",".join(ch_snapshot["status1"].error_flags)
            else:
                status.level = DiagnosticStatus.OK
                status.message = "ok"
            # [수정] 통신두절 보호(0xB3)가 실제로 확인됐는지 상시 노출 -- 시작
            # 시점 로그만 보고 놓치면(터미널을 안 보고 있었다든지) 이 안전장치가
            # 실제로 안 걸려 있는 채로 계속 운행할 수 있으므로, 매 주기 진단
            # 메시지에도 반복해서 남긴다.
            confirmed_ms = ch_snapshot["comm_timeout_confirmed_ms"]
            status.values.append(KeyValue(
                key="comm_timeout_protection_confirmed_ms",
                value=(str(confirmed_ms) if confirmed_ms is not None else "UNCONFIRMED")))
            if confirmed_ms is None and status.level == DiagnosticStatus.OK:
                status.level = DiagnosticStatus.WARN
                status.message = "comm_timeout_protection(0xB3) readback not confirmed"
            status.values.append(KeyValue(key="torque_current_a", value=f'{ch_snapshot["current_a"]:.3f}'))
            if ch_snapshot["temperature_c"] is not None:
                status.values.append(KeyValue(key="temperature_c", value=str(ch_snapshot["temperature_c"])))
            if ch_snapshot["status1"] is not None:
                status.values.append(KeyValue(key="voltage_v", value=f'{ch_snapshot["status1"].voltage_v:.1f}'))
                status.values.append(KeyValue(key="mos_temperature_c", value=str(ch_snapshot["status1"].mos_temperature_c)))
            diag.status.append(status)
        self.diag_pub.publish(diag)

    def destroy_node(self):
        # best-effort: stop motors before shutting down the node
        if self.bus is not None:
            try:
                for ch in self.channels:
                    data = proto.build_stop_command()
                    msg = can.Message(arbitration_id=proto.motor_send_id(ch.can_id),
                                       data=data, is_extended_id=False)
                    self.bus.send(msg)
            except Exception:  # noqa: BLE001
                pass
        try:
            if self._notifier is not None:
                self._notifier.stop()
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RmdX8DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()