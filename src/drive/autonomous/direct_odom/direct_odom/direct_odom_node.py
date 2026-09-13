#!/usr/bin/env python3
"""
direct_odom_node

wheel odom(vx) + IMU orientation(roll/pitch/yaw)만으로 만든 독립적인
dead-reckoning baseline. robot_localization EKF(ekf_local.yaml)와 동시에
띄워서 같은 rosbag/실주행 데이터로 A/B 비교하기 위한 목적.
(배경: HANDOFF.md "EKF X/Y 위치 covariance 무한정 증가" 섹션 -- 이 로봇
센서 구성(vx, roll, pitch, yaw 딱 4개만 관측)에서 15-state EKF는 나머지
11개 state가 원리적으로 관측 불가능해서 covariance ceiling이 predict
cycle의 X 82.4%/Y 96.9%에서 상시 개입 중. vy/vz/ax/ay/az 같은 관측 안
되는 state를 아예 만들지 않는 단순 dead-reckoning이 실제 trajectory
측면에서 더 안정적인지 확인하는 것이 이 노드의 존재 이유.)

모델 (nonholonomic 가정, body frame velocity를 [vx, 0, 0]으로 고정):
    v_world = R(q_world_base) * [vx, 0, 0]
    position += v_world * dt

vy/vz/ax/ay/az/각속도처럼 이 센서 구성으로 관측 안 되는 state는 아예
만들지 않는다(EKF처럼 별도 state로 추정 후 process noise로 계속
쌓이는 문제 자체를 구조적으로 피함).

로봇의 IMU(myAHRS+)는 raw gyro를 노출하지 않고(각속도 미신뢰,
ekf_local.yaml imu0_config 참고) orientation(roll/pitch/yaw)만 제공하므로,
yaw도 자체 적분하지 않고 IMU가 준 절대 orientation을 그대로 쓴다(자체
적분하면 없는 정보를 만들어내는 것과 같음).

[2026-08-24 정정] myAHRS+는 magnetometer를 포함하는 정식 AHRS다(이전
분석에서 "나침반이 없어 yaw가 gyro drift만 가진다"고 적었던 건 틀림 --
myahrs_driver_node.cpp가 "$RPY,..." fused 출력만 파싱/발행하고 raw
magnetometer(mx,my,mz)를 ROS로 노출하지 않을 뿐, 디바이스 자체와 내부
AHRS fusion에는 magnetometer가 관여하고 있을 가능성이 높다 -- 이
드라이버 코드로는 raw mag 필드를 직접 확인할 수 없어 "포함 여부"는
장비 스펙 문제고 "raw 값을 ROS로 못 본다"는 건 이 드라이버 구현의
한계다). 따라서 yaw 오차 원인은 gyro drift 하나가 아니라 magnetometer
기반 오차(모터/배터리/고전류 배선의 자기장 교란, hard/soft-iron
distortion, 캘리브레이션 오차)까지 포함해서 봐야 한다 -- 아래
"IMU 품질 계측"과 HANDOFF.md의 관련 섹션 참고.

frame 관계 (reduced_odom_bringup.launch.py 실측):
  - /wheel/odom: header.frame_id=odom, child_frame_id=base_link,
    twist.twist.linear.x = base_link forward(+x) 속도 (rmd_x8_driver_node.py)
  - /imu: header.frame_id=imu_link, orientation은 world-referenced
    quaternion(myahrs_driver_node.cpp, tf2::Quaternion::setRPY, REP-103).
    sensor_msgs/Imu.msg 자체엔 방향 관례가 명시돼 있지 않지만,
    geometry_msgs/TransformStamped.msg의 공식 문서("Translation and
    rotation of child_frame_id from header.frame_id")와 동일한 "child
    frame이 parent frame 기준으로 어떻게 놓여 있는가"라는 ROS 전역 관례를
    그대로 따른다(tf2/robot_localization/imu_filter_madgwick 등이 전부
    이 관례로 IMU orientation을 소비함) -- orientation은 "imu_link가
    world 기준으로 어떤 자세인가"이고, v_world = q * v_imu * q^-1 로
    적용한다.
  - base_link -> imu_link는 항등회전이 아니다: 실측 마운트 오프셋
    roll=0.0555 rad, pitch=0.0134 rad, yaw=0 (reduced_odom_bringup.launch.py의
    base_to_imu_tf static_transform_publisher). 이 노드는 이 오프셋을
    tf에서 직접 조회해서 보정한다(하드코딩/추측하지 않음).
    tf2 Buffer.lookup_transform(target='base_link', source='imu_link')는
    TransformStamped.msg 문서 그대로 "child_frame_id(imu_link)의
    자세를 header.frame_id(base_link) 기준으로" 담아 돌려준다 -- 즉
    transform.rotation = q_base_imu ("imu_link가 base_link 기준으로
    어떤 자세인가"), q_base_imu와 IMU orientation(q_world_imu)은 둘 다
    "자식 프레임이 부모 프레임 기준으로 어떤 자세인가"라는 동일한 타입의
    양이라 그대로 체인 합성이 가능하다:

        q_world_base = q_world_imu * inverse(q_base_imu)

    (imu -> world, imu -> base 두 상대 자세에서 공통 자식 imu_link를
    소거해 base -> world를 얻는 표준 쿼터니언 합성. 유도와 5가지
    단독/복합 회전 케이스 재검증은 test/test_frame_chain.py 참고 --
    손으로 유도한 회전행렬 기댓값과 비교해서 순환 논증 없이 검증함.)

품질/강건성 계측 (이번 작업에서 추가):
  - IMU: quaternion norm/finite 체크, 연속 샘플 간 orientation 각도 차이
    (max_orientation_jump_deg) 및 yaw rate(max_yaw_rate_deg_s) 체크 --
    둘 다 기본은 disabled(0.0)이고, 실제 로그로 정상 범위를 확인한 뒤
    필요하면 켜는 용도. raw magnetometer는 이 드라이버가 노출하지 않아
    직접 계측 불가 -- 대신 roll/pitch/yaw를 주기적으로 로그에 남겨
    정지 상태 yaw drift/모터 on-off 전후 yaw shift를 사후 분석할 수 있게
    한다(HANDOFF.md 실차 테스트 절차 참고).
  - wheel: NaN/Inf, max_abs_vx, max_vx_jump 체크(기본 disabled) +
    vx_deadband(기본 0=disabled, encoder quantization/정지 노이즈 억제용
    입력 전처리 -- covariance ceiling처럼 state를 사후에 자르는 것과는
    다른 층위임).
  - dt: 기존 dt<=0/dt>max_dt 스킵 유지 + 카운터 계측.
  - IMU dropout: 기본은 완전 freeze(A안, 가장 보수적). 필요하면
    allow_last_orientation_hold로 짧은 시간(B안)만 마지막 orientation을
    들고 적분하게 켤 수 있음 -- 기본은 꺼짐.
  - wheel<->imu timestamp skew를 매 wheel 콜백마다 기록, 주기 로그로
    mean/min/max/p95 출력.
  - 주기(diagnostic_period_sec) 로그로 위 모든 카운터/통계 요약
    ([DIRECT_ODOM][SUMMARY] 태그, ekf.cpp의
    [EKF_COVARIANCE_CEILING][SUMMARY] 계측과 같은 패턴).

의도적으로 안 한 것: pose/twist covariance를 새로 만들어 채우지 않음
(정확한 stochastic model 없이 그럴듯한 숫자를 채우는 건 의미 없음 --
이미 이 코드베이스에 있는 관례 하나만 재사용: freeze/reject로 이번
publish가 "믿을 수 없는 상태"일 때 twist.covariance[0]에 1e6을 넣는
것(rmd_x8_driver_node.py가 stale일 때 쓰는 것과 동일한 sentinel) --
이건 stochastic model이 아니라 이미 있는 "이 값 믿지 마라" 플래그 재사용.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector(q, v):
    """v' = q * v * q^-1, q=(x,y,z,w) normalized, v=(x,y,z)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def quat_angle_deg(q1, q2):
    """두 orientation 사이의 최소 회전각(deg). double-cover(q, -q 동일 회전)
    대비 dot에 abs를 취한다."""
    dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]
    dot = max(-1.0, min(1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def quat_to_rpy(q):
    """setRPY(intrinsic ZYX)의 역변환. 계측/로그 전용 -- 제어 로직에는
    안 쓰인다(제어는 쿼터니언을 직접 rotate_vector로 사용)."""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_deg(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def percentile(sorted_vals, p):
    if not sorted_vals:
        return float('nan')
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * p
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_vals[int(k)]
    return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)


class DirectOdomNode(Node):

    def __init__(self):
        super().__init__('direct_odom_node')

        self.declare_parameter('wheel_odom_topic', '/wheel/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('output_topic', '/direct_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # ekf_local.yaml의 sensor_timeout(0.3s)과 동일값 -- 같은 기준으로
        # A/B 비교하기 위해 일부러 맞춤.
        self.declare_parameter('imu_timeout_sec', 0.3)
        self.declare_parameter('max_dt_sec', 1.0)
        self.declare_parameter('tf_wait_log_period_sec', 2.0)

        # --- 이번 작업에서 추가된 강건성 파라미터 (전부 기본은 안전/보수
        # 쪽 -- disabled=0.0 이거나, hold는 기본 off) ---
        self.declare_parameter('vx_deadband', 0.0)          # 0=disabled
        self.declare_parameter('max_abs_vx', 0.0)           # 0=disabled
        self.declare_parameter('max_vx_jump', 0.0)          # 0=disabled
        self.declare_parameter('max_orientation_jump_deg', 0.0)  # 0=disabled
        self.declare_parameter('max_yaw_rate_deg_s', 0.0)   # 0=disabled
        self.declare_parameter('allow_last_orientation_hold', False)
        self.declare_parameter('max_orientation_hold_time', 0.1)
        self.declare_parameter('diagnostic_period_sec', 5.0)
        self.declare_parameter('diagnostic_window_size', 1000)

        self.wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)
        self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        self.tf_log_period = float(self.get_parameter('tf_wait_log_period_sec').value)

        self.vx_deadband = float(self.get_parameter('vx_deadband').value)
        self.max_abs_vx = float(self.get_parameter('max_abs_vx').value)
        self.max_vx_jump = float(self.get_parameter('max_vx_jump').value)
        self.max_orientation_jump_deg = float(self.get_parameter('max_orientation_jump_deg').value)
        self.max_yaw_rate_deg_s = float(self.get_parameter('max_yaw_rate_deg_s').value)
        self.allow_last_orientation_hold = bool(self.get_parameter('allow_last_orientation_hold').value)
        self.max_orientation_hold_time = float(self.get_parameter('max_orientation_hold_time').value)
        self.diagnostic_period_sec = float(self.get_parameter('diagnostic_period_sec').value)
        window = int(self.get_parameter('diagnostic_window_size').value)

        # 적분 상태 (x, y, z만 -- vy/vz/ax/ay/az 같은 관측 안 되는 state는
        # 아예 만들지 않는다).
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self._last_wheel_stamp = None
        self._last_vx = None

        self._last_imu_q_world_imu = None   # 마지막으로 accept된 IMU quaternion
        self._last_imu_stamp = None
        self._prev_accepted_imu_q = None    # jump/rate 계산용 (하나 전)
        self._prev_accepted_imu_stamp = None

        self._q_base_imu = None             # base_link<->imu_link 마운트 회전, tf에서 조회 후 캐시(정적이므로 1회)

        # --- 진단 카운터 (section 7/9/11) ---
        self.wheel_msg_count = 0
        self.imu_msg_count = 0
        self.integration_count = 0
        self.skipped_integration_count = 0
        self.imu_stale_count = 0
        self.invalid_quaternion_count = 0
        self.wheel_reject_count = 0
        self.imu_reject_count = 0
        self.negative_dt_count = 0
        self.large_dt_count = 0

        self._vx_window = deque(maxlen=window)
        self._skew_window = deque(maxlen=window)
        self._rpy_deg_window = deque(maxlen=window)  # (roll,pitch,yaw) deg, 진단용
        self._max_skew_abs = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # /wheel/odom, /imu 둘 다 BEST_EFFORT로 publish 되므로(rmd_x8_driver_node.py
        # best_effort_qos, EKF 쪽도 동일 이유로 BEST_EFFORT 구독하도록 최근 수정됨
        # -- 커밋 "EKF 수렴 모니터링을 위한 QoS 설정 수정") 이 노드도 동일하게 맞춘다.
        # BEST_EFFORT 구독자는 RELIABLE publisher에도 호환되므로 /imu(기본 QoS)에도 안전.
        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.wheel_sub = self.create_subscription(
            Odometry, self.wheel_odom_topic, self._wheel_odom_cb, sub_qos)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._imu_cb, sub_qos)

        self.odom_pub = self.create_publisher(Odometry, self.output_topic, 10)

        self.diag_timer = self.create_timer(self.diagnostic_period_sec, self._log_diagnostics)

        self.get_logger().info(
            f'direct_odom_node: {self.wheel_odom_topic} + {self.imu_topic} '
            f'-> {self.output_topic} (frame={self.odom_frame}, '
            f'child_frame={self.base_frame}), robustness guards: '
            f'vx_deadband={self.vx_deadband} max_abs_vx={self.max_abs_vx} '
            f'max_vx_jump={self.max_vx_jump} '
            f'max_orientation_jump_deg={self.max_orientation_jump_deg} '
            f'max_yaw_rate_deg_s={self.max_yaw_rate_deg_s} '
            f'allow_last_orientation_hold={self.allow_last_orientation_hold}')

    # ------------------------------------------------------------------
    # tf
    # ------------------------------------------------------------------
    def _try_resolve_mount_offset(self, imu_frame_id):
        """base_link <-> imu_link 정적 회전을 tf에서 조회 (하드코딩 금지 -- reduced_odom_bringup.launch.py의
        base_to_imu_tf 값이 바뀌어도 이 노드는 항상 실제 tf를 따라간다)."""
        if self._q_base_imu is not None:
            return True
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, imu_frame_id, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'{self.base_frame} <- {imu_frame_id} tf 대기 중: {e}',
                throttle_duration_sec=self.tf_log_period)
            return False
        r = t.transform.rotation
        self._q_base_imu = quat_normalize((r.x, r.y, r.z, r.w))
        self.get_logger().info(
            f'{self.base_frame} <- {imu_frame_id} 마운트 회전 tf 확보, 적분 시작')
        return True

    # ------------------------------------------------------------------
    # IMU
    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        self.imu_msg_count += 1

        raw = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        if not all(math.isfinite(v) for v in raw):
            self.invalid_quaternion_count += 1
            self.imu_reject_count += 1
            self.get_logger().warn(
                f'IMU quaternion에 NaN/Inf: {raw} -- 이 샘플 폐기',
                throttle_duration_sec=2.0)
            return

        norm = math.sqrt(sum(v * v for v in raw))
        if norm < 1e-3:
            self.invalid_quaternion_count += 1
            self.imu_reject_count += 1
            self.get_logger().warn(
                f'IMU quaternion norm={norm:.6f} (~0, 깨진 값) -- 이 샘플 폐기',
                throttle_duration_sec=2.0)
            return

        q = quat_normalize(raw)

        if not self._try_resolve_mount_offset(msg.header.frame_id):
            return

        this_stamp = Time.from_msg(msg.header.stamp)

        roll, pitch, yaw = quat_to_rpy(q)
        self._rpy_deg_window.append(
            (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

        # 연속 accept된 샘플 대비 orientation jump / yaw rate 계측 + (활성화 시) reject.
        if self._prev_accepted_imu_q is not None and self._prev_accepted_imu_stamp is not None:
            dt_imu = (this_stamp - self._prev_accepted_imu_stamp).nanoseconds * 1e-9
            ang_diff_deg = quat_angle_deg(q, self._prev_accepted_imu_q)

            if self.max_orientation_jump_deg > 0.0 and ang_diff_deg > self.max_orientation_jump_deg:
                self.imu_reject_count += 1
                self.get_logger().warn(
                    f'IMU orientation jump {ang_diff_deg:.2f}deg > '
                    f'max_orientation_jump_deg={self.max_orientation_jump_deg} -- 이 샘플 폐기',
                    throttle_duration_sec=2.0)
                return

            if self.max_yaw_rate_deg_s > 0.0 and dt_imu > 1e-6:
                prev_roll, prev_pitch, prev_yaw = quat_to_rpy(self._prev_accepted_imu_q)
                yaw_diff_deg = wrap_deg(math.degrees(yaw - prev_yaw))
                yaw_rate = yaw_diff_deg / dt_imu
                if abs(yaw_rate) > self.max_yaw_rate_deg_s:
                    self.imu_reject_count += 1
                    self.get_logger().warn(
                        f'IMU yaw rate {yaw_rate:.1f}deg/s > '
                        f'max_yaw_rate_deg_s={self.max_yaw_rate_deg_s} -- 이 샘플 폐기',
                        throttle_duration_sec=2.0)
                    return

        self._last_imu_q_world_imu = q
        self._last_imu_stamp = this_stamp
        self._prev_accepted_imu_q = q
        self._prev_accepted_imu_stamp = this_stamp

    # ------------------------------------------------------------------
    # wheel odom (적분 tick)
    # ------------------------------------------------------------------
    def _wheel_odom_cb(self, msg: Odometry):
        self.wheel_msg_count += 1
        now_stamp = Time.from_msg(msg.header.stamp)

        if self._last_wheel_stamp is None:
            self._last_wheel_stamp = now_stamp
            return

        dt = (now_stamp - self._last_wheel_stamp).nanoseconds * 1e-9
        self._last_wheel_stamp = now_stamp

        if dt <= 0.0:
            self.negative_dt_count += 1
            self.skipped_integration_count += 1
            self.get_logger().warn(
                f'비정상 dt={dt:.4f}s ({self.wheel_odom_topic} timestamp 역행/중복) -- 이번 스텝 스킵',
                throttle_duration_sec=2.0)
            return
        if dt > self.max_dt_sec:
            # dropout 이후 큰 dt를 한 번에 적분하지 않고 스킵한다 -- 그 구간
            # 동안 orientation도 같이 stale했을 가능성이 높고(둘 다 같은
            # 원인, 예: 노드/네트워크 재시작), encoder 카운터가 이 gap
            # 동안 무엇을 했는지(오버플로/리셋) 보장이 없어서, "이 큰 dt에
            # 지금 들어온 vx를 그대로 곱해 한 번에 크게 전진시키는" 것보다
            # "그 구간의 이동을 놓치는" 쪽이 안전하다(단일 큰 점프로 인한
            # 오탐/후속 로직 오작동 위험이, 몇 스텝의 거리 누락보다 큼).
            self.large_dt_count += 1
            self.skipped_integration_count += 1
            self.get_logger().warn(
                f'비정상적으로 큰 dt={dt:.3f}s (> {self.max_dt_sec}s, clock jump/재시작/dropout 추정) '
                '-- 이번 스텝 스킵(큰 dt를 한 번에 적분하지 않음)',
                throttle_duration_sec=2.0)
            return

        vx = msg.twist.twist.linear.x
        if not math.isfinite(vx):
            self.wheel_reject_count += 1
            self.skipped_integration_count += 1
            self.get_logger().warn(
                f'/wheel/odom vx가 NaN/Inf({vx}) -- 이 샘플 폐기',
                throttle_duration_sec=2.0)
            return

        if self.max_abs_vx > 0.0 and abs(vx) > self.max_abs_vx:
            self.wheel_reject_count += 1
            self.skipped_integration_count += 1
            self.get_logger().warn(
                f'|vx|={abs(vx):.3f} > max_abs_vx={self.max_abs_vx} -- encoder glitch 추정, 이 샘플 폐기',
                throttle_duration_sec=2.0)
            return

        if self.max_vx_jump > 0.0 and self._last_vx is not None:
            jump = abs(vx - self._last_vx)
            if jump > self.max_vx_jump:
                self.wheel_reject_count += 1
                self.skipped_integration_count += 1
                self.get_logger().warn(
                    f'vx jump {jump:.3f} m/s(> max_vx_jump={self.max_vx_jump}) -- '
                    f'{self._last_vx:.3f}->{vx:.3f}, 이 샘플 폐기',
                    throttle_duration_sec=2.0)
                return

        self._last_vx = vx
        self._vx_window.append(vx)

        if self._q_base_imu is None or self._last_imu_q_world_imu is None:
            self.skipped_integration_count += 1
            self.get_logger().warn(
                'IMU 데이터/tf 아직 없음 -- 적분 대기',
                throttle_duration_sec=2.0)
            return

        imu_age = (now_stamp - self._last_imu_stamp).nanoseconds * 1e-9
        self._skew_window.append(imu_age)
        self._max_skew_abs = max(self._max_skew_abs, abs(imu_age))

        imu_stale = abs(imu_age) > self.imu_timeout_sec
        hold_orientation = (
            imu_stale and self.allow_last_orientation_hold
            and abs(imu_age) <= self.max_orientation_hold_time
        )
        if imu_stale:
            self.imu_stale_count += 1
            if hold_orientation:
                self.get_logger().warn(
                    f'IMU dropout: 마지막 /imu가 {imu_age:.3f}s 전(timeout={self.imu_timeout_sec}s) '
                    f'-- allow_last_orientation_hold로 마지막 orientation을 '
                    f'{self.max_orientation_hold_time}s까지 유지해서 적분 계속',
                    throttle_duration_sec=2.0)
            else:
                self.get_logger().warn(
                    f'IMU dropout: 마지막 /imu가 {imu_age:.3f}s 전(timeout={self.imu_timeout_sec}s) '
                    '-- 이번 스텝은 위치를 얼리고(v=0) 마지막 orientation으로 publish',
                    throttle_duration_sec=2.0)

        q_world_base = quat_normalize(
            quat_mul(self._last_imu_q_world_imu, quat_conjugate(self._q_base_imu)))

        integrate = (not imu_stale) or hold_orientation
        if integrate:
            vx_used = 0.0 if (self.vx_deadband > 0.0 and abs(vx) < self.vx_deadband) else vx
            v_world = rotate_vector(q_world_base, (vx_used, 0.0, 0.0))
            self.x += v_world[0] * dt
            self.y += v_world[1] * dt
            self.z += v_world[2] * dt
            self.integration_count += 1
        else:
            v_world = (0.0, 0.0, 0.0)
            self.skipped_integration_count += 1

        # imu_stale(=freeze 중이거나 hold로 버티는 중)이면 이 publish는
        # "믿지 말라"는 신호로 twist.covariance[0]에 1e6을 넣는다 --
        # rmd_x8_driver_node.py가 stale일 때 쓰는 것과 동일한 sentinel
        # 재사용이지, 새로운 stochastic covariance 모델이 아니다.
        self._publish(now_stamp, q_world_base, v_world, invalid=imu_stale)

    def _publish(self, stamp: Time, q_world_base, v_world, invalid: bool):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = q_world_base[0]
        odom.pose.pose.orientation.y = q_world_base[1]
        odom.pose.pose.orientation.z = q_world_base[2]
        odom.pose.pose.orientation.w = q_world_base[3]

        # 주의: nav_msgs/Odometry의 twist는 관례상 child_frame_id(base_link)
        # 기준이지만, 이 필드는 EKF와의 world-frame trajectory/velocity
        # 비교를 위해 의도적으로 world(odom) frame 속도를 담는다.
        odom.twist.twist.linear.x = v_world[0]
        odom.twist.twist.linear.y = v_world[1]
        odom.twist.twist.linear.z = v_world[2]
        # 각속도: myAHRS+가 raw gyro를 노출하지 않아 관측값이 없다 --
        # 임의로 만들어내지 않고 0(미기록)으로 둔다.

        if invalid:
            odom.twist.covariance[0] = 1e6

        self.odom_pub.publish(odom)

    # ------------------------------------------------------------------
    # 진단
    # ------------------------------------------------------------------
    def _log_diagnostics(self):
        skew = sorted(self._skew_window)
        vx = list(self._vx_window)
        rpy = list(self._rpy_deg_window)

        def stats(vals):
            if not vals:
                return 'n=0'
            return (f'n={len(vals)} min={min(vals):.4f} max={max(vals):.4f} '
                    f'mean={sum(vals) / len(vals):.4f}')

        skew_line = (
            f'skew(imu-wheel)[s]: {stats(skew)} p95={percentile(skew, 0.95):.4f}'
            if skew else 'skew: n=0')
        vx_line = f'vx[m/s]: {stats(vx)}'
        if rpy:
            last_r, last_p, last_y = rpy[-1]
            rpy_line = f'last_rpy[deg]=({last_r:.2f},{last_p:.2f},{last_y:.2f})'
        else:
            rpy_line = 'last_rpy: n/a'

        self.get_logger().info(
            '[DIRECT_ODOM][SUMMARY] '
            f'wheel_msgs={self.wheel_msg_count} imu_msgs={self.imu_msg_count} '
            f'integrations={self.integration_count} skipped={self.skipped_integration_count} '
            f'imu_stale={self.imu_stale_count} invalid_quat={self.invalid_quaternion_count} '
            f'wheel_reject={self.wheel_reject_count} imu_reject={self.imu_reject_count} '
            f'neg_dt={self.negative_dt_count} large_dt={self.large_dt_count} '
            f'max_skew_abs={self._max_skew_abs:.4f} | {skew_line} | {vx_line} | {rpy_line} | '
            f'pos=({self.x:.3f},{self.y:.3f},{self.z:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = DirectOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
