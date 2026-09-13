#!/usr/bin/env python3
"""
motor_status_monitor.py

수동주행(can_driver_node) 중에 좌/우 구동 모터 상태(발열, 토크전류, 속도,
MOS 온도, 전압, 에러플래그)를 터미널에서 실시간으로 확인하는 순수 관측용
스크립트. ROS 노드가 아니라 CAN 버스를 can_driver_node와 같이 그냥 "듣기"만
한다 -- SocketCAN은 여러 프로세스가 같은 인터페이스를 동시에 read해도 서로
안 부딪힌다(broadcast라 소비 경쟁이 없음, can_driver_node 자신도 recv()를
아예 안 함 -- 이 스크립트가 새로 여는 소켓과는 완전히 독립).

두 가지 정보원:
  1) 수동주행 중이면 can_driver_node가 0xA2(속도 폐루프 제어) 명령을 매
     control_rate_hz(기본 50Hz)마다 계속 보내고, 모터가 그때마다 같은 0xA2로
     응답한다(온도/토크전류/속도/각도 포함) -- 그냥 버스를 듣기만 해도 실시간
     으로 들어옴, 이 스크립트는 아무것도 안 보내도 됨(완전 패시브).
  2) MOS(드라이버 기판) 온도/전압/에러플래그는 0xA2 응답에 없어서, 이
     스크립트가 낮은 주기(기본 2Hz)로 0x9A(Read Status 1) 요청 프레임을 직접
     보내서 받는다(--no-poll-status1로 끌 수 있음). 이건 can_driver_node의
     속도제어 프레임과 같은 커맨드가 아니라 별개의 읽기 요청이라 충돌 없음.

RMD-X8 프로토콜 상수/파싱은 rmd_x8_driver 패키지의 rmd_x8_protocol.py와 같은
스펙(MYACTUATOR Servo Motor Control Protocol V4.01)을 이 파일 안에 그대로
복제해뒀다 -- 패키지 간 의존성 없이 이 파일 하나로 독립 실행되게 하려고.

모터 ID 기본값(left=1 -> 0x141/0x241, right=2 -> 0x142/0x242)은
can_driver_node.py의 left_can_id/right_can_id 기본값과 동일하게 맞춰둠.

실행 위치: jecs(실제 로봇, CAN 어댑터 붙어있는 머신)에서 can_driver_node와
같이 실행. 아래 세 줄은 "골라서 하나만 실행"하는 예시임(순서대로 다 치는 게
아님):
    ros2 run can_driver motor_status_monitor
        # 기본값 그대로. 평소엔 이거 하나면 충분함.
    ros2 run can_driver motor_status_monitor --channel can_drive --left-id 1 --right-id 2
        # 위 줄과 결과 동일 -- 기본값을 명시적으로 적은 예시(ID/채널이 바뀌면 이렇게 값만 바꿔서 씀).
    python3 motor_status_monitor.py --no-poll-status1
        # 0x9A 요청도 아예 안 보내는 완전 패시브 모드(온도/전류/속도만 보임, MOS온도/전압/에러는 안 보임).

주의: can_driver_node와 rmd_x8_driver_node를 동시에 켜지 마세요(둘 다 같은
모터 ID로 0xA2를 보내면 지난번 겪은 것과 같은 CAN 충돌이 납니다) -- 이
모니터 스크립트 자체는 어느 쪽과 같이 켜도 안전합니다(기본적으로 읽기 전용,
0x9A 요청만 예외적으로 보냄).
"""
import argparse
import struct
import sys
import time

import can

CMD_SPEED_CONTROL = 0xA2
CMD_READ_STATUS_1 = 0x9A

# System_errorState 비트 의미 (RMD-X8 매뉴얼 section 2.14.3,
# rmd_x8_driver/rmd_x8_protocol.py의 ERROR_FLAGS와 동일)
ERROR_FLAGS = {
    0x0002: "motor_stall",
    0x0004: "low_voltage",
    0x0008: "over_voltage",
    0x0010: "over_current",
    0x0040: "power_overrun",
    0x0080: "calibration_write_error",
    0x0100: "over_speed",
    0x1000: "over_temperature",
    0x2000: "encoder_calibration_error",
}


def motor_send_id(motor_can_id: int) -> int:
    return 0x140 + motor_can_id


def motor_reply_id(motor_can_id: int) -> int:
    return 0x240 + motor_can_id


def build_read_status1_command() -> bytes:
    return bytes([CMD_READ_STATUS_1, 0, 0, 0, 0, 0, 0, 0])


def parse_a2_reply(data: bytes) -> dict:
    """
    0xA2 응답 (can_driver_node가 계속 보내는 속도명령의 응답, 패시브 수신):
        DATA[0] command byte
        DATA[1] temperature          int8,  1 C/LSB
        DATA[2:4] torque current iq  int16, 0.01 A/LSB
        DATA[4:6] output shaft speed int16, 1 dps/LSB
        DATA[6:8] output shaft angle int16, 1 degree/LSB
    """
    cmd, temp, iq, speed, angle = struct.unpack("<Bbhhh", data[:8])
    return {
        "temperature_c": temp,
        "torque_current_a": iq * 0.01,
        "speed_dps": float(speed),
        "angle_deg": angle,
    }


def parse_status1_reply(data: bytes) -> dict:
    """0x9A 응답: 모터온도 / MOS(기판)온도 / 브레이크 / 전압 / 에러플래그."""
    _, temp, mos_temp, brake, voltage, err = struct.unpack("<BbbBHH", data[:8])
    active_errors = [name for bit, name in ERROR_FLAGS.items() if err & bit]
    return {
        "temperature_c": temp,
        "mos_temperature_c": mos_temp,
        "brake_released": bool(brake),
        "voltage_v": voltage * 0.1,
        "error_flags": active_errors,
    }


class MotorState:
    def __init__(self, name):
        self.name = name
        self.temperature_c = None
        self.torque_current_a = None
        self.speed_dps = None
        self.mos_temperature_c = None
        self.voltage_v = None
        self.error_flags = []
        self.last_a2_time = None

    def is_stale(self, now, timeout=1.0):
        # can_driver_node가 죽었거나 CAN 버스가 끊긴 경우를 감지하기 위한
        # 타임아웃(정지 명령이라도 0xA2로 계속 보내므로, 응답이 끊기면 이상 신호).
        return self.last_a2_time is None or (now - self.last_a2_time) > timeout


def draw(states, poll_status1):
    now = time.time()
    header = f"{'':7s}{'temp(C)':>9s}{'current(A)':>12s}{'speed(dps)':>12s}"
    if poll_status1:
        header += f"{'MOS(C)':>9s}{'volt(V)':>8s}  errors"
    lines = [header]
    for st in states:
        temp = f"{st.temperature_c:9d}" if st.temperature_c is not None else f"{'--':>9s}"
        cur = f"{st.torque_current_a:12.2f}" if st.torque_current_a is not None else f"{'--':>12s}"
        spd = f"{st.speed_dps:12.1f}" if st.speed_dps is not None else f"{'--':>12s}"
        row = f"{st.name:7s}{temp}{cur}{spd}"
        if poll_status1:
            mos = f"{st.mos_temperature_c:9d}" if st.mos_temperature_c is not None else f"{'--':>9s}"
            volt = f"{st.voltage_v:8.1f}" if st.voltage_v is not None else f"{'--':>8s}"
            err = ",".join(st.error_flags) if st.error_flags else "-"
            row += f"{mos}{volt}  {err}"
        if st.is_stale(now):
            row += "   [STALE -- 응답 끊김, can_driver_node 켜져있는지 확인]"
        lines.append(row)
    sys.stdout.write("\033[H\033[J")  # 커서 홈 + 화면 지우기
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--channel', default='can_drive')
    parser.add_argument('--left-id', type=int, default=1,
                         help='can_driver_node의 left_can_id와 맞출 것 (기본 1 -> 0x141/0x241)')
    parser.add_argument('--right-id', type=int, default=2,
                         help='can_driver_node의 right_can_id와 맞출 것 (기본 2 -> 0x142/0x242)')
    parser.add_argument('--refresh-hz', type=float, default=5.0, help='화면 갱신 주기')
    parser.add_argument('--status1-hz', type=float, default=2.0,
                         help='0x9A(MOS온도/전압/에러) 요청 주기 [Hz]')
    parser.add_argument('--no-poll-status1', action='store_true',
                         help='완전 패시브 모드 -- 0x9A 요청 프레임을 아예 안 보냄 (온도/전류/속도만 보임)')
    args = parser.parse_args()

    poll_status1 = (not args.no_poll_status1) and args.status1_hz > 0

    left = MotorState('LEFT')
    right = MotorState('RIGHT')
    reply_map = {
        motor_reply_id(args.left_id): left,     # 기본: 0x241 -> LEFT
        motor_reply_id(args.right_id): right,   # 기본: 0x242 -> RIGHT
    }
    send_ids = [motor_send_id(args.left_id), motor_send_id(args.right_id)]

    bus = can.interface.Bus(channel=args.channel, bustype='socketcan')
    print(f'CAN bus opened on {args.channel} '
          f'({"패시브+0x9A polling" if poll_status1 else "완전 패시브"}) -- '
          f'LEFT=id{args.left_id}(0x{motor_reply_id(args.left_id):X}) '
          f'RIGHT=id{args.right_id}(0x{motor_reply_id(args.right_id):X}) -- Ctrl+C로 종료',
          file=sys.stderr)
    time.sleep(0.5)

    draw_period = 1.0 / args.refresh_hz
    status1_period = (1.0 / args.status1_hz) if poll_status1 else None
    last_draw = 0.0
    last_status1_poll = 0.0

    try:
        while True:
            now = time.time()

            if poll_status1 and (now - last_status1_poll) >= status1_period:
                cmd = build_read_status1_command()
                for mid in send_ids:
                    try:
                        bus.send(can.Message(arbitration_id=mid, data=cmd, is_extended_id=False))
                    except can.CanError:
                        pass
                last_status1_poll = now

            msg = bus.recv(timeout=0.02)
            if msg is not None and msg.arbitration_id in reply_map:
                st = reply_map[msg.arbitration_id]
                data = bytes(msg.data)
                if len(data) == 8 and data[0] == CMD_SPEED_CONTROL:
                    fb = parse_a2_reply(data)
                    st.temperature_c = fb['temperature_c']
                    st.torque_current_a = fb['torque_current_a']
                    st.speed_dps = fb['speed_dps']
                    st.last_a2_time = now
                elif len(data) == 8 and data[0] == CMD_READ_STATUS_1:
                    s1 = parse_status1_reply(data)
                    st.mos_temperature_c = s1['mos_temperature_c']
                    st.voltage_v = s1['voltage_v']
                    st.error_flags = s1['error_flags']

            if now - last_draw >= draw_period:
                draw([left, right], poll_status1)
                last_draw = now

    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
