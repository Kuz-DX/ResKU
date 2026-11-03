
import asyncio
import time
from bleak import BleakScanner, BleakClient

# ==========================================
# 사용자 설정
# ==========================================
BMS_MAC = "41:1A:01:01:3C:06"

DURATION_MINUTES = 10   # 총 측정 시간 (분)
INTERVAL_SECONDS = 1    # 측정 간격 (초)

DEBUG_MODE = True       # RAW 데이터 출력 여부

NOTIFY_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
WRITE_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"

REQUEST = bytes.fromhex("D2 03 00 28 00 03 96 60")


# ==========================================
# CRC16 MODBUS
# ==========================================
def crc16(data):
    crc = 0xFFFF

    for b in data:
        crc ^= b

        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc


# ==========================================
# BMS 응답 파싱
# ==========================================
def parse_bms_frame(frame):

    if len(frame) != 11:
        return None

    expected_crc = crc16(frame[:-2])
    received_crc = int.from_bytes(frame[-2:], "little")

    if expected_crc != received_crc:
        if DEBUG_MODE:
            print("[DEBUG] CRC 오류")
        return None

    raw_voltage = int.from_bytes(frame[3:5], "big")
    raw_current = int.from_bytes(frame[5:7], "big")

    voltage = raw_voltage / 10.0
    current = (raw_current - 30000) / 10.0
    power = voltage * current

    if DEBUG_MODE:
        print("\n========== BMS RAW DEBUG ==========")
        print(f"FRAME       : {frame.hex(' ').upper()}")
        print(f"RAW VOLTAGE : {raw_voltage}")
        print(f"RAW CURRENT : {raw_current}")
        print(f"VOLTAGE     : {voltage:.2f} V")
        print(f"CURRENT     : {current:.2f} A")
        print(f"POWER       : {power:.2f} W")
        print("===================================")

    return voltage, current, power


# ==========================================
# 메인
# ==========================================
async def main():

    duration_seconds = DURATION_MINUTES * 60

    if DURATION_MINUTES <= 0 or INTERVAL_SECONDS <= 0:
        raise ValueError("측정 시간과 간격은 0보다 커야 합니다.")

    print("BMS 검색 중...")

    device = await BleakScanner.find_device_by_address(
        BMS_MAC,
        timeout=20.0
    )

    if device is None:
        print("BMS 검색 실패")
        return

    async with BleakClient(device, timeout=30.0) as client:

        print("BMS 연결 성공")

        rx_queue = asyncio.Queue()

        # BLE 알림 수신
        def on_notify(sender, data):
            rx_queue.put_nowait(bytes(data))

        await client.start_notify(NOTIFY_UUID, on_notify)

        # ==================================
        # BMS 데이터 요청
        # ==================================
        async def read_bms():

            # 이전 응답 제거
            while not rx_queue.empty():
                rx_queue.get_nowait()

            await client.write_gatt_char(
                WRITE_UUID,
                REQUEST,
                response=False
            )

            buffer = bytearray()

            while True:

                data = await rx_queue.get()
                buffer.extend(data)

                while buffer:

                    if buffer[0] != 0xD2:
                        del buffer[0]
                        continue

                    if len(buffer) < 3:
                        break

                    if buffer[1:3] != b"\x03\x06":
                        del buffer[0]
                        continue

                    if len(buffer) < 11:
                        break

                    frame = bytes(buffer[:11])

                    expected_crc = crc16(frame[:-2])
                    received_crc = int.from_bytes(
                        frame[-2:], "little"
                    )

                    if expected_crc != received_crc:
                        del buffer[0]
                        continue

                    result = parse_bms_frame(frame)

                    if result is not None:
                        return result

                    del buffer[0]

        # ==================================
        # 측정 데이터 저장
        # ==================================
        power_values = []
        current_values = []
        voltage_values = []

        timeout_count = 0
        zero_current_count = 0

        start_time = time.monotonic()
        end_time = start_time + duration_seconds
        next_measurement = start_time

        print("\n========== 측정 설정 ==========")
        print(f"총 측정 시간 : {DURATION_MINUTES:g} 분")
        print(f"측정 간격    : {INTERVAL_SECONDS:g} 초")
        print(f"DEBUG MODE   : {DEBUG_MODE}")
        print("================================\n")

        try:

            while next_measurement < end_time:

                wait_seconds = (
                    next_measurement - time.monotonic()
                )

                if wait_seconds > 0:
                    await asyncio.sleep(wait_seconds)

                remaining = (
                    end_time - time.monotonic()
                )

                if remaining <= 0:
                    break

                try:

                    voltage, current, power = (
                        await asyncio.wait_for(
                            read_bms(),
                            timeout=min(3.0, remaining)
                        )
                    )

                    voltage_values.append(voltage)
                    current_values.append(current)
                    power_values.append(power)

                    elapsed = (
                        time.monotonic() - start_time
                    )

                    if current == 0:
                        zero_current_count += 1
                        status = " <-- 전류 0A 감지"
                    else:
                        status = ""

                    print(
                        f"[{elapsed:7.1f}초] "
                        f"전압: {voltage:.1f} V | "
                        f"전류: {current:.1f} A | "
                        f"전력: {power:.2f} W"
                        f"{status}"
                    )

                except asyncio.TimeoutError:

                    timeout_count += 1
                    print(
                        "[WARNING] BMS 응답 시간 초과"
                    )

                # 다음 측정 시간 설정
                next_measurement += INTERVAL_SECONDS

                now = time.monotonic()

                # 지연된 측정 시점 건너뛰기
                if next_measurement < now:

                    skipped = (
                        int(
                            (now - next_measurement)
                            / INTERVAL_SECONDS
                        ) + 1
                    )

                    next_measurement += (
                        skipped * INTERVAL_SECONDS
                    )

        finally:

            elapsed = (
                time.monotonic() - start_time
            )

            print("\n========== 최종 측정 결과 ==========")

            print(
                f"측정 경과 시간 : {elapsed / 60:.2f} 분"
            )

            print(
                f"측정 간격      : {INTERVAL_SECONDS:g} 초"
            )

            print(
                f"측정 성공 횟수 : {len(power_values)} 회"
            )

            print(
                f"응답 시간 초과 : {timeout_count} 회"
            )

            print(
                f"전류 0A 횟수   : {zero_current_count} 회"
            )

            if power_values:

                avg_voltage = (
                    sum(voltage_values)
                    / len(voltage_values)
                )

                avg_current = (
                    sum(current_values)
                    / len(current_values)
                )

                avg_power = (
                    sum(power_values)
                    / len(power_values)
                )

                print(
                    f"평균 전압 : {avg_voltage:.2f} V"
                )

                print(
                    f"평균 전류 : {avg_current:.2f} A"
                )

                print(
                    f"평균 전력 : {avg_power:.2f} W"
                )

                print(
                    f"최대 전력 : {max(power_values):.2f} W"
                )

                print(
                    f"최소 전력 : {min(power_values):.2f} W"
                )

            else:
                print("측정 데이터 없음")

            print("====================================")

            if client.is_connected:
                await client.stop_notify(NOTIFY_UUID)


if __name__ == "__main__":

    try:
        asyncio.run(main())

    except KeyboardInterrupt:
        print("\n프로그램 종료")
