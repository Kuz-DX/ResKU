
import asyncio
import time
from bleak import BleakScanner, BleakClient

# ==========================================
# 사용자 설정
# ==========================================
BMS_MAC = "41:1A:01:01:3C:06"

DURATION_MINUTES = 10   # 총 측정 시간 (분)
INTERVAL_SECONDS = 1    # 측정 간격 (초)

NOTIFY_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
WRITE_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"


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


async def main():
    # 상단에 선언된 상수 사용
    duration_minutes = DURATION_MINUTES
    interval_seconds = INTERVAL_SECONDS
    duration_seconds = duration_minutes * 60

    if duration_minutes <= 0 or interval_seconds <= 0:
        raise ValueError("측정 시간과 간격은 0보다 커야 합니다.")

    print("\nBMS 검색 중...")

    device = await BleakScanner.find_device_by_address(
        BMS_MAC, timeout=20.0
    )

    if device is None:
        print("BMS 검색 실패")
        return

    async with BleakClient(device, timeout=30.0) as client:
        print("BMS 연결 성공")

        rx_queue = asyncio.Queue()

        def on_notify(sender, data):
            rx_queue.put_nowait(bytes(data))

        await client.start_notify(NOTIFY_UUID, on_notify)

        request = bytes.fromhex("D2 03 00 28 00 03 96 60")

        async def read_bms():
            while not rx_queue.empty():
                rx_queue.get_nowait()

            await client.write_gatt_char(
                WRITE_UUID,
                request,
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

                    voltage = int.from_bytes(
                        frame[3:5], "big"
                    ) / 10.0

                    current = (
                        int.from_bytes(frame[5:7], "big")
                        - 30000
                    ) / 10.0

                    return voltage, current

        power_values = []
        timeout_count = 0

        start_time = time.monotonic()
        end_time = start_time + duration_seconds
        next_measurement = start_time

        print(
            f"\n총 {duration_minutes:g}분 동안 "
            f"{interval_seconds:g}초 간격으로 측정합니다.\n"
        )

        try:
            while next_measurement < end_time:
                wait_seconds = next_measurement - time.monotonic()

                if wait_seconds > 0:
                    await asyncio.sleep(wait_seconds)

                remaining = end_time - time.monotonic()

                if remaining <= 0:
                    break

                try:
                    voltage, current = await asyncio.wait_for(
                        read_bms(),
                        timeout=min(3.0, remaining)
                    )

                    power = voltage * current
                    power_values.append(power)

                    elapsed = time.monotonic() - start_time

                    print(
                        f"[{elapsed:7.1f}초] "
                        f"전압: {voltage:.1f} V | "
                        f"전류: {current:.1f} A | "
                        f"전력: {power:.2f} W"
                    )

                except asyncio.TimeoutError:
                    timeout_count += 1
                    print("BMS 응답 시간 초과 — 평균에서 제외")

                next_measurement += interval_seconds

                now = time.monotonic()

                if next_measurement < now:
                    skipped = (
                        int((now - next_measurement) / interval_seconds)
                        + 1
                    )
                    next_measurement += skipped * interval_seconds

        finally:
            elapsed = time.monotonic() - start_time

            print("\n========== 측정 결과 ==========")
            print(f"측정 경과 시간: {elapsed / 60:.2f}분")
            print(f"설정 측정 간격: {interval_seconds:g}초")
            print(f"측정 성공 횟수: {len(power_values)}회")
            print(f"응답 시간 초과: {timeout_count}회")

            if power_values:
                average_power = sum(power_values) / len(power_values)
                print(f"평균 전력: {average_power:.2f} W")
            else:
                print("측정 데이터가 없어 평균을 계산할 수 없습니다.")

            print("==============================")

            if client.is_connected:
                await client.stop_notify(NOTIFY_UUID)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n프로그램 종료")
