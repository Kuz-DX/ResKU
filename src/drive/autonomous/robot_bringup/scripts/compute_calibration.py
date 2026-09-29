"""
calib_monitor.csv를 읽어서, 사용자가 알려준 시작/끝 시각(HH:MM:SS, wall clock)
구간의 필터 yaw 변화량을 실제 측정 각도와 비교해 트랙 폭 보정 배수를 계산한다.

쓰는 법:
    python3 compute_calibration.py "14:32:10-14:32:15:90"   # 시작-끝:실제각도(도, 부호는 방향)
    python3 compute_calibration.py "14:32:10-14:32:15:90" "14:35:00-14:35:04:-82"
    (여러 구간을 한 번에 넣으면 평균 배수와 각 구간 개별값을 같이 보여줌)

현재 launch 기본값 effective_track_width_m=1.58 기준으로 계산하고,
"새 배수 = 1.58 / (측정 배율)"을 마지막에 출력한다.
"""
import csv
import sys
import time
from datetime import datetime

CSV_PATH = 'calib_monitor.csv'
CURRENT_TRACK_WIDTH = 1.58


def parse_spec(spec: str, today):
    win, real_deg = spec.rsplit(':', 1)
    t0, t1 = win.split('-')
    real_deg = float(real_deg)

    def to_epoch(hms):
        h, m, s = hms.split(':')
        dt = today.replace(hour=int(h), minute=int(m), second=int(s), microsecond=0)
        return dt.timestamp()

    return to_epoch(t0), to_epoch(t1), real_deg


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    rows = []
    with open(CSV_PATH) as f:
        for r in csv.DictReader(f):
            if r['filt_yaw_deg']:
                rows.append((float(r['wall_time']), float(r['filt_yaw_deg'])))
    if not rows:
        print('calib_monitor.csv에 yaw 데이터가 없습니다 -- calib_monitor.py가 로봇에 연결됐는지 확인하세요.')
        return

    today = datetime.now()
    ratios = []
    print(f'{"segment":24} {"real(deg)":>10} {"filt_chg(deg)":>14} {"ratio real/filt":>16}')
    for spec in sys.argv[1:]:
        t0, t1, real_deg = parse_spec(spec, today)
        window = [(t, y) for t, y in rows if t0 - 1 <= t <= t1 + 1]
        if len(window) < 2:
            print(f'  {spec}: 그 시간대 데이터가 없음 (로봇 연결 확인)')
            continue
        y0 = min(window, key=lambda x: abs(x[0] - t0))[1]
        y1 = min(window, key=lambda x: abs(x[0] - t1))[1]
        filt_chg = y1 - y0
        ratio = real_deg / filt_chg if abs(filt_chg) > 1e-6 else float('nan')
        ratios.append(ratio)
        print(f'  {spec:22} {real_deg:10.1f} {filt_chg:14.1f} {ratio:16.3f}')

    if ratios:
        avg = sum(ratios) / len(ratios)
        print(f'\n평균 ratio(real/filtered) = {avg:.3f}')
        print(f'현재 effective_track_width_m = {CURRENT_TRACK_WIDTH}')
        print(f'제안 effective_track_width_m = {CURRENT_TRACK_WIDTH * avg:.3f}  '
              f'(이 값으로 실제 회전량이 필터 yaw와 더 가까워짐)')


if __name__ == '__main__':
    main()
