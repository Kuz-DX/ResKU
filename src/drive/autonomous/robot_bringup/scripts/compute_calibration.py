"""
calib_monitor.csv를 읽어서, 사용자가 알려준 시작/끝 시각(HH:MM:SS, wall clock)
구간의 필터 yaw 변화량을 실제 측정 각도와 비교해 effective_track_width_m을
다시 계산한다.

쓰는 법:
    python3 compute_calibration.py "14:32:10-14:32:15:90"   # 시작-끝:실제각도(도, 부호는 방향)
    python3 compute_calibration.py "14:32:10-14:32:15:90" "14:35:00-14:35:04:-82"
    (여러 구간을 한 번에 넣으면 각 구간이 제안하는 값과 그 평균을 같이 보여줌)

방향(왜 real/filt가 아니라 filt/real인가):
    filt_yaw(오도메트리)는 "바퀴가 실제로 얼마나 돌았나"를 CURRENT_TRACK_WIDTH로
    나눠서 계산한다(w_est = wheel_diff / track_width). 조이스틱 수동 회전은
    dps 명령이 track_width와 무관하게 그대로 바퀴에 전달되므로(드라이버 쪽
    설명 참고), 같은 물리적 바퀴 회전에 대해 track_width를 키우면 filt_yaw는
    작아진다 -- 즉 filt_yaw ∝ 1/track_width.
    그래서 "이번 시험에서 봤던 filt_yaw가 실제로 real_deg가 되게 하려면
    track_width를 얼마로 바꿔야 하나"는:
        new_track_width = CURRENT_TRACK_WIDTH * (filt_chg / real_deg)
    (real/filt가 아니라 filt/real로 곱한다 -- 반대로 하면 보정 방향이 뒤집힌다.
    실제 최초 보정치 도출 때도 이 방향으로 계산했었다: 0.4904 * (267.5/90) ≈ 1.46,
    이후 좌우 평균으로 1.58까지 조정됨.)
"""
import csv
import os
import sys
from datetime import datetime

CSV_PATH = 'calib_monitor.csv'
# 시험 당시 로봇에 실제로 적용돼 있던 값이어야 한다 -- launch에서
# effective_track_width_m:=X 로 다르게 띄웠다면 CALIB_CURRENT_TW=X 환경변수로
# 넘길 것 (매번 이 파일을 고치지 않도록).
CURRENT_TRACK_WIDTH = float(os.environ.get('CALIB_CURRENT_TW', 1.58))


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
    suggestions = []
    print(f'{"segment":24} {"real(deg)":>10} {"filt_chg(deg)":>14} {"이 구간 제안 track_width":>24}')
    for spec in sys.argv[1:]:
        t0, t1, real_deg = parse_spec(spec, today)
        window = [(t, y) for t, y in rows if t0 - 1 <= t <= t1 + 1]
        if len(window) < 2:
            print(f'  {spec}: 그 시간대 데이터가 없음 (로봇 연결 확인)')
            continue
        y0 = min(window, key=lambda x: abs(x[0] - t0))[1]
        y1 = min(window, key=lambda x: abs(x[0] - t1))[1]
        filt_chg = y1 - y0
        if abs(real_deg) < 1e-6:
            print(f'  {spec}: real_deg가 0이라 계산 불가')
            continue
        suggested = CURRENT_TRACK_WIDTH * (filt_chg / real_deg)
        suggestions.append(suggested)
        print(f'  {spec:22} {real_deg:10.1f} {filt_chg:14.1f} {suggested:24.3f}')

    if suggestions:
        avg = sum(suggestions) / len(suggestions)
        print(f'\n현재 effective_track_width_m = {CURRENT_TRACK_WIDTH}')
        print(f'제안 effective_track_width_m (평균) = {avg:.3f}')
        if len(suggestions) >= 2:
            spread = max(suggestions) - min(suggestions)
            print(f'구간별 편차 = {spread:.3f}  (편차가 크면 시험을 더 해서 평균을 안정시킬 것)')


if __name__ == '__main__':
    main()
