"""
calib_monitor.csv에 남은 cmd_w 기록에서 "제자리 회전 구간"을 자동으로 찾아
번호를 매긴다. 사람이 시작/끝 시각을 시계 보고 적을 필요 없이, 각 구간이
몇 번째였는지와 실제로 잰 각도만 알려주면 effective_track_width_m을 계산한다.

0단계 -- 지난 기록 치우고 새로 시작하고 싶을 때 (지우지 않고 백업만 함):
    python3 compute_calibration.py --reset

1단계 -- 구간 목록 보기 (회전 시험을 다 마친 뒤):
    python3 compute_calibration.py --list

2단계 -- 번호=실제각도로 계산 (왼쪽 +, 오른쪽 -):
    python3 compute_calibration.py "1=90" "2=-82" "3=88" "4=-79"

옛날 방식(시작-끝 시각을 직접 적는 것)도 그대로 된다:
    python3 compute_calibration.py "14:32:10-14:32:15:90"

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
CURRENT_TRACK_WIDTH = float(os.environ.get('CALIB_CURRENT_TW', 1.244))

# cmd_w가 이 값 이상일 때만 "회전 중"으로 본다 (0 근처 float 잡음 제거)
W_THRESHOLD = 0.03
# 같은 회전 구간으로 묶는 최대 공백(초). tick 주기가 0.5s라 1.5s면 최대 2~3개
# 샘플이 비어도 안 끊긴다.
GAP_TOLERANCE_S = 1.5
# 이 이상 선속도가 섞이면 순수 제자리 회전이 아니라 커브 주행으로 보고 제외
V_PURITY_THRESHOLD = 0.03


def load_rows():
    if not os.path.exists(CSV_PATH):
        return None
    rows = []
    with open(CSV_PATH) as f:
        for r in csv.DictReader(f):
            if r['filt_yaw_deg']:
                rows.append((float(r['wall_time']), float(r['filt_yaw_deg']),
                             float(r['cmd_v']), float(r['cmd_w'])))
    return rows


def find_segments(rows):
    """cmd_w가 켜져 있는 연속 구간을 찾는다. 각 구간: (t0, t1, y0, y1, mixed_v)."""
    segments = []
    cur = None  # [t0, t1, y0, y1, mixed_v]
    for t, y, v, w in rows:
        if abs(w) >= W_THRESHOLD:
            mixed = abs(v) >= V_PURITY_THRESHOLD
            if cur is None:
                cur = [t, t, y, y, mixed]
            elif t - cur[1] <= GAP_TOLERANCE_S:
                cur[1] = t
                cur[3] = y
                cur[4] = cur[4] or mixed
            else:
                segments.append(cur)
                cur = [t, t, y, y, mixed]
        else:
            if cur is not None and t - cur[1] > GAP_TOLERANCE_S:
                segments.append(cur)
                cur = None
    if cur is not None:
        segments.append(cur)
    return segments


def print_segment_list(segments):
    if not segments:
        print('cmd_w가 걸린 구간을 하나도 못 찾았습니다 -- calib_monitor.py가 로봇에 '
              '연결된 상태에서 회전 시험을 하셨는지 확인하세요.')
        return
    print(f'{"#":>3} {"시작":>8} {"끝":>8} {"길이(s)":>8} {"filt_chg(deg)":>14}  비고')
    for i, (t0, t1, y0, y1, mixed) in enumerate(segments, 1):
        start_s = datetime.fromtimestamp(t0).strftime('%H:%M:%S')
        end_s = datetime.fromtimestamp(t1).strftime('%H:%M:%S')
        note = '이동+회전 혼합 (제자리 회전 아닐 수 있음)' if mixed else ''
        print(f'{i:>3} {start_s:>8} {end_s:>8} {t1 - t0:8.1f} {y1 - y0:14.1f}  {note}')
    print('\n실제로 잰 각도를 "번호=각도" 형식으로 순서대로 넣어서 다시 실행하세요, 예:')
    print('  python3 compute_calibration.py "1=90" "2=-82"')


def parse_time_window_spec(spec: str, today):
    win, real_deg = spec.rsplit(':', 1)
    t0, t1 = win.split('-')
    real_deg = float(real_deg)

    def to_epoch(hms):
        h, m, s = hms.split(':')
        dt = today.replace(hour=int(h), minute=int(m), second=int(s), microsecond=0)
        return dt.timestamp()

    return to_epoch(t0), to_epoch(t1), real_deg


def compute_and_print(items):
    """items: list of (label, real_deg, filt_chg)"""
    suggestions = []
    print(f'{"segment":24} {"real(deg)":>10} {"filt_chg(deg)":>14} {"이 구간 제안 track_width":>24}')
    for label, real_deg, filt_chg in items:
        if abs(real_deg) < 1e-6:
            print(f'  {label}: real_deg가 0이라 계산 불가')
            continue
        suggested = CURRENT_TRACK_WIDTH * (filt_chg / real_deg)
        suggestions.append(suggested)
        print(f'  {label:22} {real_deg:10.1f} {filt_chg:14.1f} {suggested:24.3f}')

    if suggestions:
        avg = sum(suggestions) / len(suggestions)
        print(f'\n현재 effective_track_width_m = {CURRENT_TRACK_WIDTH}')
        print(f'제안 effective_track_width_m (평균) = {avg:.3f}')
        if len(suggestions) >= 2:
            spread = max(suggestions) - min(suggestions)
            print(f'구간별 편차 = {spread:.3f}  (편차가 크면 시험을 더 해서 평균을 안정시킬 것)')


def reset_log():
    if not os.path.exists(CSV_PATH):
        print(f'{CSV_PATH}가 없습니다 -- 지울 게 없음.')
        return
    backup = f'calib_monitor_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv.bak'
    os.rename(CSV_PATH, backup)
    print(f'{CSV_PATH} -> {backup} 로 백업했습니다. calib_monitor.py를 다시 실행하면 '
          f'새 {CSV_PATH}가 만들어집니다 (--list 결과에 이전 기록은 안 섞임).')


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return

    if sys.argv[1] == '--reset':
        reset_log()
        return

    rows = load_rows()
    if rows is None:
        print(f'{CSV_PATH}가 없습니다 -- calib_monitor.py를 먼저 실행하세요.')
        return
    if not rows:
        print('calib_monitor.csv에 yaw 데이터가 없습니다 -- calib_monitor.py가 로봇에 연결됐는지 확인하세요.')
        return

    if sys.argv[1] == '--list':
        print_segment_list(find_segments(rows))
        return

    today = datetime.now()
    items = []
    segments = None  # lazily computed, only needed for "N=deg" specs
    for spec in sys.argv[1:]:
        if '=' in spec:
            idx_str, real_str = spec.split('=', 1)
            idx = int(idx_str)
            real_deg = float(real_str)
            if segments is None:
                segments = find_segments(rows)
            if not (1 <= idx <= len(segments)):
                print(f'  {spec}: 구간 {idx}번이 없습니다 (--list로 확인, 총 {len(segments)}개)')
                continue
            t0, t1, y0, y1, mixed = segments[idx - 1]
            if mixed:
                print(f'  {spec}: 경고 -- 이 구간은 이동+회전이 섞여 있어 결과가 부정확할 수 있음')
            items.append((f'#{idx}', real_deg, y1 - y0))
        else:
            t0, t1, real_deg = parse_time_window_spec(spec, today)
            window = [(t, y) for t, y, _, _ in rows if t0 - 1 <= t <= t1 + 1]
            if len(window) < 2:
                print(f'  {spec}: 그 시간대 데이터가 없음 (로봇 연결 확인)')
                continue
            y0 = min(window, key=lambda x: abs(x[0] - t0))[1]
            y1 = min(window, key=lambda x: abs(x[0] - t1))[1]
            items.append((spec, real_deg, y1 - y0))

    if items:
        compute_and_print(items)


if __name__ == '__main__':
    main()
