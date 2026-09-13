#!/usr/bin/env python3
"""
PathAlignCritic 등 실제 nav2_mppi_controller critic 공식(각 .cpp에서 그대로
옮김) + 실제 소프트맥스 공식(optimizer.cpp:390-392)을 재현해서, "직진 후보"
vs "곡선(경로) 추종 후보" 중 뭐가 선택되는지 예전 설정값과 새 설정값으로
비교하는 오프라인 검증 스크립트. ROS/rclpy 불필요, 순수 계산.
"""
import math

MODEL_DT = 0.05
TIME_STEPS = 30
HORIZON_S = MODEL_DT * TIME_STEPS
TEMPERATURE = 0.25
TRACK_WIDTH = 0.4904


def gen_path(turn_deg, turn_length_m, total_length_m, n_points=200):
    """publish_synthetic_left_turn.py와 동일 로직 -- (s, x, y, yaw) 리스트."""
    turn_sign = 1.0 if turn_deg >= 0 else -1.0
    theta_max = math.radians(abs(turn_deg))
    turn_radius = turn_length_m / theta_max
    x_end = turn_radius * math.sin(theta_max)
    y_end = turn_sign * turn_radius * (1.0 - math.cos(theta_max))

    pts = []
    for i in range(n_points):
        s = total_length_m * i / (n_points - 1)
        if s <= turn_length_m:
            theta = theta_max * (s / turn_length_m)
            x = turn_radius * math.sin(theta)
            y = turn_sign * turn_radius * (1.0 - math.cos(theta))
        else:
            extra = s - turn_length_m
            x = x_end + extra * math.cos(theta_max)
            y = y_end + turn_sign * extra * math.sin(theta_max)
        pts.append((s, x, y))
    return pts


def path_point_at_s(path, s):
    s = max(0.0, min(path[-1][0], s))
    for i in range(1, len(path)):
        if path[i][0] >= s:
            s0, x0, y0 = path[i - 1]
            s1, x1, y1 = path[i]
            t = 0.0 if s1 == s0 else (s - s0) / (s1 - s0)
            return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
    return path[-1][1], path[-1][2]


def nearest_path_s(path, x, y):
    """실제 findClosestPathPt는 누적거리 기반 근사매칭인데, 여기선 검증
    목적상 진짜 최근접점(더 관대한 쪽)으로 계산 -- 실제 코드보다 critic에
    유리하게 잡는 셈이라, 이걸로도 곡선이 안 이기면 더 확실한 신호."""
    best_s, best_d = 0.0, float("inf")
    for s, px, py in path:
        d = math.hypot(px - x, py - y)
        if d < best_d:
            best_d, best_s = d, s
    return best_s, best_d


def simulate_candidate(is_curve, robot_s0, vx, turn_deg, turn_length_m, path):
    """robot_s0 지점에서 출발해 horizon 동안 진행하는 후보 궤적 (x,y,yaw,wz) 시뮬레이션."""
    turn_sign = 1.0 if turn_deg >= 0 else -1.0
    theta_max = math.radians(abs(turn_deg))
    remaining_turn = max(0.0, turn_length_m - robot_s0)

    x, y, yaw = path_point_at_s(path, robot_s0) + (0.0,)
    # 시작 헤딩: 곡선 후보는 경로의 그 지점 접선 방향에서 출발(이미 정렬된 상태 가정 대신,
    # 시작은 둘 다 로봇의 "현재" 헤딩=0(직진 관성)에서 출발 -- 더 현실적이고 curve에 불리한 조건)
    x0, y0 = path_point_at_s(path, robot_s0)
    x, y, yaw = x0, y0, 0.0

    traj = []
    wz_hist = []
    for t in range(TIME_STEPS):
        if is_curve:
            # 곡선 후보: 아직 남은 회전(remaining_turn)이 있으면 그만큼 회전, 이후 직진
            dist_into_turn = min(vx * MODEL_DT, remaining_turn)
            if dist_into_turn > 1e-9 and remaining_turn > 1e-9:
                wz = turn_sign * theta_max * (vx / turn_length_m)
                remaining_turn -= dist_into_turn
            else:
                wz = 0.0
        else:
            wz = 0.0

        yaw += wz * MODEL_DT
        x += vx * math.cos(yaw) * MODEL_DT
        y += vx * math.sin(yaw) * MODEL_DT
        traj.append((x, y, yaw))
        wz_hist.append(wz)
    return traj, wz_hist


def path_align_cost(traj, path, point_step, threshold_to_consider, robot_xy, goal_xy):
    dist_to_goal = math.hypot(robot_xy[0] - goal_xy[0], robot_xy[1] - goal_xy[1])
    if dist_to_goal < threshold_to_consider:
        return 0.0, False  # critic 꺼짐
    dists = []
    for i in range(point_step, TIME_STEPS, point_step):
        x, y, _ = traj[i]
        _, d = nearest_path_s(path, x, y)
        dists.append(d)
    mean_d = sum(dists) / len(dists) if dists else 0.0
    return mean_d, True


def path_follow_cost(traj, path, offset_from_furthest_s, threshold_to_consider, robot_xy, goal_xy):
    dist_to_goal = math.hypot(robot_xy[0] - goal_xy[0], robot_xy[1] - goal_xy[1])
    if dist_to_goal < threshold_to_consider:
        return 0.0, False
    tx, ty = path_point_at_s(path, offset_from_furthest_s)
    ex, ey, _ = traj[-1]
    return math.hypot(ex - tx, ey - ty), True


def path_angle_cost(traj, path, lookahead_s, threshold_to_consider, robot_xy, goal_xy):
    """path_angle_critic.cpp:74-97 재현: 각 타임스텝에서 '후보 자신의 yaw'와
    '그 지점 -> lookahead 목표점 방향(bearing)' 사이 각도 오차를 평균."""
    dist_to_goal = math.hypot(robot_xy[0] - goal_xy[0], robot_xy[1] - goal_xy[1])
    if dist_to_goal < threshold_to_consider:
        return 0.0, False
    tx, ty = path_point_at_s(path, lookahead_s)
    errs = []
    for x, y, yaw in traj:
        bearing = math.atan2(ty - y, tx - x)
        err = abs(math.atan2(math.sin(yaw - bearing), math.cos(yaw - bearing)))
        errs.append(err)
    return sum(errs) / len(errs), True


def twirling_cost(wz_hist):
    return sum(abs(w) for w in wz_hist) / len(wz_hist)


def skid_cost(vx, wz_hist):
    slips = [abs((vx + w * TRACK_WIDTH / 2.0) - (vx - w * TRACK_WIDTH / 2.0)) for w in wz_hist]
    return sum(slips) / len(slips)


def run_scenario(label, turn_deg, turn_length_m, total_length_m, robot_progress_s, vx,
                  align_weight, align_step, align_threshold,
                  follow_weight, follow_threshold,
                  twirl_weight, skid_weight,
                  angle_weight=0.0, angle_threshold=0.15):
    path = gen_path(turn_deg, turn_length_m, total_length_m)
    robot_xy = path_point_at_s(path, robot_progress_s)
    goal_xy = (path[-1][1], path[-1][2])
    dist_to_goal = math.hypot(robot_xy[0] - goal_xy[0], robot_xy[1] - goal_xy[1])
    lookahead_s = min(robot_progress_s + 0.2, path[-1][0])

    results = {}
    for name, is_curve in (("STRAIGHT", False), ("CURVE", True)):
        traj, wz_hist = simulate_candidate(is_curve, robot_progress_s, vx, turn_deg, turn_length_m, path)
        align_d, align_on = path_align_cost(
            traj, path, align_step, align_threshold, robot_xy, goal_xy)
        follow_d, follow_on = path_follow_cost(
            traj, path, min(robot_progress_s + 1.4 * 6, path[-1][0]), follow_threshold, robot_xy, goal_xy)
        angle_e, angle_on = path_angle_cost(
            traj, path, lookahead_s, angle_threshold, robot_xy, goal_xy)
        twirl = twirling_cost(wz_hist)
        skid = skid_cost(vx, wz_hist)

        cost = (align_d * align_weight) + (follow_d * follow_weight) + \
               (angle_e * angle_weight) + \
               (twirl * twirl_weight) + (skid * skid_weight)
        results[name] = dict(cost=cost, align_d=align_d, align_on=align_on,
                              follow_d=follow_d, follow_on=follow_on,
                              angle_e=angle_e, angle_on=angle_on, twirl=twirl, skid=skid)

    costs = [results["STRAIGHT"]["cost"], results["CURVE"]["cost"]]
    min_c = min(costs)
    exps = [math.exp(-1.0 / TEMPERATURE * (c - min_c)) for c in costs]
    total = sum(exps)
    softmax = [e / total for e in exps]

    print(f"\n=== {label} (robot_progress_s={robot_progress_s:.2f}m, dist_to_goal={dist_to_goal:.3f}m) ===")
    for i, name in enumerate(("STRAIGHT", "CURVE")):
        r = results[name]
        print(f"  {name:8s}: align_on={r['align_on']!s:5} align_d={r['align_d']:.4f}  "
              f"follow_on={r['follow_on']!s:5} follow_d={r['follow_d']:.4f}  "
              f"angle_on={r['angle_on']!s:5} angle_e={r['angle_e']:.4f}  "
              f"twirl={r['twirl']:.4f}  skid={r['skid']:.4f}  "
              f"TOTAL_COST={r['cost']:.4f}  softmax_weight={softmax[i]:.4f}")
    winner = "CURVE" if softmax[1] > softmax[0] else "STRAIGHT"
    print(f"  -> softmax 승자: {winner} (curve 비중 {softmax[1]*100:.1f}%)")
    return softmax[1]


# 실제 테스트에 쓰인 값: turn_deg=45, turn_length=0.15, 짧은 실경로 가정 0.7m, vx=0.15
TURN_DEG = 45.0
TURN_LEN = 0.15
PATH_LEN = 0.7
VX = 0.15

print("#" * 70)
print("# 최초 설정값 (align=19/step4/thr0.5, follow_thr=1.4(미설정), twirl=10, skid=10)")
print("#" * 70)
for s0 in (0.0, 0.15, 0.3, 0.45):
    run_scenario(f"ORIG s0={s0}", TURN_DEG, TURN_LEN, PATH_LEN, s0, VX,
                 align_weight=19.0, align_step=4, align_threshold=0.5,
                 follow_weight=5.0, follow_threshold=1.4,
                 twirl_weight=10.0, skid_weight=10.0,
                 angle_weight=0.0)  # PathAngleCritic threshold 미설정이던 시절

print()
print("#" * 70)
print("# 2026-08-19 1차 수정 (align=28/step2/thr0.15, twirl=10 그대로) -- 이때는 아직 부족했음")
print("#" * 70)
for s0 in (0.0, 0.15, 0.3, 0.45):
    run_scenario(f"1st-fix s0={s0}", TURN_DEG, TURN_LEN, PATH_LEN, s0, VX,
                 align_weight=28.0, align_step=2, align_threshold=0.15,
                 follow_weight=5.0, follow_threshold=0.15,
                 twirl_weight=10.0, skid_weight=10.0,
                 angle_weight=2.0, angle_threshold=0.15)

print()
print("#" * 70)
print("# 최근 커밋(09571d8) 기준 실제 값: align=28/step2/thr0.15,")
print("# follow_thr=0.1(주석엔 0.15인데 실값은 0.1 -- 불일치 확인됨),")
print("# angle_weight=2.0/thr=0.15, twirl=1.0, skid=2.0")
print("#" * 70)
for s0 in (0.0, 0.15, 0.3, 0.45):
    run_scenario(f"HEAD s0={s0}", TURN_DEG, TURN_LEN, PATH_LEN, s0, VX,
                 align_weight=28.0, align_step=2, align_threshold=0.15,
                 follow_weight=5.0, follow_threshold=0.1,
                 twirl_weight=1.0, skid_weight=2.0,
                 angle_weight=2.0, angle_threshold=0.15)

print()
print("#" * 70)
print("# 같은 HEAD 설정, 완만한 회전(turn_length=0.4)으로도 교차검증")
print("#" * 70)
for s0 in (0.0, 0.1, 0.2):
    run_scenario(f"HEAD-gentle s0={s0}", TURN_DEG, 0.4, PATH_LEN, s0, VX,
                 align_weight=28.0, align_step=2, align_threshold=0.15,
                 follow_weight=5.0, follow_threshold=0.1,
                 twirl_weight=1.0, skid_weight=2.0,
                 angle_weight=2.0, angle_threshold=0.15)

# [2026-08-23 추가] HEAD 이후 twirl_weight가 문서화 없이 1.0 -> 7.0까지
# 다시 올라간 채 방치되면서, 여기서 잡아냈던 "곡선 관통" 버그가 재발했었음
# (synthetic_track_loop_path.py 실주행/오프라인 벤치에서 재관찰 -> 원인
# 추적). 앞으로 TwirlingCritic/SkidCritic/PathAlignCritic 등을 건드릴 때마다
# 이 블록의 CURRENT_* 값을 our_mppi_params.yaml과 수동으로 맞춰서 이
# 스크립트를 다시 돌려볼 것 -- 자동으로 값을 읽어오진 않으므로 동기화는
# 사람이 챙겨야 한다.
CURRENT_TWIRL = 2.5  # our_mppi_params.yaml / mppi_params_debug.yaml TwirlingCritic.cost_weight
CURRENT_SKID = 2.0   # 〃 SkidCritic.cost_weight

print()
print("#" * 70)
print(f"# 현재 라이브 값 회귀 체크: twirl={CURRENT_TWIRL}, skid={CURRENT_SKID}"
      " (yaml과 반드시 일치시킬 것)")
print("#" * 70)
_regression = False
for label, turn_len, s0_list in (("sharp", TURN_LEN, (0.0, 0.15, 0.3, 0.45)),
                                  ("gentle", 0.4, (0.0, 0.1, 0.2))):
    for s0 in s0_list:
        curve_share = run_scenario(
            f"CURRENT-{label} s0={s0}", TURN_DEG, turn_len, PATH_LEN, s0, VX,
            align_weight=28.0, align_step=2, align_threshold=0.15,
            follow_weight=5.0, follow_threshold=0.1,
            twirl_weight=CURRENT_TWIRL, skid_weight=CURRENT_SKID,
            angle_weight=2.0, angle_threshold=0.15)
        if s0 == 0.0 and curve_share < 0.5:
            _regression = True
if _regression:
    print("\n!!! 회귀 경고: s0=0.0(회전 시작 시점)에서 CURVE가 STRAIGHT에 졌음 -- "
          "곡선 관통 증상 재발 가능성. twirl/skid/align 균형을 다시 볼 것. !!!")
else:
    print("\nOK: s0=0.0 기준 CURVE가 이김 (곡선 관통 회귀 없음)")
