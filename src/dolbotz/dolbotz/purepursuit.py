"""
purepursuit_node

Pure Pursuit 기반 경로추종 컨트롤러 -- Nav2 MPPI(controller_server)가 하던
"/path -> 모터 명령" 역할을 대체한다. planner_server/bt_navigator/
path_relay_node/controller_server(FollowPath 액션)를 전혀 거치지 않고,
/path(인지팀 최종 경로, slope_decision.py가 15Hz로 발행, camera_link 프레임의
body 좌표 x=전방/y=좌측)를 직접 구독해서 정적 TF(base_link<-camera_link,
reduced_odom_bringup.launch.py가 CAD 실측값으로 쏨)로 base_link 좌표로 옮긴 뒤, 매 프레임
그 자리에서 pure pursuit으로 조향을 계산한다. 로봇은 항상 base_link 원점이라
EKF(/odometry/filtered)나 전역(odom) 위치추정이 필요 없다 -- 경로 자체가 매
프레임 로봇 기준으로 새로 갱신되기 때문 (flat_drive.py/gradient_map.py도
동일한 상대좌표 규약을 씀).

산출된 v(선속도)/w(각속도)는 rmd_x8_driver_node._skid_steer_inverse() +
_send_speed_command()와 동일한 공식으로 좌우 바퀴 dps로 바꿔서, can_driver_node
(can_driver 패키지, manual.launch.py가 조종 모드에서 쓰는 그 노드)가 그대로
구독하는 /motor_speed_cmd(std_msgs/Float32MultiArray, [left_dps, right_dps])로
발행한다 -- rmd_x8_driver_node/controller_server/joy_mux_node/current_ramp_node
체인은 전혀 거치지 않는, can_driver_node만 재사용하는 독립된 새 경로다.

구독:
    /path (nav_msgs/Path) -- slope_decision.py가 발행하는 최종 경로.
                              frame_id는 보통 'camera_link', 좌표는 body
                              규약(x=전방, y=좌측). force_mode가 무엇이든
                              (flat_drive/gradient_map 어느 쪽이든) 최종
                              선택된 경로가 이 토픽 하나로 나온다.

발행:
    /motor_speed_cmd (std_msgs/Float32MultiArray) -- [left_dps, right_dps].
                              can_driver_node가 그대로 구독. can_driver_node
                              의 cmd_timeout_sec(기본 0.3s) 워치독에 안 걸리게
                              control_rate_hz(기본 20Hz)로 항상 재발행한다
                              (정지 상태에서도 [0, 0]을 계속 보냄).

TF:
    base_frame(기본 'base_link') <- 경로의 header.frame_id 를 경로의
    header.stamp 시점으로 조회해서 각 포즈를 변환한다 (path_relay_node.cpp의
    TF 변환 로직과 동일한 방식). 조회 실패/경로 수신 자체가 끊기면
    path_timeout_sec 안전망이 정지 명령을 낸다.

파라미터 (실측/현장 튜닝 전 placeholder 다수):
    path_topic                str    '/path'
    base_frame                str    'base_link'
    tf_lookup_timeout_sec     float  0.1    (path_relay_node와 동일 기본값)
    path_timeout_sec          float  1.0    (path_relay_node와 동일 기본값 --
                                             이보다 오래 /path가 안 오면 정지)
    lookahead_distance_m      float  0.8    실측 전 placeholder, 현장 튜닝 필요
    target_linear_speed_m_s   float  0.3    실측 전 placeholder, 현장 튜닝 필요
    goal_tolerance_m          float  0.3    경로 마지막 점까지 이 거리 이내면
                                            도착으로 보고 정지
    track_width_m              float 0.50   rmd_x8_driver_node의
                                            effective_track_width_m 기본값과
                                            동일 (실물 캘리브레이션 필요, 그쪽
                                            control_guide 3.1 참고)
    wheel_radius_m              float 0.1125 rmd_x8_driver_node와 동일
    left_motor_sign/right_motor_sign
                                 float 1.0/-1.0  manual_joy_control_node와
                                            동일 극성 (같은 can_driver_node를
                                            그대로 쓰므로 -- 배선이 바뀌면
                                            거기와 같이 맞춰서 뒤집을 것)
    max_wheel_speed_dps          float 800.0  manual_joy_control_node의
                                            max_speed_dps 기본값과 동일
    control_rate_hz               float 20.0
    cmd_topic                     str  '/motor_speed_cmd'

    use_distance_scaled_speed     bool  False  [2026-08-30 신규] 기본 꺼짐(다른
                                            미션 영향 없음) -- escort_follow
                                            (5구간, 선도로봇 추종) 전용으로
                                            추가. False(기본)면 기존과 동일하게
                                            target_linear_speed_m_s 고정속도.
                                            True면 목표점까지 거리(look_dist)로
                                            [distance_band_min_m,
                                            distance_band_max_m] 밴드를
                                            유지한다: look_dist가 밴드보다
                                            멀면 v = clamp(distance_speed_gain *
                                            (look_dist - distance_band_max_m),
                                            distance_speed_min_m_s,
                                            distance_speed_max_m_s)로 전진,
                                            밴드보다 가까우면 v = clamp(
                                            distance_speed_gain * (look_dist -
                                            distance_band_min_m),
                                            -distance_speed_reverse_max_m_s, 0)
                                            로 후진, 밴드 안이면 v=0(정지).
                                            [2026-08-30, 재조정] 원래는
                                            goal_tolerance_m 하나만 setpoint로
                                            쓰고 후진 없이 "가까우면 그냥
                                            정지"만 했는데, 그러면 1.5~2.0m
                                            규정 밴드 하한(1.5m)을 밑도는 걸
                                            막을 방법이 없어서(선도가 우리
                                            쪽으로 다가오는 경우 등) 밴드+후진
                                            방식으로 바꿈. look_dist가
                                            escort_follow처럼 2점(로봇 원점+
                                            추종대상) 경로에서만 "선도 로봇까지의
                                            실제 거리"와 일치한다 -- 여러 점으로
                                            이루어진 긴 경로(flat_drive 등)에
                                            켜면 look_dist가 lookahead 반경 밖
                                            첫 경로점까지 거리일 뿐이라 의도한
                                            의미가 아니게 되므로 켜지 말 것.
    distance_speed_gain           float 2.0    [2026-08-30, 대회 규정 반영] 위
                                            P제어 게인[m/s per m]. P제어는
                                            선도로봇이 등속으로 계속 움직이면
                                            distance_band_max_m에서 정확히
                                            안 멈추고 그보다 (선도속도/gain)
                                            만큼 더 먼 거리에서 정상상태에
                                            도달한다(로봇 속도=선도속도가
                                            되는 평형점). [2026-08-30, 재조정]
                                            대회 규정 목표 간격이 1.5~2.0m로
                                            좁혀지면서 distance_band_max_m도
                                            2.0(=규정 상한 그대로)이 됐는데,
                                            그러면 이 평형점 산식상 선도로봇이
                                            조금이라도 움직이는 동안엔 실제
                                            거리가 항상 2.0m를 살짝 넘는
                                            지점(2.0 + 선도속도/gain)에서
                                            정상상태에 도달한다 -- 예전처럼
                                            "정상상태 오차를 허용오차 안에
                                            넣는" 여유가 더 이상 없다(선도가
                                            멈춰있을 때만 정확히 2.0m로
                                            수렴). gain을 올리면 이 초과분은
                                            줄지만 오버슈트/진동 위험이
                                            커진다 -- 실기에서 "선도 이동 중
                                            2.0m를 계속 넘는다"가 문제되면
                                            gain을 올리거나 distance_band_max_m
                                            을 규정 상한보다 살짝 낮게(예:
                                            1.9) 잡는 것도 방법. 실측/현장
                                            튜닝 필요.
    distance_speed_min_m_s        float 0.0    [2026-08-30 신규] 위 P제어 속도
                                            하한 -- 실측 전 placeholder.
    distance_speed_max_m_s        float 1.8    [2026-08-30, 대회 규정 반영] 위
                                            P제어 속도 상한("가림막 통과 후
                                            따라잡기"에 해당하는 캡). 선도로봇
                                            최고속도(로봇개 1.0m/s)보다 반드시
                                            커야 한다 -- 안 그러면 정상상태
                                            추종 자체가 원리적으로 불가능
                                            (로봇이 선도로봇보다 항상 느려서
                                            간격이 무한정 벌어짐). 이전
                                            placeholder(0.8)는 1.0보다 낮아서
                                            이 조건을 어겼음 -- dog_follow_node.cpp
                                            의 max_vx_catchup(2.5)보다는
                                            보수적으로 잡되(공용 노드라 안전
                                            우선) 1.0m/s를 확실히 넘도록 1.8로
                                            상향. 실측/현장 튜닝 필요.
    distance_band_min_m           float 1.5    [2026-08-30 신규] 위 밴드 하한 --
                                            대회 규정 목표 간격 1.5~2.0m의
                                            하한 그대로. 이보다 가까우면
                                            distance_speed_reverse_max_m_s로
                                            캡된 속도로 후진한다.
    distance_band_max_m           float 2.0    [2026-08-30 신규, 2026-08-30
                                            재조정: 2.5->2.0] 위 밴드 상한 --
                                            대회 규정 목표 간격 상한 그대로.
                                            이보다 멀면 distance_speed_max_m_s
                                            로 캡된 속도로 전진(따라잡기)한다.
                                            [주의] 이 값이 곧 규정 상한이라
                                            여유가 없음 -- distance_speed_gain
                                            항목의 정상상태 오차 설명 참고.
    distance_speed_reverse_max_m_s float 0.3   [2026-08-30 신규] 밴드 하한보다
                                            가까울 때 후진 속도 상한.
                                            [안전 주의] 드라이브 카메라는
                                            전방만 보고 후방 장애물 감지가
                                            전혀 없다 -- 그래서 전진 캡
                                            (distance_speed_max_m_s=1.8)보다
                                            훨씬 보수적으로 잡음. 실측 전
                                            placeholder, 실기에서 후방 여유
                                            공간 확인 후 조정할 것.
    use_bearing_steering          bool  False  [2026-08-30 신규] escort_follow
                                            전용, 기본 꺼짐(다른 미션 영향
                                            없음). 로봇개가 카메라 화면
                                            외곽에 잡혔을 때 표준 pure
                                            pursuit curvature(2y/L^2)로
                                            조향이 거의 안 나가는 문제가
                                            실기에서 확인됨 -- 화면 외곽은
                                            depth 오차가 커지는 구간이라
                                            (RealSense 흔한 현상) L(목표까지
                                            거리)이 실제보다 부풀려지면,
                                            각도(alpha)가 커도 curvature는
                                            반대로 작아지기 때문. True면
                                            curvature를 2y/L^2 대신
                                            bearing_steering_gain*atan2(y,x)/v
                                            로 계산 -- L 크기와 무관하게
                                            각도만으로 조향을 정해서, 화면
                                            중앙에 오도록 확실히 회전한다.
                                            _compute_pure_pursuit() 참고.
    bearing_steering_gain         float 3.0    [2026-08-30 신규] 위
                                            use_bearing_steering=True일 때
                                            각도 오차(rad) 대비 목표
                                            각속도(rad/s) 게인. [2026-08-30,
                                            재조정] 제자리 회전이 너무
                                            굼뜨다는 실기 피드백으로 1.5에서
                                            2배(3.0)로 올림. 실측 전
                                            placeholder -- 너무 크면
                                            제자리에서 좌우로 진동(오버슈트)
                                            할 수 있으니 실기에서 낮춰
                                            조정할 것.
    bearing_inplace_threshold_rad float 0.05   [2026-08-30, 사용자 결정,
                                            2026-09-03 제거] 원래
                                            use_bearing_steering=True일 때
                                            목표각(alpha)이 이 값보다 크면
                                            전진하며 도는 대신 제자리
                                            회전(v=0, 회전만)으로 먼저
                                            정면에 가깝게 맞추는 피벗 턴
                                            분기에 쓰였다 -- 회전하는 동안
                                            전진이 아예 멎어서 추종 거리가
                                            계속 벌어지는 문제로,
                                            slope_traverse_node.cpp의
                                            computeBoostedVx() 기반 차동
                                            조향(use_outer_wheel_boost)으로
                                            대체하며 이 분기 자체를
                                            _compute_pure_pursuit()에서
                                            없앴다(사용자 지시). 지금은 큰
                                            각도도 v_target을 0으로 죽이지
                                            않고 바깥쪽 바퀴 부스트로 돌며
                                            전진한다. 이 파라미터는 더 이상
                                            안 읽히지만 하위 호환/향후 재사용
                                            대비로 declare_parameter는
                                            남겨뒀다.
    bearing_deadband_lateral_m    float 0.8    [2026-08-31, 사용자 결정] 위
                                            use_bearing_steering=True일 때,
                                            좌우 오프셋(y, base_link 기준 --
                                            카메라 roll이 거의 0이라 카메라
                                            광학 프레임 X와 부호만 반대일 뿐
                                            사실상 같은 값)의 절댓값이 이
                                            범위 안이면 각도가 얼마든 조향을
                                            아예 안 한다(w=0) -- 예전엔 각도
                                            (bearing_inplace_threshold_rad)
                                            만 보고 계속 미세 조향해서 직선
                                            정렬에 집착했는데, "좌우로 이
                                            정도 벗어난 건 이미 충분히
                                            정면"으로 보는 거리 기준
                                            데드밴드로 바꿈. bearing_inplace_
                                            threshold_rad보다 먼저 검사됨 --
                                            가까운 거리에서 각도만 큰
                                            경우(예: dist=0.5m, alpha=40°
                                            라도 y=0.3m면 데드밴드 안)도
                                            걸러진다. 실측 전 placeholder.
    use_single_wheel_pivot        bool  False  [2026-08-30, 사용자 결정]
                                            제자리 회전(위 w_override 활성
                                            구간)만 적용되는 옵션 --
                                            기본(False)은 양쪽 바퀴가 반대로
                                            돌아 로봇 중심을 축으로 도는
                                            기존 스큐-스티어 제자리 회전.
                                            True면 한쪽 바퀴는 세워두고
                                            반대쪽만 굴려서(전체
                                            track_width_m 반경) 그 세워둔
                                            바퀴를 축으로 피벗 턴한다 --
                                            _single_wheel_pivot_to_dps() 참고.
                                            전진/후진하며 도는 구간에는
                                            영향 없음(그 구간은 항상 기존
                                            양쪽바퀴 스큐-스티어).
    use_lost_left_turn_recovery   bool  False  [2026-09-04 신규, 사용자 요청]
                                            escort_follow 전용, 기본 꺼짐.
                                            /path가 비어있는 프레임을 받으면
                                            (=escort_follow의 LOST, 위
                                            _on_path()/_control_loop() 참고)
                                            원래는 항상 즉시 정지였는데,
                                            "로봇개가 왼쪽 가장자리로 빠지며
                                            놓친 경우"만 구분해서 대응을
                                            바꾼다. [2026-09-05 수정, 사용자
                                            요청] 트리거 조건을 OR 두 개로
                                            확장 -- ① 놓치기 직전 경로 자체가
                                            좌측으로 휘어 있었는지(마지막
                                            목표점 각도 alpha=atan2(y,x) > 0,
                                            self._path_xy가 비기 직전 값 기준) ②
                                            직전 실제 발행 각속도(self._last_w)
                                            가 lost_left_turn_w_threshold_rad_s
                                            이상이었는지(기존 조건 그대로
                                            유지). 둘 중 하나만 참이어도
                                            "왼쪽으로 빠져나감"으로 보고
                                            lost_left_turn_duration_sec 동안
                                            제자리에서 좌회전하며 재포착을
                                            시도한다 -- 회전 목표각속도는
                                            lost_left_turn_bearing_gain*alpha와
                                            lost_left_turn_w_rad_s(하한) 중 큰
                                            값(아래 두 파라미터 참고). 둘 다
                                            거짓이면(직진하다 놓친 경우) 기존
                                            동작과 동일하게 즉시 정지.
                                            [2026-09-05 버그 수정] 이 재포착
                                            창이 열려 있는 동안은 path_timeout_sec
                                            (/path 발행 자체가 끊긴 시간) 초과
                                            여부와 무관하게 무조건 최우선으로
                                            돈다(_control_loop() 맨 앞 참고) --
                                            인지팀이 빈 /path를 1회만 보내고
                                            재발행을 안 하면 age_s 타임아웃과
                                            이 창이 거의 동시에 만료돼서 좌회전이
                                            중간에 끊기는 문제가 있었음.
    lost_left_turn_w_threshold_rad_s
                                   float 0.1    use_lost_left_turn_recovery=
                                            True일 때, 놓치는 순간
                                            self._last_w가 이 값(rad/s,
                                            양수=좌회전) 이상이어야 "좌회전
                                            중 놓침"으로 본다 -- 노이즈로
                                            거의 0인 w까지 좌회전으로 오판해
                                            직진 중 가림막 상황에서도 도는 걸
                                            막기 위한 문턱값. 실측 전
                                            placeholder.
    lost_left_turn_w_rad_s        float 1.0    좌회전 재포착 중 목표
                                            각속도[rad/s]의 하한(floor) --
                                            아래 lost_left_turn_bearing_gain
                                            기반 계산값이 이보다 작으면 이
                                            값을 대신 쓴다. v=0(제자리) 상태로
                                            최종 목표 w를 향해
                                            max_angular_accel_rad_s2로 램프된다
                                            (_ramp_w() 재사용). 실측 전
                                            placeholder.
    lost_left_turn_bearing_gain   float 8.0    [2026-09-05 신규, 사용자 요청]
                                            놓치는 순간의 마지막 경로 목표점
                                            각도(alpha=atan2(y,x), 좌측=양수)에
                                            곱해서 재포착 회전 목표각속도를
                                            낸다 -- 로봇개가 화면 왼쪽 얼마나
                                            바깥까지 나가 있었는지에 비례해서
                                            더 세게 돈다(use_bearing_steering의
                                            bearing_steering_gain과 같은 공식,
                                            재포착 전용으로 훨씬 큰 게인을
                                            따로 둠). alpha<=0(왼쪽으로 안
                                            휘어있던 상태 -- 즉 last_w 조건만
                                            으로 트리거된 경우)이면 이 항은
                                            0 이하가 되므로 아래
                                            lost_left_turn_w_rad_s 하한이
                                            대신 적용된다(_on_path() 참고).
    lost_left_turn_duration_sec   float 1.0    좌회전 재포착을 시도하는
                                            최대 시간[초] -- 이 안에 /path가
                                            다시 채워져 재포착되면(_on_path()
                                            참고) 그 즉시 취소되고 정상
                                            추종으로 복귀한다. 시간을 넘기면
                                            기존과 동일하게 정지(v=0,w=0)로
                                            내려간다.
    max_linear_accel_mps2         float 1.0    [2026-08-30 신규] v(선속도)가
                                            커지는(가속하는) 방향의 변화율
                                            상한[m/s^2] -- _compute_pure_pursuit()가
                                            매 틱 계산하는 목표 v(특히
                                            use_distance_scaled_speed=True일
                                            때, escort_follow 재포착 직후처럼
                                            거리가 크게 벌어져 있다가 갑자기
                                            좁혀진 경우)로 바로 점프하지 않고
                                            이 가속도로 점진적으로 다가가게
                                            한다. 이 노드가 나가는
                                            /motor_speed_cmd는 can_driver_node로
                                            직접 가서 robot_bringup의
                                            current_ramp_node 같은 램프 레이어를
                                            안 거치므로, v 변화 자체를 여기서
                                            제한해야 한다. 조향 각속도 w는
                                            curvature(=w/v, 회전반경 결정)를
                                            유지한 채 이 램프된 v로 재계산하므로
                                            추종 경로 자체는 안 바뀌고 속도만
                                            부드러워진다. 경로 소실/타임아웃
                                            정지는 이 램프를 안 거치고 즉시
                                            0으로(안전 우선, _control_loop()
                                            참고) -- 실측 전 placeholder.
                                            [2026-08-31, 가감속 분리] 원래는
                                            가속/감속에 같은 값을 썼는데,
                                            escort_follow에서 "정지 상태에서
                                            로봇개를 바로 못 따라간다"는 문제가
                                            보고됨 -- 이 값(1.0) 하나로 0->
                                            distance_speed_max_m_s(1.8m/s)까지
                                            올라가는 데만 1.8초가 걸려서,
                                            추종 시작/재포착 직후처럼 정지
                                            상태에서 급하게 붙어야 하는 상황의
                                            지연으로 이어졌다. 감속(아래
                                            max_linear_decel_mps2)은 안전/전류
                                            보호 목적이라 그대로 보수적으로
                                            두고, 가속만 따로 올릴 수 있게
                                            분리했다(_ramp_v() 참고) --
                                            escort_follow은 launch에서 이
                                            값만 올려 쓰고(예: 3.0), 다른
                                            미션(flat_drive 등)은 기본값
                                            그대로라 영향 없음. 실측/현장
                                            튜닝 필요.
    max_linear_decel_mps2         float 1.0    [2026-08-31 신규] v(선속도)가
                                            작아지는(감속하는, 0 또는 반대
                                            부호 쪽으로 다가가는) 방향의
                                            변화율 상한[m/s^2] -- 위
                                            max_linear_accel_mps2와 분리된
                                            값(_ramp_v() 참고). 정지/속도를
                                            낮추는 쪽은 계속 부드럽게(전류
                                            스파이크 방지) 유지하고 싶어서
                                            기본값은 기존 max_linear_accel_mps2와
                                            동일한 1.0을 유지 -- 필요하면
                                            독립적으로 튜닝할 것. 경로 소실/
                                            타임아웃 정지는 이 램프도 안
                                            거치고 즉시 0(안전 우선,
                                            _control_loop() 참고).
    max_linear_accel_while_turning_mps2
                                  float 0.0    [2026-09-01 신규] 회전
                                            중(w_override 활성이든 curvature
                                            != 0이든, _control_loop()의
                                            is_turning 참고) 가속 방향
                                            변화율 상한[m/s^2] -- 위
                                            max_linear_accel_mps2 대신 이
                                            값을 쓴다. 기본 0.0 = 회전 중엔
                                            가속을 아예 안 함(정지 상태에서
                                            추종을 시작해 로봇개를 따라잡을
                                            땐 직선 구간에서만 가속하고,
                                            방향을 트는 동안은 그 순간의
                                            속도를 넘어서지 않음) -- 회전과
                                            가속이 동시에 걸리면 바퀴 슬립이
                                            심해진다는 실차 피드백 대응.
                                            감속(max_linear_decel_mps2)은
                                            회전 여부와 무관하게 항상 적용
                                            (안전 우선). purepursuit.
                                            launch.py에 CLI 인자로는 안
                                            뺐다 -- config/
                                            purepursuit_params.yaml로만
                                            설정(실험 버전마다 이 값만
                                            바꾼 yaml을 스왑하는 용도).
    max_angular_accel_rad_s2      float 6.0    [2026-09-01 신규, 재도입] w(각속도)
                                            변화율 상한[rad/s^2] --
                                            _ramp_w() 참고. 위 v(선속도)만
                                            램프되고 w는 매 틱 목표치로
                                            바로 튀던 걸(w_override든
                                            v_ramped*curvature든) 완만하게
                                            바꾼다 -- "회전이 갑자기 확
                                            꺾인다"는 실차 피드백 대응.
                                            [주의] 각속도 "자체"의 최댓값이
                                            아니라 각속도가 "변하는
                                            속도"만 제한한다 -- 정상상태
                                            w의 크기는 여전히 게인*각도
                                            계산값과 바퀴 dps
                                            하드클램프(max_wheel_speed_dps)로
                                            정해진다. 가속/감속 구분 없이
                                            단일 상한(전류 보호가 아니라
                                            회전 자체를 부드럽게 만드는
                                            목적이라 방향 구분 불필요).
                                            경로 소실/타임아웃 정지는 이
                                            램프도 안 거치고 즉시
                                            0(_control_loop() 참고). 실측
                                            전 placeholder.
    use_outer_wheel_boost         bool  False  [2026-08-27 신규] 경사 주행용.
                                            기본(False)은 기존과 동일한 대칭
                                            스큐-스티어(v_left=v-w*halftrack,
                                            v_right=v+w*halftrack) -- 회전할수록
                                            안쪽 바퀴가 target_linear_speed_m_s
                                            밑으로 깎임. True면 _skid_steer_to_dps()
                                            에서 v 자체를 v+|w|*halftrack으로
                                            올려서 보내, 안쪽 바퀴는
                                            target_linear_speed_m_s에 그대로
                                            고정하고 바깥쪽만 부스트한다 --
                                            slope_traverse_node.cpp의
                                            computeBoostedVx()와 동일한 근거
                                            (매뉴얼 주행 실측: 한쪽 300dps/
                                            반대쪽 200dps 조합이 경사에서 잘
                                            됐다는 피드백). 경사 주행 시
                                            target_linear_speed_m_s를 그
                                            "안쪽" 목표(예: 0.4m/s, 약
                                            200dps)로 잡을 것.
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray

import tf2_geometry_msgs  # noqa: F401 -- PoseStamped 변환 등록 (do_transform_pose_stamped 사용)
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('purepursuit_node')

        # ---- 파라미터 ----
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_lookup_timeout_sec', 0.1)
        self.declare_parameter('path_timeout_sec', 1.0)

        self.declare_parameter('lookahead_distance_m', 0.5)
        self.declare_parameter('target_linear_speed_m_s', 0.3)
        self.declare_parameter('goal_tolerance_m', 0.3)

        self.declare_parameter('track_width_m', 0.50)
        self.declare_parameter('wheel_radius_m', 0.1125)
        self.declare_parameter('left_motor_sign', 1.0)
        self.declare_parameter('right_motor_sign', -1.0)
        self.declare_parameter('max_wheel_speed_dps', 800.0)

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_topic', '/motor_speed_cmd')

        # [2026-08-27 신규] 경사 주행용 -- 모듈 docstring 참고.
        self.declare_parameter('use_outer_wheel_boost', False)

        # [2026-08-30 신규, 2026-08-31 가감속 분리] v 변화율 상한 -- 모듈
        # docstring 참고.
        self.declare_parameter('max_linear_accel_mps2', 1.0)
        self.declare_parameter('max_linear_decel_mps2', 1.0)
        # [2026-09-01 신규] 회전 중(제자리 회전이든 곡선 추종이든) 가속
        # 상한 -- 회전+가속 동시 발생 시 바퀴 슬립이 심해진다는 실차 피드백
        # 대응. 기본 0.0(회전 중엔 가속 완전 금지, 감속/등속만 허용) --
        # purepursuit.launch.py에 launch 인자로는 안 뺐다(config/
        # purepursuit_params.yaml로만 설정 -- 실험 버전마다 이 값만 바꾼
        # yaml을 스왑하는 용도). _ramp_v() 참고.
        self.declare_parameter('max_linear_accel_while_turning_mps2', 0.0)
        # [2026-09-01 신규, 재도입] w(각속도) 변화율 상한 -- v(선속도)만
        # 램프되고 w는 매 틱 목표치로 바로 튀던 걸(w_override든
        # v_ramped*curvature든) 완만하게 만든다. "회전이 갑자기 확 꺾인다"는
        # 실차 피드백 대응. 각속도 "자체"의 최댓값을 제한하는 게 아니라
        # 각속도가 "변하는 속도"(각가속도)만 제한한다는 점에 주의 --
        # 최종적으로 얼마나 빨리 도는지는 여전히 게인*각도 계산값과
        # 바퀴 dps 하드클램프(max_wheel_speed_dps)로 정해진다. _ramp_w() 참고.
        self.declare_parameter('max_angular_accel_rad_s2', 6.0)

        # [2026-08-30 신규] escort_follow 전용 거리 비례 속도 -- 기본 꺼짐,
        # 모듈 docstring 참고.
        self.declare_parameter('use_distance_scaled_speed', False)
        self.declare_parameter('distance_speed_gain', 2.0)
        self.declare_parameter('distance_speed_min_m_s', 0.0)
        self.declare_parameter('distance_speed_max_m_s', 1.8)
        # [2026-08-30, 사용자 결정] 단일 setpoint(goal_tolerance_m) P제어 대신
        # 밴드[distance_band_min_m, distance_band_max_m]를 유지하는 방식으로
        # 변경 -- 대회 규정 목표 간격 1.5~2.0m(2026-08-30 재조정, 기존
        # 1.5~2.5m에서 상한 좁혀짐)를 그대로 밴드로 씀. 밴드보다
        # 멀면 전진(위 distance_speed_gain/min/max_m_s로 P제어, 기존과 동일),
        # 밴드보다 가까우면 "후진"으로 다시 밴드 안까지 물러난다(신규,
        # distance_speed_reverse_max_m_s로 캡). 밴드 안이면 정지(v=0).
        # [안전 주의] 드라이브 카메라는 전방만 보므로 후진은 후방 장애물을
        # 전혀 감지 못 한 채로 나간다 -- 그래서 후진 속도 캡을 전진 캡
        # (distance_speed_max_m_s=1.8)보다 훨씬 보수적으로 별도로 둔다.
        self.declare_parameter('distance_band_min_m', 1.5)
        self.declare_parameter('distance_band_max_m', 2.0)
        self.declare_parameter('distance_speed_reverse_max_m_s', 0.3)

        # [2026-08-30 신규] escort_follow 전용 -- 로봇개가 화면(카메라 시야)
        # 외곽에 잡혔을 때 표준 pure pursuit curvature(2y/L^2)로는 조향이
        # 거의 안 나가는 문제가 실기에서 확인됨(모듈 docstring 참고: 화면
        # 외곽은 depth 오차가 커지는 구간이라 L이 부풀려지면 각도가 커도
        # curvature는 반대로 작아짐). 기본 꺼짐(다른 미션 영향 없음).
        self.declare_parameter('use_bearing_steering', False)
        # [2026-08-30, 재조정] 1.5 -> 3.0으로 2배 (제자리 회전이 너무
        # 굼뜨다는 실기 피드백 -- w=gain*alpha라 dps도 그대로 2배가 됨).
        self.declare_parameter('bearing_steering_gain', 5.0)
        # [2026-08-31, 사용자 결정] 좌우 오프셋(y, base_link) 절댓값이 이
        # 범위 안이면 각도가 얼마든 조향을 아예 안 한다 -- "카메라 x좌표
        # 기준 -0.8~+0.8은 조향 안 하는 구간"으로 해달라는 요청. 예전엔
        # bearing_inplace_threshold_rad(각도 기준, 0.05rad) 하나만 있어서
        # 각도가 정확히 0에 가까워질 때까지 계속 미세 조향했는데, 그 대신
        # 좌우로 이만큼 벗어난 건 이미 충분히 정면으로 보는 거리 기준
        # 데드밴드. _compute_pure_pursuit() 참고.
        self.declare_parameter('bearing_deadband_lateral_m', 0.5)
        # [2026-08-30, 사용자 결정] 목표각(alpha)이 이 값보다 크면 전진하며
        # 도는 대신 제자리 회전(v=0, 회전만)으로 먼저 정면에 가깝게 맞춘다
        # -- use_bearing_steering=True일 때만 적용. _compute_pure_pursuit()
        # 참고. [2026-08-30, 재조정] "회전은 항상 제자리 회전으로" 요청으로
        # 기본값을 0.35(약 20°, 전진하며 어느 정도 도는 걸 허용)에서 0.05
        # (약 3°)로 낮춤 -- 이제 각도가 거의 0에 가깝지 않은 한(순수 직진/
        # 후진만 필요한 경우) 사실상 항상 제자리 회전부터 하고, 정면에 거의
        # 맞춰진 뒤에만 전진/후진한다. 0.0으로 완전히 없애지 않은 이유는
        # depth/각도 추정 노이즈로 alpha가 정확히 0이 되는 일이 거의 없어서,
        # 0으로 두면 정지 상태에서도 미세한 회전 명령이 계속 나가 흔들릴 수
        # 있기 때문(데드존 역할).
        # [2026-09-03, 사용자 지시로 제거] 위에서 설명한 제자리 회전 분기
        # 자체를 _compute_pure_pursuit()에서 없앴다(모듈 docstring 참고) --
        # 회전 중 전진이 멎어 추종 거리가 벌어지는 문제로,
        # use_outer_wheel_boost 차동 조향으로 대체함. 이 파라미터는 더 이상
        # 안 읽히지만 하위 호환용으로 declare_parameter는 남겨둠.
        self.declare_parameter('bearing_inplace_threshold_rad', 0.05)
        # [2026-08-30, 사용자 결정] 제자리 회전(w_override 활성 구간) 방식 --
        # 기본(False)은 양쪽 바퀴가 서로 반대로 돌아 로봇 중심을 축으로
        # 도는 기존 스큐-스티어 제자리 회전. True면 한쪽 바퀴는 세워두고
        # 반대쪽만 굴려서 그 세워둔 바퀴를 축으로 피벗 턴한다 --
        # _single_wheel_pivot_to_dps() 참고. 전진/후진하며 도는 구간(w가
        # curvature*v로 계산되는 경우)에는 영향 없음 -- 거기는 항상 기존
        # 양쪽바퀴 스큐-스티어 그대로.
        self.declare_parameter('use_single_wheel_pivot', False)

        # [2026-09-04 신규, 사용자 요청] escort_follow 전용, 기본 꺼짐 --
        # 모듈 docstring 참고. /path가 비면(LOST) 항상 즉시 정지하던 걸,
        # 놓치기 직전 좌회전 중이었을 때만 잠깐 좌회전을 유지하며 재포착을
        # 시도하도록 구분한다.
        self.declare_parameter('use_lost_left_turn_recovery', False)
        self.declare_parameter('lost_left_turn_w_threshold_rad_s', 0.1)
        self.declare_parameter('lost_left_turn_w_rad_s', 1.0)
        self.declare_parameter('lost_left_turn_bearing_gain', 8.0)
        self.declare_parameter('lost_left_turn_duration_sec', 1.0)

        p = self.get_parameter
        self.base_frame = p('base_frame').value
        self.tf_lookup_timeout_sec = float(p('tf_lookup_timeout_sec').value)
        self.path_timeout_sec = float(p('path_timeout_sec').value)

        self.lookahead_distance_m = float(p('lookahead_distance_m').value)
        self.target_linear_speed_m_s = float(p('target_linear_speed_m_s').value)
        self.goal_tolerance_m = float(p('goal_tolerance_m').value)

        self.track_width_m = float(p('track_width_m').value)
        self.wheel_radius_m = float(p('wheel_radius_m').value)
        self.left_motor_sign = float(p('left_motor_sign').value)
        self.right_motor_sign = float(p('right_motor_sign').value)
        self.max_wheel_speed_dps = float(p('max_wheel_speed_dps').value)
        self.use_outer_wheel_boost = bool(p('use_outer_wheel_boost').value)
        self.max_linear_accel_mps2 = float(p('max_linear_accel_mps2').value)
        self.max_linear_decel_mps2 = float(p('max_linear_decel_mps2').value)
        self.max_angular_accel_rad_s2 = float(p('max_angular_accel_rad_s2').value)
        self.max_linear_accel_while_turning_mps2 = float(
            p('max_linear_accel_while_turning_mps2').value)

        self.use_distance_scaled_speed = bool(p('use_distance_scaled_speed').value)
        self.distance_speed_gain = float(p('distance_speed_gain').value)
        self.distance_speed_min_m_s = float(p('distance_speed_min_m_s').value)
        self.distance_speed_max_m_s = float(p('distance_speed_max_m_s').value)
        self.distance_band_min_m = float(p('distance_band_min_m').value)
        self.distance_band_max_m = float(p('distance_band_max_m').value)
        self.distance_speed_reverse_max_m_s = float(p('distance_speed_reverse_max_m_s').value)

        self.use_bearing_steering = bool(p('use_bearing_steering').value)
        self.bearing_steering_gain = float(p('bearing_steering_gain').value)
        self.bearing_deadband_lateral_m = float(p('bearing_deadband_lateral_m').value)
        self.bearing_inplace_threshold_rad = float(p('bearing_inplace_threshold_rad').value)
        self.use_single_wheel_pivot = bool(p('use_single_wheel_pivot').value)

        self.use_lost_left_turn_recovery = bool(p('use_lost_left_turn_recovery').value)
        self.lost_left_turn_w_threshold_rad_s = float(p('lost_left_turn_w_threshold_rad_s').value)
        self.lost_left_turn_w_rad_s = float(p('lost_left_turn_w_rad_s').value)
        self.lost_left_turn_bearing_gain = float(p('lost_left_turn_bearing_gain').value)
        self.lost_left_turn_duration_sec = float(p('lost_left_turn_duration_sec').value)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 상태 ----
        self._path_xy = []           # base_frame 기준 [(x, y), ...], 근->원 순서 유지
        self._last_path_time = None  # None = /path 아직 한 번도 못 받음 (조기 정지 스팸 방지)
        # [2026-09-05 신규, 사용자 요청] "메시지 자체가 안 옴"과 "메시지는
        # 오는데 TF 변환만 계속 실패함"을 구분하기 위한 별도 타임스탬프 --
        # _on_path()가 맨 위에서 msg.poses/TF 성공 여부와 무관하게 매번
        # 갱신한다(_last_path_time은 TF 실패 시 안 갱신됨, _on_path() 참고).
        # _control_loop()가 두 값을 비교해서 어느 쪽인지 로그로 구분한다.
        self._last_msg_received_time = None
        self._goal_reached = False
        self._last_v = 0.0           # max_linear_accel_mps2 램프의 기준점(직전 틱 실제 발행 v)
        self._last_w = 0.0           # max_angular_accel_rad_s2 램프의 기준점(직전 틱 실제 발행 w)
        # [2026-09-04 신규] use_lost_left_turn_recovery=True일 때, 좌회전 중
        # 놓쳐서 블라인드 좌회전 재포착을 시도 중이면 그 종료 시각(rclpy Time),
        # 아니면 None. _on_path()가 세팅/취소하고 _control_loop()가 읽는다.
        self._lost_turn_until = None
        # [2026-09-05 신규] 위 재포착 구간에서 쓸 목표 각속도[rad/s] --
        # _on_path()가 트리거 시점에 lost_left_turn_bearing_gain*alpha와
        # lost_left_turn_w_rad_s 중 큰 값으로 한 번 계산해서 저장해두고,
        # _control_loop()가 그 값으로 _ramp_w()를 호출한다.
        self._lost_turn_w = self.lost_left_turn_w_rad_s

        # ---- ROS I/O ----
        path_topic = p('path_topic').value
        self.create_subscription(Path, path_topic, self._on_path, 10)

        cmd_topic = p('cmd_topic').value
        self.cmd_pub = self.create_publisher(Float32MultiArray, cmd_topic, 10)

        rate_hz = float(p('control_rate_hz').value)
        self.control_rate_hz = rate_hz  # max_linear_accel_mps2 램프의 dt 계산용
        self.control_timer = self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'purepursuit_node started: {path_topic} -> {self.base_frame}, '
            f'lookahead={self.lookahead_distance_m}m, v_target={self.target_linear_speed_m_s}m/s, '
            f'use_outer_wheel_boost={self.use_outer_wheel_boost}, '
            f'use_distance_scaled_speed={self.use_distance_scaled_speed}'
            + (f' (gain={self.distance_speed_gain}, '
               f'v=[{self.distance_speed_min_m_s},{self.distance_speed_max_m_s}]m/s, '
               f'setpoint=goal_tolerance_m={self.goal_tolerance_m}m)'
               if self.use_distance_scaled_speed else '') +
            f', cmd_topic={cmd_topic} @ {rate_hz}Hz'
        )

    # ------------------------------------------------------------------
    def _on_path(self, msg: Path):
        # [2026-09-05 신규] TF 성공/실패, poses 비었는지와 무관하게 "메시지가
        # 도착했다"는 사실 자체는 항상 기록한다 -- _control_loop()의 진단
        # 로그 분리용(모듈 상태 docstring 참고).
        self._last_msg_received_time = self.get_clock().now()

        if not msg.poses:
            # [2026-09-04 신규] 직전 프레임까지 경로가 있다가(=추종 중) 이번에
            # 처음 비어서 왔으면(was_tracking) "방금 놓친 순간"이다 -- 이때만
            # use_lost_left_turn_recovery 판단을 한다. 이미 빈 채로 여러
            # 프레임째(재포착 실패 지속 중)면 다시 트리거하지 않는다(2초
            # 창은 처음 놓친 그 순간 한 번만).
            was_tracking = bool(self._path_xy)
            # [2026-09-05 신규, 사용자 요청] self._path_xy가 아직 안 비워진
            # 이 시점의 값(=놓치기 직전 마지막 경로)에서 목표점 각도를 구해
            # "경로 자체가 좌측으로 휘어 있었는지"도 트리거 조건에 OR로
            # 추가한다 -- _compute_pure_pursuit()와 동일한 lookahead 목표점
            # 선택(짧으면 마지막 점 폴백)만 가져오고 속도 계산/상태갱신은
            # 안 하는 순수 헬퍼(_last_target_bearing_rad()).
            last_alpha = self._last_target_bearing_rad(self._path_xy) if was_tracking else 0.0
            path_curving_left = last_alpha > 0.0  # y>0=좌측(body 좌표 규약)
            was_turning_left = self._last_w >= self.lost_left_turn_w_threshold_rad_s

            if (self.use_lost_left_turn_recovery and was_tracking
                    and (path_curving_left or was_turning_left)):
                self._lost_turn_until = (
                    self.get_clock().now()
                    + Duration(seconds=self.lost_left_turn_duration_sec))
                # 목표 각속도 = bearing_gain*alpha(경로가 많이 휘어 있었을수록
                # 더 세게) 와 lost_left_turn_w_rad_s(하한) 중 큰 값. alpha<=0
                # (경로 조건 없이 last_w만으로 트리거된 경우)이면 이 항이
                # 0 이하가 돼서 자동으로 하한값이 대신 쓰인다.
                self._lost_turn_w = max(
                    self.lost_left_turn_bearing_gain * last_alpha,
                    self.lost_left_turn_w_rad_s)
                self.get_logger().warn(
                    f'Lost /path (last_alpha={math.degrees(last_alpha):.1f}deg, '
                    f'last_w={self._last_w:.2f}rad/s) -- blind left turn '
                    f'(w={self._lost_turn_w:.2f}rad/s) for '
                    f'{self.lost_left_turn_duration_sec:.1f}s to reacquire.',
                    throttle_duration_sec=2.0)
            else:
                if was_tracking:
                    self.get_logger().warn(
                        'Received an empty /path -- treating as no path (stopping).',
                        throttle_duration_sec=2.0)
                self._lost_turn_until = None
            self._path_xy = []
            self._last_path_time = self.get_clock().now()
            self._goal_reached = False
            return

        # 경로를 다시 받았다(재포착) -- 블라인드 좌회전 재시도 창이 열려
        # 있었다면 즉시 취소하고 정상 추종으로 복귀한다.
        self._lost_turn_until = None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=self.tf_lookup_timeout_sec))
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'TF lookup {self.base_frame} <- {msg.header.frame_id} failed: {exc}',
                throttle_duration_sec=2.0)
            return  # 이전에 받았던 경로/타임스탬프 유지 -- path_timeout_sec 안전망이 처리

        path_xy = []
        for pose_in in msg.poses:
            pose_out = tf2_geometry_msgs.do_transform_pose_stamped(pose_in, transform)
            path_xy.append((pose_out.pose.position.x, pose_out.pose.position.y))

        self._path_xy = path_xy
        self._last_path_time = self.get_clock().now()
        # 주의: 여기서 _goal_reached를 리셋하지 않는다 -- /path는 목표 근처에
        # 도달해 있는 동안에도 계속(15Hz) 재발행되므로, 매 수신마다 리셋하면
        # "한 번만 로그" 가드가 매번 풀려서 _compute_pure_pursuit()가 매 컨트롤
        # 틱마다 "Goal reached"를 다시 로그해버린다 (dryrun으로 확인된 스팸).
        # _goal_reached의 리셋은 _compute_pure_pursuit()가 실제 거리 기준으로
        # 판단해서 처리한다.

    # ------------------------------------------------------------------
    def _control_loop(self):
        left_dps, right_dps = 0.0, 0.0
        now = self.get_clock().now()

        # [2026-09-05 버그 수정, 사용자 확인] 예전엔 age_s(/path 발행이 끊긴
        # 시간) > path_timeout_sec 체크가 이 아래 lost_left_turn_recovery
        # 체크보다 elif 순서상 먼저 걸렸다 -- 그런데 인지팀이 빈 /path를
        # 한 번만 보내고 그 이후로 재발행을 안 하는 경우(매 tick 반복 발행이
        # 아니라 이벤트성 1회 발행), _lost_turn_until 재포착 창과 age_s
        # 타임아웃이 같은 시각(그 1회 메시지 수신 시점)에 시작해서 둘 다
        # path_timeout_sec/lost_left_turn_duration_sec(현재 둘 다 1.0초)
        # 만큼의 길이를 가지므로, age_s 타임아웃이 먼저 걸려서
        # "No /path for Xs. Stopping."으로 좌회전 재포착 로직에 도달하기도
        # 전에 끊겨버렸다. 그래서 재포착 창이 열려 있으면 /path 발행 상태와
        # 무관하게 이걸 최우선으로 체크/유지한다.
        if self._lost_turn_until is not None and now < self._lost_turn_until:
            self._last_v = 0.0
            # [2026-09-05] 고정값(lost_left_turn_w_rad_s) 대신 _on_path()가
            # 트리거 시점에 bearing_gain*alpha로 계산해둔 self._lost_turn_w
            # 사용 -- 자세한 계산은 _on_path() 참고.
            w = self._ramp_w(self._lost_turn_w)
            if self.use_single_wheel_pivot:
                left_dps, right_dps = self._single_wheel_pivot_to_dps(w)
            else:
                left_dps, right_dps = self._skid_steer_to_dps(0.0, w)
            msg = Float32MultiArray()
            msg.data = [left_dps, right_dps]
            self.cmd_pub.publish(msg)
            return

        self._lost_turn_until = None  # 창이 있었다면 자연 만료된 것

        age_s = (
            (now - self._last_path_time).nanoseconds * 1e-9
            if self._last_path_time is not None else None
        )
        if age_s is None:
            self._last_v = 0.0  # /path 아직 한 번도 못 받음 -- 정지 유지, 램프 기준점도 리셋
            self._last_w = 0.0
        elif age_s > self.path_timeout_sec:
            # [2026-09-05 신규] self._last_msg_received_time(메시지 도착
            # 자체, TF 성공 여부 무관)이 최근이면 "메시지는 오는데 TF
            # 변환만 계속 실패"하는 것 -- 진짜 "/path가 안 옴"과 로그로
            # 구분한다(모듈 상태 docstring 참고).
            recv_age_s = (
                (now - self._last_msg_received_time).nanoseconds * 1e-9
                if self._last_msg_received_time is not None else None
            )
            if recv_age_s is not None and recv_age_s <= self.path_timeout_sec:
                self.get_logger().warn(
                    f'/path messages ARE arriving (last one {recv_age_s:.2f}s ago) but '
                    f'TF transform has been failing for {age_s:.2f}s (timeout='
                    f'{self.path_timeout_sec}s) -- check "TF lookup ... failed" warnings '
                    f'above. Stopping.',
                    throttle_duration_sec=1.0)
            else:
                self.get_logger().warn(
                    f'No /path message received at all for {age_s:.2f}s '
                    f'(timeout={self.path_timeout_sec}s). Stopping.',
                    throttle_duration_sec=1.0)
            # 경로 소실/타임아웃은 안전 우선 -- max_linear_accel_mps2/
            # max_angular_accel_rad_s2 램프를 거치지 않고 즉시 0으로
            # 정지한다(모듈 docstring 참고). 다음 tick에 경로가 복구되면
            # 그때 0부터 다시 가속 램프를 탄다.
            self._last_v = 0.0
            self._last_w = 0.0
        elif not self._path_xy:
            # 빈 경로지만(escort_follow의 LOST 등) 재포착 창은 이미 위에서
            # 처리(활성 중이면 return, 만료됐으면 여기로 옴) -- 즉시 정지.
            self._last_v = 0.0
            self._last_w = 0.0
        else:
            v_target, curvature, w_override = self._compute_pure_pursuit(self._path_xy)
            # [2026-09-01 신규] 회전 중(제자리 회전이든 곡선 추종이든)에는
            # max_linear_accel_mps2 대신 max_linear_accel_while_turning_mps2
            # (기본 0.0 -- yaml로만 설정, 아래 declare_parameter 참고)로
            # 가속을 제한한다 -- 회전과 가속이 동시에 걸리면 바퀴 슬립이
            # 심해진다는 실차 피드백 대응. w_override가 있거나(제자리 회전)
            # curvature가 0이 아니면(곡선 추종) 회전 중으로 본다. 감속은
            # 이 구분과 무관하게 항상 max_linear_decel_mps2(안전 우선).
            # [2026-09-03, 버그 수정] w_override가 "None이 아니면" 회전 중으로
            # 봤었는데, bearing deadband 분기(_compute_pure_pursuit() 참고)가
            # "회전 불필요"를 표현하려고 w_override=0.0을 명시적으로 채우는
            # 경우까지 회전 중으로 오판됐다 -- 0.0도 "None이 아님"이라서.
            # 목표가 정면 데드밴드 안(가장 흔한 정상 상황)일 때마다
            # max_linear_accel_while_turning_mps2=0.0이 잘못 적용돼 정지
            # 상태에서 절대 가속을 못 하는 데드락이었다(관측: /motor_speed_cmd
            # 가 계속 [0.0, -0.0]). w_override의 크기까지 봐서 실제로
            # 회전량이 있을 때만(제자리 회전 분기) 회전 중으로 판정하도록 수정.
            is_turning = (
                (w_override is not None and abs(w_override) > 1e-9)
                or abs(curvature) > 1e-9)
            v = self._ramp_v(v_target, is_turning)
            # w_override가 있으면(제자리 회전 모드) v와 무관하게 그 값을
            # 그대로 쓴다 -- v*curvature 방식은 v가 0으로 램프되면 w도
            # 같이 0이 돼버려서 진짜 제자리 회전이 안 나온다(모듈 docstring/
            # _compute_pure_pursuit() 참고).
            w_target = w_override if w_override is not None else v * curvature
            # [2026-09-01 신규, 재도입] w도 v처럼 한 틱만에 목표치로 바로
            # 튀지 않고 max_angular_accel_rad_s2 이내로만 다가가게 램프 --
            # _ramp_w() 참고.
            w = self._ramp_w(w_target)
            if self.use_single_wheel_pivot and w_override is not None:
                # [2026-08-30, 사용자 결정] 제자리 회전 구간(w_override 활성)만
                # 한쪽 바퀴는 세워두고 반대쪽만 굴리는 피벗 턴으로 -- 아래
                # _single_wheel_pivot_to_dps() 참고. 전진/후진하며 도는 구간
                # (w_override가 None)은 그대로 기존 스큐-스티어.
                left_dps, right_dps = self._single_wheel_pivot_to_dps(w)
            else:
                left_dps, right_dps = self._skid_steer_to_dps(v, w)

        msg = Float32MultiArray()
        msg.data = [left_dps, right_dps]
        self.cmd_pub.publish(msg)

    def _ramp_v(self, v_target: float, is_turning: bool = False) -> float:
        """직전 tick에 실제로 낸 v(self._last_v)에서 v_target 쪽으로
        가속/감속 방향에 따라 다른 상한(max_linear_accel_mps2/
        max_linear_decel_mps2) 이내로만 움직인다 -- escort_follow 재포착
        직후처럼 v_target이 한 틱만에 크게 뛰어도 실제 발행 v는 점진적으로만
        따라간다(모듈 docstring 참고). |v_target|이 |직전 v|보다 크면(더
        빨라지는 쪽) 가속, 아니면(0 또는 반대 부호 쪽으로 다가가는 중) 감속
        으로 본다 -- 예를 들어 전진 중 후진으로 바뀌는 경우 먼저 0까지는
        감속 상한으로, 그 다음 0에서 후진 방향으로 커지는 구간은 다시 가속
        상한으로 램프된다(각각 단계에서 |v_target|>|last_v| 여부로 자연히
        갈림). dt는 control_timer 주기(고정, 20Hz 기본)를 그대로 쓴다.

        [2026-09-01 신규] is_turning=True(회전 중)이고 가속 방향이면
        max_linear_accel_mps2 대신 max_linear_accel_while_turning_mps2를
        쓴다 -- 회전+가속 동시 발생 시 바퀴 슬립이 심해지는 문제 대응
        (_control_loop() 참고). 감속은 회전 여부와 무관하게 항상
        max_linear_decel_mps2(안전 우선, 전류 스파이크 방지 목적 그대로).

        [2026-09-04, 버그 수정] max_linear_accel_while_turning_mps2 기본값이
        0.0(회전 중 가속 완전 금지)이라, 완전 정지(self._last_v==0.0)
        상태에서 목표가 정면 데드밴드 밖(bearing_deadband_lateral_m 밖)에
        있으면 curvature!=0 -> is_turning=True가 거의 항상 성립해서 영원히
        가속을 못 하는 데드락이 있었다(가림막 재포착 직후 로봇개가 화면
        중앙이 아니면 100% 재현 -- escort_follow가 LOST 동안 빈 /path를
        내서 _last_v=0으로 정지했다가, 재포착 시 y가 데드밴드 밖이면
        그대로 묶임). "완전 정지 상태에서 처음 출발할 때"만 예외로
        max_linear_accel_mps2를 쓰도록 해서, 정지 상태 탈출은 항상
        보장하고, 이미 움직이는 중에 회전이 걸리는 경우(원래 의도한 슬립
        방지 대상)는 기존처럼 max_linear_accel_while_turning_mps2를 그대로
        적용한다."""
        dt = 1.0 / self.control_rate_hz
        accelerating = abs(v_target) > abs(self._last_v)
        starting_from_stop = self._last_v == 0.0
        if accelerating and is_turning and not starting_from_stop:
            rate = self.max_linear_accel_while_turning_mps2
        elif accelerating:
            rate = self.max_linear_accel_mps2
        else:
            rate = self.max_linear_decel_mps2
        max_dv = rate * dt
        v = max(self._last_v - max_dv, min(self._last_v + max_dv, v_target))
        self._last_v = v
        return v

    def _ramp_w(self, w_target: float) -> float:
        """직전 tick에 실제로 낸 w(self._last_w)에서 w_target 쪽으로
        max_angular_accel_rad_s2 이내로만 움직인다 -- _ramp_v()와 같은
        방식이지만 가속/감속 구분 없이 단일 상한 하나만 쓴다(각속도는
        전류 스파이크가 아니라 회전이 갑자기 확 꺾이는 걸 완화하는 게
        목적이라 방향 구분이 불필요). w_override 활성 구간(제자리 회전)도
        이 램프를 그대로 거친다 -- 목표 각속도로 바로 튀지 않고 점진적으로
        다가간다. [주의] 이건 각속도 "자체"의 상한이 아니라 각속도가
        "변하는 속도"의 상한이다 -- 정상상태 w의 크기는 여전히 게인*각도
        계산값과 바퀴 dps 하드클램프(max_wheel_speed_dps)로 정해진다."""
        dt = 1.0 / self.control_rate_hz
        max_dw = self.max_angular_accel_rad_s2 * dt
        w = max(self._last_w - max_dw, min(self._last_w + max_dw, w_target))
        self._last_w = w
        return w

    # ------------------------------------------------------------------
    def _last_target_bearing_rad(self, path_xy) -> float:
        """use_lost_left_turn_recovery 전용 순수 헬퍼 -- _compute_pure_pursuit()
        와 동일한 lookahead 목표점 선택(짧으면 마지막 점 폴백)만 가져와서
        alpha=atan2(y,x)만 반환한다. self._goal_reached 등 상태를 전혀
        건드리지 않는다(속도 계산이 아니라 "경로가 어느 쪽으로 휘어
        있었는지"만 필요하므로)."""
        if not path_xy:
            return 0.0
        target = None
        for x, y in path_xy:
            if math.hypot(x, y) >= self.lookahead_distance_m:
                target = (x, y)
                break
        if target is None:
            target = path_xy[-1]
        return math.atan2(target[1], target[0])

    # ------------------------------------------------------------------
    def _compute_pure_pursuit(self, path_xy):
        """base_frame 기준 경로(근->원 순서)에서 lookahead 지점을 찾아
        (v_target, curvature, w_override)를 계산한다. 로봇은 항상
        base_frame 원점, 전방 = +x, 좌측 = +y.

        w(각속도)는 보통 여기서 안 곱한다 -- 호출부(_control_loop())가
        v_target을 max_linear_accel_mps2로 램프한 실제 v로 w=v*curvature를
        다시 계산한다. curvature(=회전반경의 역수)만 유지하면 램프 도중
        속도가 낮아도 같은 반경으로 돌기 때문에 추종 경로 자체는 안
        바뀐다(모듈 docstring 참고). [2026-08-31, 사용자 결정] 단,
        use_bearing_steering일 때 목표가 좌우 데드밴드
        (bearing_deadband_lateral_m) 안이면 "회전 불필요"를 뜻하는
        w_override=0.0을 채운다 -- v와 무관하게 직접 w를 지정해야 하기
        때문. w_override가 None이 아니면 호출부는 v*curvature 대신 이 값을
        그대로 쓴다. [2026-08-30, 도입 -> 2026-09-03, 사용자 지시로 제거]
        한때 목표각이 커도(bearing_inplace_threshold_rad 초과) v_target=0+
        w_override로 제자리 회전부터 시켰었는데, 회전하는 동안 전진이
        멎어 추종 거리가 벌어지는 문제로 이 분기를 없앴다 -- 이제 큰
        각도도 v_target을 죽이지 않고 curvature를 그대로 계산해서
        use_outer_wheel_boost(_skid_steer_to_dps() 참고, slope_traverse_
        node.cpp의 computeBoostedVx() 이식)로 바깥쪽 바퀴만 부스트하며
        돈다."""
        target = None
        for x, y in path_xy:
            if math.hypot(x, y) >= self.lookahead_distance_m:
                target = (x, y)
                break

        goal_x, goal_y = path_xy[-1]
        dist_to_goal = math.hypot(goal_x, goal_y)

        if target is None:
            # 경로 전체가 lookahead 반경 안 -- 마지막 점(경로 끝)을 목표로 삼는다.
            # [2026-08-30] use_distance_scaled_speed(escort_follow)일 때는 이
            # 하드 정지(goal_tolerance_m 기준)를 건너뛴다 -- 아래 밴드 로직이
            # 이 가까운 거리에서도(후진 포함) v_target을 알아서 정하므로,
            # 여기서 먼저 (0,0)으로 끊어버리면 밴드 하한 아래로 들어왔을 때
            # 후진 로직 자체가 실행이 안 된다.
            if not self.use_distance_scaled_speed and dist_to_goal <= self.goal_tolerance_m:
                if not self._goal_reached:  # 도착 순간에만 한 번 로그 (엣지 트리거)
                    self.get_logger().info(f'Goal reached (dist={dist_to_goal:.2f}m). Stopping.')
                    self._goal_reached = True
                return 0.0, 0.0, None
            self._goal_reached = False
            target = (goal_x, goal_y)
        else:
            self._goal_reached = False

        x, y = target
        look_dist = max(math.hypot(x, y), 1e-3)  # 0 나눗셈 방지

        if self.use_distance_scaled_speed:
            # escort_follow 전용 -- 모듈 docstring 참고. _on_path()에서 카메라
            # optical 검출점을 최신 base_link<-camera_link TF(위치+roll/pitch)
            # 로 먼저 변환했으므로, look_dist=hypot(x_base, y_base)는 카메라
            # 원점이 아니라 로봇 base_link 원점 기준 수평 추종거리다.
            # [2026-08-30, 사용자 결정] 단일 setpoint 대신
            # [distance_band_min_m, distance_band_max_m] 밴드를 유지한다 --
            # 밴드보다 멀면 전진 P제어(기존과 동일), 밴드보다 가까우면 후진
            # P제어(신규, distance_speed_reverse_max_m_s로 보수적으로 캡 --
            # 드라이브 카메라가 전방만 봐서 후방 장애물을 못 보기 때문),
            # 밴드 안이면 정지.
            if look_dist > self.distance_band_max_m:
                error = look_dist - self.distance_band_max_m
                v_target = max(self.distance_speed_min_m_s,
                                min(self.distance_speed_max_m_s, self.distance_speed_gain * error))
            elif look_dist < self.distance_band_min_m:
                error = look_dist - self.distance_band_min_m  # 음수
                v_target = max(-self.distance_speed_reverse_max_m_s,
                                min(0.0, self.distance_speed_gain * error))
            else:
                v_target = 0.0
        else:
            v_target = self.target_linear_speed_m_s

        w_override = None
        if self.use_bearing_steering:
            # escort_follow 전용 -- 모듈 docstring 참고. 표준 curvature(2y/L^2)는
            # L(목표까지 거리)이 커지면 각도가 커도 오히려 작아진다 -- 화면
            # 외곽(depth 오차 큰 구간)에서 L이 부풀려지면 조향이 거의 안
            # 나가는 문제로 이어짐. 대신 각도(alpha=atan2(y,x))만으로 직접
            # w를 정한다.
            alpha = math.atan2(y, x)
            w_target = self.bearing_steering_gain * alpha

            if abs(y) < self.bearing_deadband_lateral_m:
                # [2026-08-31, 사용자 결정] 좌우 오프셋(y, base_link 기준 --
                # 카메라 roll이 거의 0이라 카메라 광학 프레임 X와 부호만
                # 반대일 뿐 사실상 같은 값)이 이 범위 안이면 각도가 얼마든
                # 조향을 아예 안 한다(w=0) -- 예전엔 각도(alpha)가 정확히
                # 0에 가까워질 때까지 계속 미세 조향해서 직선 정렬에
                # 집착했는데, 그 대신 "좌우로 이 정도 벗어난 건 이미 충분히
                # 정면"으로 보는 데드밴드.
                w_override = 0.0
                curvature = 0.0  # 안 쓰임(w_override가 우선)
            # [2026-09-03, 사용자 지시로 제거] 목표각이 큰 경우(예전
            # bearing_inplace_threshold_rad 초과 시) v_target=0으로 덮어쓰고
            # w_override로 제자리 회전(피벗 턴)부터 시키던 분기가 여기 있었다
            # -- 되돌아보니 회전하는 동안 전진이 아예 멎어서 추종 거리가
            # 계속 벌어지는 문제가 있었다. slope_traverse_node.cpp의
            # computeBoostedVx() 기반 차동 조향(_skid_steer_to_dps()의
            # use_outer_wheel_boost, 안쪽 바퀴는 v_target 그대로 두고
            # 바깥쪽만 부스트)을 그대로 가져와 재사용 -- 아래 else 분기가
            # 각도 크기와 무관하게 curvature를 그대로 계산하므로, 큰 각도도
            # v_target을 0으로 죽이지 않고 회전이 필요한 쪽(바깥쪽 바퀴)만
            # 부스트해서 전진하며 돈다(mission_escort_drive.launch.py의
            # use_outer_wheel_boost:=true 참고). bearing_inplace_threshold_rad
            # 파라미터는 더 이상 안 쓰이지만(하위 호환/향후 재사용 대비)
            # 그대로 남겨둔다.
            else:
                # 임계값 이내 -- 기존처럼 전진/후진하면서 같이 돈다.
                # curvature=w/v 형태로 바꿔서 반환한다 -- 호출부(_control_loop())가
                # 어차피 w=v*curvature로 재계산하므로 이렇게 하면 v가 가속
                # 램프 중이어도 목표 각속도(w)가 유지된다. v_target이 후진
                # (음수)일 수 있어서, 분모의 부호를 v_target 부호에 맞춰야
                # 한다 -- 안 그러면(예: 그냥 max(v_target,0.05)) 후진 중에
                # 조향 방향이 반대로 뒤집힌다. (검산: w_target=+0.3,
                # v_target=-0.2 -> denom=min(-0.2,-0.05)=-0.2 -> curvature=
                # -1.5 -> ramp된 v가 -0.2에 도달하면 w=(-0.2)*(-1.5)=+0.3으로
                # 원하는 부호가 그대로 나옴.)
                denom = max(v_target, 0.05) if v_target >= 0.0 else min(v_target, -0.05)
                curvature = w_target / denom
        else:
            curvature = 2.0 * y / (look_dist * look_dist)  # 표준 pure pursuit 공식: kappa = 2*sin(alpha)/L = 2y/L^2
        return v_target, curvature, w_override

    # ------------------------------------------------------------------
    def _skid_steer_to_dps(self, v: float, w: float):
        """rmd_x8_driver_node._skid_steer_inverse() + _send_speed_command()와
        동일한 공식 (v[m/s], w[rad/s] -> 좌우 바퀴 각속도[dps]).

        use_outer_wheel_boost=True(경사 주행용, 모듈 docstring 참고)면 v를
        v+|w|*halftrack으로 올려서 안쪽 바퀴가 v(=target_linear_speed_m_s)
        밑으로 안 깎이고 바깥쪽만 부스트되게 한다 -- slope_traverse_node.cpp의
        computeBoostedVx()와 동일 공식(부호에 안 걸리게 |w| 사용, 대입해보면
        어느 방향으로 돌든 안쪽 바퀴가 정확히 v로 나옴을 검산할 수 있음)."""
        half_track = self.track_width_m / 2.0
        if self.use_outer_wheel_boost:
            v = v + abs(w) * half_track
        v_left = v - w * half_track
        v_right = v + w * half_track

        left_dps = math.degrees(v_left / self.wheel_radius_m) * self.left_motor_sign
        right_dps = math.degrees(v_right / self.wheel_radius_m) * self.right_motor_sign

        left_dps = max(-self.max_wheel_speed_dps, min(self.max_wheel_speed_dps, left_dps))
        right_dps = max(-self.max_wheel_speed_dps, min(self.max_wheel_speed_dps, right_dps))
        return left_dps, right_dps

    def _single_wheel_pivot_to_dps(self, w: float):
        """제자리 회전을 한쪽 바퀴 고정 + 반대쪽만 굴리는 피벗 턴으로
        변환한다(v[m/s]는 항상 0인 게 전제 -- 호출부에서 w_override가 활성일
        때만 이 함수를 씀).

        표준 스큐-스티어 제자리 회전(_skid_steer_to_dps(0, w))은 ICR(순간
        회전중심)이 로봇 중심이라 양쪽 바퀴가 반대 방향으로 서로 track_width/2
        만큼 돈다. 이 함수는 ICR을 한쪽 바퀴 위치로 옮겨서 그 바퀴는 정지,
        반대쪽만 굴린다 -- ICR에서 반대쪽 바퀴까지 거리가 track_width_m
        (절반이 아니라 전체)이므로 그 바퀴의 선속도는 |w| * track_width_m이다.

        w>0(좌회전, CCW)이면 왼쪽 바퀴를 축으로 오른쪽만, w<0(우회전, CW)이면
        오른쪽 바퀴를 축으로 왼쪽만 돌린다(_skid_steer_to_dps의 부호 규약과
        일치: w>0일 때 v_right가 커지고 v_left가 작아지는 것과 같은 회전
        방향)."""
        v_move = abs(w) * self.track_width_m
        if w >= 0.0:
            v_left, v_right = 0.0, v_move
        else:
            v_left, v_right = v_move, 0.0

        left_dps = math.degrees(v_left / self.wheel_radius_m) * self.left_motor_sign
        right_dps = math.degrees(v_right / self.wheel_radius_m) * self.right_motor_sign

        left_dps = max(-self.max_wheel_speed_dps, min(self.max_wheel_speed_dps, left_dps))
        right_dps = max(-self.max_wheel_speed_dps, min(self.max_wheel_speed_dps, right_dps))
        return left_dps, right_dps

    def destroy_node(self):
        # 종료 시 정지 명령을 한 번 명시적으로 발행 (can_driver_node의 워치독
        # 타임아웃만 믿지 않고, 노드가 정상 종료되는 경로에서는 즉시 멈추게)
        try:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0]
            self.cmd_pub.publish(msg)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl+C(SIGINT) 시 rclpy의 기본 시그널 핸들러가 이미
        # rclpy.shutdown()을 먼저 호출해버리는 경우가 있어서(TF 리스너
        # 스레드가 있어서 종료가 살짝 늦어지는 이 노드에서 특히 재현됨),
        # 여기서 또 부르면 "rcl_shutdown already called" 예외가 나고
        # ros2 launch가 정상 종료를 "process has died"로 잘못 표시한다.
        # 이미 종료됐으면 건너뛴다 -- 정상 종료 경로를 크래시처럼 안 보이게.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
