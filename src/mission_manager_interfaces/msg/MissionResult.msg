# TODO(임시): 실제 미션 로직 구현 전 스키마라 검증 안 됨 — 안 맞으면 교체될 수 있음.
std_msgs/Header header
string mission_name        # "spring_ifof" | "summer_traffic" | "fall_marker" | "escort_follow"
string state                # 미션별 상태 문자열
geometry_msgs/Point target_point   # 필요 없으면 0,0,0
bool valid                  # 이번 프레임에서 유효한 판정이 나왔는지
