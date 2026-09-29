#!/usr/bin/env python3
"""Horizontal serial-arm gravity screening, SI units, no ROS dependencies.

Not inverse dynamics, CAD mass extraction, or a motor certification tool.
Each segment starts at a pitch joint; actuator mass sits at that joint.

워크스페이스 루트의 설계 보조 도구: ROS 노드나 description 설치 리소스가 아니다.
모든 링크가 같은 수평 방향으로 펼쳐진 피치 체인을 가정한다.
길이 m, 질량 kg, 토크 Nm를 사용하며 동역학·강성·roll/yaw 계산은 포함하지 않는다.
"""
# 명령행 입력, 후보 설정 복제, JSON 입출력, 유한수 검사와 파일 경로 처리.
import argparse
import copy
import json
import math
from pathlib import Path


def positive(value, label, zero=True):
    """value: 검사할 수치, label: 오류에 표시할 항목명, zero: 0 허용 여부.

    bool은 Python에서 int의 하위 타입이므로 숫자 검사 전에 명시적으로 제외한다.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label}: finite number required')
    # NaN/무한대/음수는 물리 입력으로 허용하지 않는다.
    if not math.isfinite(value) or value < 0 or (not zero and value == 0):
        raise ValueError(f'{label}: invalid value {value}')
    return value


def calculate(config):
    """JSON 설정 딕셔너리를 받아 관절별 정적 토크와 판정 결과를 반환한다."""
    # 1. 입력 검증: segments는 어깨에서 말단 방향으로 정렬된 피치 관절/링크 목록.
    segments = config['segments']  # 각 원소는 관절 시작점과 그 뒤 링크의 사양.
    if not segments:
        raise ValueError('segments must not be empty')
    sf = positive(config['safety_factor'], 'safety_factor', False)  # 무차원 정적 안전계수.
    if sf < 1:
        raise ValueError('safety_factor must be >= 1')
    payload = positive(config['payload_kg'], 'payload_kg')  # 취급 물체 질량 [kg].
    offset = positive(config['tool_offset_m'], 'tool_offset_m')  # 마지막 링크 끝~TCP 거리 [m].
    tool = positive(config['tool_mass_kg'], 'tool_mass_kg')  # 툴/어댑터 등 말단 조립체 질량 [kg].
    # 2. 수평 1차원 질량 모델: 첫 관절 위치를 x=0으로 둔다.
    points = []  # (첫 관절 기준 x 좌표 [m], 집중 질량 [kg]) 목록.
    origins = []  # 각 피치 관절의 x 좌표 [m].
    fixed_components = []  # EE 등 자세축은 아니지만 상류 하중에 포함되는 조립체.
    position = 0.0  # 현재 링크 시작점; 반복 종료 후 전체 링크 길이 [m].
    names = set()  # 중복 관절 이름을 검출하기 위한 집합.
    for s in segments:  # s: 현재 관절/링크의 입력 딕셔너리.
        # fixed는 EE 구성품: 길이/질량은 반영하지만 독립 자세 관절이 아니다.
        if s.get('joint_type', 'pitch') not in ('pitch', 'fixed'):
            raise ValueError('joint_type must be pitch or fixed')
        if s['name'] in names:
            raise ValueError('duplicate segment name')
        names.add(s['name'])
        length = positive(s['length_m'], 'length_m', False)  # 링크 길이 [m].
        mass = positive(s['link_mass_kg'], 'link_mass_kg')  # 모터/툴과 중복되지 않는 링크 질량 [kg].
        motor_mass = positive(s['actuator_mass_kg'], 'actuator_mass_kg')  # 관절 위치의 모터 조립체 질량 [kg].
        com = positive(s['com_fraction'], 'com_fraction')  # 링크 시작점에서 COM까지 거리/길이 [0~1].
        if com > 1:
            raise ValueError('com_fraction must be <= 1')
        rating = s['continuous_output_nm']  # 출력축 연속 정격 [Nm]; None이면 미확인.
        if rating is not None:
            positive(rating, 'continuous_output_nm', False)
        # 모터는 관절 위치, 링크 질량은 무게중심(COM)에 집중시킨다.
        origins.append(position)
        points.extend([(position, motor_mass), (position + length * com, mass)])
        position += length  # 다음 관절 위치로 이동 [m].
    # 별도 브래킷 등 추가 부품은 자세 관절과 분리하여 하류 질량으로만 반영한다.
    for load in config.get('additional_loads', []):  # load: 추가 부품 질량/장착 링크 정보.
        index = load['segment_index']  # 0부터 시작하는 장착 링크 번호.
        if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(segments):
            raise ValueError('additional_loads: invalid segment_index')
        fraction = positive(load['position_fraction'], 'position_fraction')  # 링크 내 위치 비율.
        if fraction > 1:
            raise ValueError('position_fraction must be <= 1')
        load_mass = positive(load['mass_kg'], 'additional load mass')  # 브래킷 포함 질량 [kg].
        points.append((origins[index] + segments[index]['length_m'] * fraction, load_mass))
    # 툴 COM을 따로 계산하지 않고 툴과 물체 질량 모두 TCP에 집중시키는 근사.
    points.append((position + offset, tool + payload))
    # 3. 관절별 하류 중력 모멘트를 합산하고 입력 정격과 비교한다.
    rows = []  # 관절 이름/모터/토크/정격/여유/판정을 담는 결과 목록.
    for s, origin in zip(segments, origins):  # origin: 계산 대상 관절의 x 위치 [m].
        if s.get('joint_type', 'pitch') == 'fixed':
            fixed_components.append(dict(component=s['name'], motor=s['motor'],
                                         link_length_m=s['length_m'],
                                         assembly_mass_kg=s['actuator_mass_kg'] + s['link_mass_kg'],
                                         role=s.get('role', 'fixed'),
                                         status='FIXED_LOAD_INCLUDED'))
            continue  # 도킹 모터 자체 토크는 체결 기구 계산이 필요하므로 여기서는 출력하지 않는다.
        # x: 질량 위치 [m], m: 질량 [kg], 9.80665: 표준 중력가속도 [m/s²].
        # x-origin은 지렛팔. 상류 질량은 제외하며 자기 모터는 지렛팔이 0이다.
        # 이 제외법은 링크가 모두 +x 방향으로 펼쳐진 현재 가정에서만 유효하다.
        gravity = 9.80665 * sum(m * max(0.0, x - origin) for x, m in points)
        required = sf * gravity  # 안전계수를 곱한 정적 요구 토크 [Nm]; 가속 토크 미포함.
        rating = s['continuous_output_nm']  # 출력축 연속 정격 [Nm]; None이면 미확인.
        # 여유=정격-요구 토크 [Nm]. STATIC_ONLY_OK는 정적 조건만의 통과다.
        rows.append(dict(joint=s['name'], motor=s['motor'], link_length_m=s['length_m'], gravity_nm=gravity,
                         factored_static_nm=required, continuous_output_nm=rating,
                         static_margin_nm=None if rating is None else rating-required,
                         status='UNKNOWN' if rating is None else
                         ('STATIC_ONLY_OK' if rating >= required else 'STATIC_OVERLOAD')))
    # reach_proxy는 길이 합일 뿐 관절 제한/오프셋을 고려한 실제 작업영역이 아니다.
    return dict(note='Illustrative masses; static horizontal screening only. No dynamic approval.',
                reach_proxy_m=position+offset, joints=rows, fixed_components=fixed_components)


def resize_link(config, name, length, mass_mode='fixed'):
    """지정 링크만 변경한 복사본 반환. fixed는 질량 유지, proportional은 선밀도 유지."""
    positive(length, 'link length', False)  # 변경 후 절대 길이 [m].
    if mass_mode not in ('fixed', 'proportional'):
        raise ValueError('invalid mass mode')
    changed = copy.deepcopy(config)  # 원본과 다른 비교 후보를 독립적으로 보존한다.
    matches = [s for s in changed['segments'] if s['name'] == name]  # 관절 이름으로 링크 선택.
    if len(matches) != 1:
        raise ValueError(f'unknown or duplicate link: {name}')
    link = matches[0]  # 해당 관절에서 다음 관절로 이어지는 링크.
    if mass_mode == 'proportional':
        link['link_mass_kg'] *= length / link['length_m']  # 동일 재료/단면일 때만 적용.
    link['length_m'] = length  # 모터, 툴 질량과 다른 링크 길이는 유지한다.
    return changed


def resize_links(config, overrides, mass_mode="fixed"):
    """Apply multiple structural-link changes to one candidate."""
    changed = copy.deepcopy(config)
    names = set()
    for name, length in overrides:
        if name in names:
            raise ValueError(f"duplicate --link-length for {name}")
        names.add(name)
        changed = resize_link(changed, name, length, mass_mode)
    return changed


def floor_target_summary(target_config, reach_proxy):
    """Report target distance only; never claim IK/collision validation."""
    base = target_config["base_origin_world_xyz_m"]
    target = target_config["target_tcp_world_xyz_m"]
    if len(base) != 3 or len(target) != 3:
        raise ValueError("target config base/target coordinates must have three values")
    base = [positive(value, "base origin coordinate") for value in base]
    target = [positive(value, "target coordinate") for value in target]
    relative = [target_value - base_value for target_value, base_value in zip(target, base)]
    distance = math.sqrt(sum(value * value for value in relative))
    return dict(frame=target_config.get("frame"), target_tcp_world_xyz_m=target,
                base_origin_world_xyz_m=base, target_relative_xyz_m=relative,
                target_distance_m=distance, reach_proxy_m=reach_proxy,
                reach_proxy_margin_m=reach_proxy - distance,
                status="REACH_PROXY_ONLY_NOT_IK_OR_COLLISION_VALIDATED")


def configure_extra_pitch(config, length):
    """길이 [m]가 0이면 미장착, 양수이면 별도 MX-106T 자세축을 EE 앞에 배치.

    이전 실험용 CLI/ROS 모니터와의 호환을 위해 함수명은 유지한다. 현재 기준 입력에는
    이미 ``extra``가 들어 있으므로, 일반적인 계산에는 이 함수를 호출할 필요가 없다.
    """
    positive(length, 'extra_pitch_m')
    changed = copy.deepcopy(config)  # 원본 입력은 보존한다.
    segments = changed['segments']  # 추가 축을 넣거나 뺄 직렬 체인.
    existing = next((i for i, link in enumerate(segments)
                     if link['name'] == 'extra_pitch'), None)
    loads = changed.get('additional_loads', [])  # 인덱스 변경 시 장착 링크를 유지한다.
    if length == 0:
        if existing is not None:
            if any(load['segment_index'] == existing for load in loads):
                raise ValueError('Move loads attached to extra_pitch before removing it')
            segments.pop(existing)
            for load in loads:
                if load['segment_index'] > existing:
                    load['segment_index'] -= 1
    elif existing is not None:
        segments[existing]['length_m'] = length  # 사용자 질량/COM은 유지한다.
    else:
        # wrist 뒤, 고정 EE 구동부 앞에 삽입한다. 따라서 고정 EE와 TCP 하중은
        # extra의 하류 질량으로 포함된다.
        index = next((i for i, link in enumerate(segments)
                      if link.get('role') == 'end_effector_power'), len(segments))
        segments.insert(index, dict(name='extra_pitch', motor='MX-106T', joint_type='pitch',
                                    length_m=length, link_mass_kg=.05, com_fraction=.5,
                                    actuator_mass_kg=.153, continuous_output_nm=None,
                                    role='distal_pose',
                                    motor_spec_note='4.2 Nm is a provisional static design limit, not a manufacturer continuous rating.'))
        for load in loads:
            if load['segment_index'] >= index:
                load['segment_index'] += 1
    return changed


def configure_extra_roll(config, length):
    """Insert a TCP-axis roll module after yaw and before the fixed EE."""
    positive(length, "extra_roll_m")
    changed = copy.deepcopy(config)
    segments = changed["segments"]
    existing = next((i for i, link in enumerate(segments) if link["name"] == "extra_roll"), None)
    loads = changed.get("additional_loads", [])
    if length == 0:
        if existing is not None:
            if any(load["segment_index"] == existing for load in loads):
                raise ValueError("Move loads attached to extra_roll before removing it")
            segments.pop(existing)
            for load in loads:
                if load["segment_index"] > existing:
                    load["segment_index"] -= 1
    elif existing is not None:
        segments[existing]["length_m"] = length
    else:
        index = next((i for i, link in enumerate(segments)
                      if link.get("role") == "end_effector_power"), len(segments))
        segments.insert(index, dict(name="extra_roll", motor="MX-106T", joint_type="fixed",
                                    length_m=length, link_mass_kg=.05, com_fraction=.5,
                                    actuator_mass_kg=.153, continuous_output_nm=None,
                                    role="distal_roll",
                                    motor_spec_note="TCP-axis roll. Static gravity is included in upstream pitch axes; roll-axis torque requires a 3D external-wrench/dynamics check."))
        for load in loads:
            if load["segment_index"] >= index:
                load["segment_index"] += 1
    return changed


def configure_extra_pitch_rating(config, rating):
    """Set the distal-pitch continuous torque used for the static screen."""
    positive(rating, "extra pitch continuous torque", False)
    changed = copy.deepcopy(config)
    matches = [segment for segment in changed["segments"]
               if segment["name"] == "extra_pitch" and segment.get("joint_type", "pitch") == "pitch"]
    if len(matches) != 1:
        raise ValueError("extra pitch rating requires exactly one extra_pitch joint")
    matches[0]["continuous_output_nm"] = rating
    return changed


def configure_ee_assembly_mass(config, mass):
    """고정 도킹 EE 조립체의 총질량을 바꾸되 기존 COM 근사 비율을 보존한다.

    ``ee``의 actuator_mass_kg(모터/브래킷, 링크 시작점)와 link_mass_kg(구조물,
    링크 COM)는 서로 다른 지렛팔에 놓인다. 총질량 하나만 입력받을 때 두 값을
    같은 비율로 스케일해 이 위치 관계를 유지한다.
    """
    positive(mass, 'ee assembly mass')
    changed = copy.deepcopy(config)
    matches = [segment for segment in changed['segments']
               if segment['name'] == 'ee' and segment.get('joint_type') == 'fixed']
    if len(matches) != 1:
        raise ValueError('ee assembly mass requires exactly one fixed segment named ee')
    ee = matches[0]
    original = ee['actuator_mass_kg'] + ee['link_mass_kg']
    if original <= 0:
        raise ValueError('ee assembly mass requires a positive existing ee mass')
    scale = mass / original
    ee['actuator_mass_kg'] *= scale
    ee['link_mass_kg'] *= scale
    return changed


def add_deltas(report, baseline):
    """기준안 대비 정적 토크 변화량과 감소율을 같은 이름의 관절에 추가한다."""
    reference = {j['joint']: j['gravity_nm'] for j in baseline['joints']}  # 기준 토크 [Nm].
    for row in report['joints']:  # 후보 관절별 결과.
        before = reference.get(row['joint'])  # 새 관절은 비교 대상이 없으므로 None.
        row['gravity_delta_nm'] = None if before is None else row['gravity_nm'] - before
        row['gravity_reduction_percent'] = (None if before is None or before == 0 else
                                            100 * (before - row['gravity_nm']) / before)
    return report


def main():
    """기본 설계와 변경 후보를 계산하고 터미널 또는 JSON으로 출력한다."""
    # 4. 명령행 인터페이스: 설정 경로와 길이 배율/payload 덮어쓰기를 받는다.
    parser = argparse.ArgumentParser(description=__doc__)  # CLI 인자 정의/오류 출력기.
    parser.add_argument('config', type=Path)
    parser.add_argument('--length-scale', type=float, default=1.0,
                        help='Scale segment lengths AND link masses at constant linear density')
    parser.add_argument('--payload-kg', type=float)
    parser.add_argument('--candidate-config', type=Path,
                        help='Separate candidate design JSON; scale and payload overrides apply to candidate')
    # 지정 링크의 절대 길이 목록을 각각 독립적인 후보로 계산한다(단위 m).
    parser.add_argument('--sweep-link', help='Segment name: shoulder=L1, elbow=L2, wrist=L3')
    parser.add_argument('--lengths-m', nargs='+', type=float, help='Candidate absolute lengths in metres')
    parser.add_argument('--link-mass-mode', choices=['fixed', 'proportional'], default='fixed',
                        help='Sweep only: fixed mass (default) or constant linear density')
    parser.add_argument('--link-length', action='append', nargs=2, metavar=('NAME', 'METRES'),
                        help='Set a structural-link length; repeat to change L2/L3 together')
    parser.add_argument('--target-config', type=Path,
                        help='Target JSON to report a reach proxy, not IK/collision validation')
    parser.add_argument('--extra-pitch-m', type=float,
                        help='Legacy convenience option: 0 removes extra_pitch; positive adds/resizes it before fixed EE')
    parser.add_argument('--extra-pitch-continuous-nm', type=float,
                        help='Continuous output torque for added extra_pitch; omit when unknown')
    parser.add_argument('--extra-roll-m', type=float,
                        help='Add/remove an MX-106T TCP-axis roll module before the fixed EE')
    parser.add_argument('--ee-assembly-mass-kg', type=float,
                        help='Override total mass of fixed segment "ee"; preserves its existing motor/bracket-to-structure mass ratio')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()  # config: Path, length_scale: 배율, payload_kg: 선택값, json: 출력 형식.
    # 5. 기준안을 먼저 계산하고 깊은 복사로 후보만 변경한다.
    try:
        if bool(args.sweep_link) != bool(args.lengths_m):
            raise ValueError('--sweep-link and --lengths-m must be supplied together')
        config = json.loads(args.config.read_text())  # 사용자가 제공한 기본 설계 입력.
        base = calculate(config)  # 수정 전 기준안 결과.
        # 별도 후보 파일이 있으면 다른 링크/말단 하중 구성을 기준안과 비교한다.
        changed = (json.loads(args.candidate_config.read_text()) if args.candidate_config
                   else copy.deepcopy(config))  # 기준 입력을 변경하지 않는 후보 딕셔너리.
        calculate(changed)  # 배율 적용 전에 후보의 입력 형식과 물리량을 검증한다.
        positive(args.length_scale, 'length_scale', False)
        # 같은 재료/단면(선밀도 일정)을 가정해 길이와 링크 질량을 함께 늘린다.
        # 모터 질량, COM 비율, TCP offset은 그대로 유지한다.
        for s in changed['segments']:  # s: 변경 후보의 개별 링크.
            s['length_m'] *= args.length_scale
            s['link_mass_kg'] *= args.length_scale
        # 옵션을 생략하면 기준 payload 유지; 명시한 0 kg도 정상 반영한다.
        if args.payload_kg is not None:
            changed['payload_kg'] = args.payload_kg
        if args.extra_pitch_m is not None:
            changed = configure_extra_pitch(changed, args.extra_pitch_m)
        if args.extra_roll_m is not None:
            changed = configure_extra_roll(changed, args.extra_roll_m)
        if args.link_length:
            overrides = [(name, float(length)) for name, length in args.link_length]
            changed = resize_links(changed, overrides, args.link_mass_mode)
        if args.extra_pitch_continuous_nm is not None:
            changed = configure_extra_pitch_rating(changed, args.extra_pitch_continuous_nm)
        result = dict(baseline=base)  # 모든 후보를 원래 기준안과 비교한다.
        if args.sweep_link:
            for index, length in enumerate(args.lengths_m):  # index: 중복 길이도 구분하는 후보 번호.
                candidate = resize_link(changed, args.sweep_link, length, args.link_mass_mode)
                # sweep이 구조 링크 질량을 먼저 갱신한 다음 총 EE 질량을 맞춘다.
                # 그래야 --ee-assembly-mass-kg가 최종 조립체 총질량을 뜻한다.
                if args.ee_assembly_mass_kg is not None:
                    candidate = configure_ee_assembly_mass(candidate, args.ee_assembly_mass_kg)
                report = add_deltas(calculate(candidate), base)  # 후보 토크와 기준 대비 변화.
                report['sweep'] = dict(link=args.sweep_link, length_m=length,
                                       link_mass_mode=args.link_mass_mode)
                result[f'candidate_{index + 1}_{args.sweep_link}_{length:g}m'] = report
        else:
            if args.ee_assembly_mass_kg is not None:
                changed = configure_ee_assembly_mass(changed, args.ee_assembly_mass_kg)
            result['candidate'] = add_deltas(calculate(changed), base)
        if args.target_config:
            target_config = json.loads(args.target_config.read_text())
            for report in result.values():
                report["floor_target"] = floor_target_summary(target_config, report["reach_proxy_m"])
    except (ValueError, KeyError, TypeError, OSError) as exc:  # exc: 입력/파일 관련 오류 정보.
        parser.error(str(exc))
    # 6. JSON은 후속 분석용, 일반 출력은 사람이 읽는 관절별 요약용이다.
    if args.json:
        print(json.dumps(result, indent=2, allow_nan=False))
    else:
        print('ILLUSTRATIVE STATIC SCREENING — replace assumed masses before design decisions')
        for label, report in result.items():  # label: baseline/candidate, report: 해당 계산 결과.
            print(f"\n{label}: straight-line reach proxy {report['reach_proxy_m']:.3f} m")
            if 'sweep' in report:
                print(f"  mass mode: {report['sweep']['link_mass_mode']}")
            if "floor_target" in report:
                floor = report["floor_target"]
                print(f"  floor target: {floor['target_distance_m']:.3f} m; reach-proxy margin {floor['reach_proxy_margin_m']:.3f} m ({floor['status']})")
            print(f"{'joint':12} {'motor':16} {'link(m)':>8} {'gravity':>10} {'SF load':>10} {'limit':>10} {'margin':>10} status")
            for j in report['joints']:  # j: 현재 출력할 관절별 결과 딕셔너리.
                reduction = j.get('gravity_reduction_percent')  # 양수면 감소, 음수면 증가 [%].
                suffix = '' if reduction is None else f" reduction={reduction:.2f}%"
                limit = 'UNKNOWN' if j['continuous_output_nm'] is None else f"{j['continuous_output_nm']:.3f}"
                margin = 'UNKNOWN' if j['static_margin_nm'] is None else f"{j['static_margin_nm']:.3f}"
                print(f"{j['joint']:12} {j['motor']:16} {j['link_length_m']:8.3f} "
                      f"{j['gravity_nm']:8.3f} Nm {j['factored_static_nm']:8.3f} Nm "
                      f"{limit:>8} Nm {margin:>8} Nm {j['status']}{suffix}")
            for fixed in report['fixed_components']:
                print(f"{fixed['component']:12} {fixed['motor']:16} {fixed['link_length_m']:8.3f} "
                      f"{'N/A':>10} {'N/A':>10} {'N/A':>10} {'N/A':>10} "
                      f"{fixed['status']} mass={fixed['assembly_mass_kg']:.3f} kg")


# 직접 실행할 때만 CLI를 시작한다. 테스트에서 import하면 계산 함수만 사용한다.
if __name__ == '__main__':
    main()
