#!/usr/bin/env python3
# 이 파일을 직접 실행할 때 Python 3 인터프리터를 사용합니다.
"""Pure helpers shared by arm named-pose tools.

This module deliberately has no ROS imports so configuration checks can run in
unit tests and before a node is started.
"""
# 이 모듈은 ROS 실행 환경 없이도 설정과 관절 상태의 안전성을 검사합니다.
# 따라서 테스트나 ROS 노드 시작 전에도 같은 검증 함수를 재사용할 수 있습니다.

# 타입 어노테이션의 지연 평가를 활성화합니다.
from __future__ import annotations

# 수치가 유한한지와 관절 범위 조건을 검사하는 데 사용합니다.
import math
# 설정 파일 경로를 타입으로 명확히 표현하는 표준 라이브러리 클래스입니다.
from pathlib import Path
# 키-값 맵과 순서가 있는 관절 이름 입력의 타입을 선언합니다.
from typing import Mapping, Sequence

# YAML 설정 파일을 파이썬 자료형으로 안전하게 읽습니다.
import yaml


# 지정한 관절 각각의 유효한 소프트 제한값을 YAML에서 읽어 반환합니다.
def load_soft_limits(path: Path, joint_names: Sequence[str]) -> dict[str, tuple[float, float]]:
    """Load finite, ordered soft limits for exactly the requested joints."""
    # 파일을 UTF-8로 열고 YAML 내용을 파이썬 객체로 안전하게 파싱합니다.
    try:
        # with 블록을 사용해 파싱 성공 여부와 관계없이 파일을 닫습니다.
        with path.open(encoding="utf-8") as stream:
            # 임의의 객체 생성 없이 YAML 기본 자료형만 읽습니다.
            config = yaml.safe_load(stream)
    # YAML 문법이 잘못된 경우 파일 경로와 원인을 포함해 입력 오류로 변환합니다.
    except yaml.YAMLError as exc:
        # 원래 YAML 예외를 연결해 구체적인 파싱 원인도 보존합니다.
        raise ValueError(f"{path}: invalid YAML: {exc}") from exc
    # 최상위 YAML 노드는 관절 설정을 담는 맵이어야 합니다.
    if not isinstance(config, dict):
        # 다른 형식의 최상위 값은 제한 설정으로 사용할 수 없습니다.
        raise ValueError(f"{path}: YAML root must be a mapping")

    # 최상위 맵에서 관절별 설정 블록을 가져옵니다.
    joints = config.get("joints")
    # 관절별 설정도 이름을 키로 갖는 맵이어야 합니다.
    if not isinstance(joints, dict):
        # 필수 joints 블록이 없거나 형식이 틀리면 즉시 실패시킵니다.
        raise ValueError(f"{path}: joints mapping is required")

    # 요청된 관절 이름과 각각의 (하한, 상한)을 담을 결과 맵을 준비합니다.
    limits: dict[str, tuple[float, float]] = {}
    # 호출자가 요청한 관절만 지정된 순서대로 검증하고 결과에 포함합니다.
    for name in joint_names:
        # 현재 관절 이름에 해당하는 YAML 설정을 찾습니다.
        spec = joints.get(name)
        # 설정이 맵일 때만 soft_limit_rad 항목을 읽고, 아니면 누락으로 취급합니다.
        raw = spec.get("soft_limit_rad") if isinstance(spec, dict) else None
        # 제한값은 하한과 상한 두 값을 가진 YAML 리스트여야 합니다.
        if not isinstance(raw, list) or len(raw) != 2:
            # 제한 누락이나 원소 수 오류를 파일과 관절 이름을 포함해 알립니다.
            raise ValueError(f"{path}: {name}.soft_limit_rad must contain [lower, upper]")
        # 두 제한값을 수치형 실수로 변환합니다.
        lower, upper = (float(raw[0]), float(raw[1]))
        # 값이 유한하며 하한이 상한보다 작은 정상 구간인지 확인합니다.
        if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
            # NaN, 무한대, 역전되거나 비어 있는 구간을 거부합니다.
            raise ValueError(f"{path}: invalid {name}.soft_limit_rad={raw}")
        # 검증된 하한과 상한을 관절 이름에 연결해 저장합니다.
        limits[name] = (lower, upper)
    # 요청한 모든 관절에 대해 검증된 소프트 제한값을 반환합니다.
    return limits


# 주어진 관절 위치가 제한값과 안전 여유 안에 있는지 검사합니다.
def validate_joint_positions(
    # 관절 이름별 현재 또는 목표 위치를 입력받습니다.
    positions: Mapping[str, float],
    # 관절 이름별 (하한, 상한) 쌍을 입력받습니다.
    limits: Mapping[str, tuple[float, float]],
    # 기계적 제한에서 안쪽으로 확보할 최소 여유를 라디안으로 받습니다.
    margin_rad: float = 0.0,
    # 위치의 종류를 오류 메시지에 표시하기 위한 선택형 키워드 인자입니다.
    *,
    label: str = "pose",
) -> None:
    """Reject missing, non-finite, or limit-adjacent joint positions."""
    # 여유값 자체가 유한한 0 이상인지 먼저 검증합니다.
    if not math.isfinite(margin_rad) or margin_rad < 0.0:
        # 음수나 비유한 여유값은 유효한 안전 구간으로 사용할 수 없습니다.
        raise ValueError("limit margin must be finite and nonnegative")

    # 여러 관절에서 발견된 위반 사유를 모아 한 번에 보고합니다.
    errors = []
    # 제한 맵의 모든 관절을 확인해 누락·비유한 값·범위 초과를 찾습니다.
    for name, (lower, upper) in limits.items():
        # 여유를 양쪽에서 제외하고도 유효 구간이 남는지 검사합니다.
        if margin_rad * 2.0 >= upper - lower:
            # 여유가 제한 폭 이상이면 그 관절에는 안전한 위치가 존재하지 않습니다.
            errors.append(
                f"{name}: margin {margin_rad:.6f} leaves no valid interval "
                f"inside [{lower:.6f}, {upper:.6f}]"
            )
            # 이 관절은 위치값 검사로 넘어가지 않고 다음 관절을 확인합니다.
            continue
        # 현재 관절의 위치를 맵에서 가져옵니다; 없으면 None이 됩니다.
        value = positions.get(name)
        # 위치가 누락되었거나 실수 변환 후 유한하지 않으면 검증 오류입니다.
        if value is None or not math.isfinite(float(value)):
            # 관절마다 오류를 기록하고 다른 관절 검사도 계속합니다.
            errors.append(f"{name}: finite position is required")
            continue
        # 검사와 출력에 일관되도록 위치를 실수형으로 변환합니다.
        value = float(value)
        # 여유값만큼 안쪽으로 이동한 안전 하한을 계산합니다.
        safe_lower = lower + margin_rad
        # 여유값만큼 안쪽으로 이동한 안전 상한을 계산합니다.
        safe_upper = upper - margin_rad
        # 위치가 안전 구간의 경계 바깥에 있는지 검사합니다.
        if value < safe_lower or value > safe_upper:
            # 위반 관절과 허용된 안전 구간을 진단 목록에 기록합니다.
            errors.append(
                f"{name}={value:.6f} outside "
                f"[{safe_lower:.6f}, {safe_upper:.6f}]"
            )
    # 하나 이상의 위반이 모였으면 성공으로 간주하지 않고 통합 오류를 냅니다.
    if errors:
        # 호출자가 지정한 대상 이름과 각 관절 오류를 하나의 메시지로 전달합니다.
        raise ValueError(f"{label} violates arm soft limits: " + "; ".join(errors))


# 현재 자세와 목표 자세 사이의 관절별 절대 오차를 계산합니다.
def pose_errors(
    # 관절 이름별 현재 위치를 입력받습니다.
    current: Mapping[str, float],
    # 관절 이름별 목표 위치를 입력받습니다.
    target: Mapping[str, float],
    # 오차를 계산할 관절과 계산 순서를 지정합니다.
    joint_names: Sequence[str],
) -> dict[str, float]:
    """Return absolute per-joint errors, rejecting incomplete states."""
    # 관절 이름과 절대 오차를 담을 결과 맵을 준비합니다.
    errors: dict[str, float] = {}
    # 요청된 각 관절에 대해 현재값과 목표값을 짝지어 검사합니다.
    for name in joint_names:
        # 현재 상태 맵에서 해당 관절의 피드백 값을 찾습니다.
        actual = current.get(name)
        # 목표 상태 맵에서 같은 관절의 목표값을 찾습니다.
        goal = target.get(name)
        # 어느 한쪽이라도 없으면 오차를 정의할 수 없습니다.
        if actual is None or goal is None:
            # 누락된 관절 이름을 포함해 상태 불완전 오류를 전달합니다.
            raise ValueError(f"missing joint position: {name}")
        # 입력 자료형과 무관하게 현재 위치를 실수로 변환합니다.
        actual = float(actual)
        # 목표 위치도 실수로 변환합니다.
        goal = float(goal)
        # 두 위치 모두 유한한 수인지 확인합니다.
        if not (math.isfinite(actual) and math.isfinite(goal)):
            # NaN이나 무한대가 오차 결과로 전파되지 않도록 거부합니다.
            raise ValueError(f"non-finite joint position: {name}")
        # 목표와 현재의 차이에 절댓값을 적용해 관절별 오차로 저장합니다.
        errors[name] = abs(actual - goal)
    # 요청한 관절 순서에 따라 계산된 오차 맵을 반환합니다.
    return errors


# 모든 지정 관절이 유한한 속도 피드백을 가지며 정지 기준 이내인지 판정합니다.
def joints_stopped(
    # 관절 이름별 각속도 피드백을 입력받습니다.
    velocities: Mapping[str, float],
    # 정지 여부를 확인할 관절 이름과 검사 순서를 지정합니다.
    joint_names: Sequence[str],
    # 허용 가능한 최대 절대 각속도 기준을 라디안/초로 입력받습니다.
    maximum_abs_velocity_rad_s: float,
) -> bool:
    """Return true only if every requested joint has finite, near-zero feedback."""
    # 정지 기준은 유한한 0 이상의 값이어야 합니다.
    if (not math.isfinite(maximum_abs_velocity_rad_s) or
            maximum_abs_velocity_rad_s < 0.0):
        # 의미가 없는 기준값이면 정지 여부 대신 입력 오류를 알립니다.
        raise ValueError("maximum stopped velocity must be finite and nonnegative")
    # 지정된 관절이 모두 기준을 만족하는지 차례대로 확인합니다.
    for name in joint_names:
        # 현재 관절의 속도 피드백을 가져옵니다.
        value = velocities.get(name)
        # 피드백이 없거나 유한한 숫자가 아니면 정지로 확정할 수 없습니다.
        if value is None or not math.isfinite(float(value)):
            # 불완전하거나 잘못된 피드백에 대해서는 안전하게 False를 반환합니다.
            return False
        # 해당 관절의 절대 각속도가 허용 기준보다 큰지 확인합니다.
        if abs(float(value)) > maximum_abs_velocity_rad_s:
            # 한 관절이라도 기준보다 움직이면 전체 관절이 정지한 것은 아닙니다.
            return False
    # 모든 관절 피드백이 유효하고 속도 기준을 만족하면 정지로 판정합니다.
    return True
