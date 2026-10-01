#!/usr/bin/env python3
"""Fail-closed preflight for real arm control and attachment-only docking."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

from hardware_preflight import validate as validate_hardware
from tool_ids import NO_TOOL_ID, SUPPORTED_TOOL_IDS, TOOL_TAG_IDS, UNKNOWN_TOOL_ID


def _number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _number_list(value, length: int) -> bool:
    return isinstance(value, list) and len(value) == length and all(_number(item) for item in value)


def _positive(value) -> bool:
    return _number(value) and value > 0.0


def _ros_number_list(value, length: int) -> bool:
    """ROS 2 YAML arrays must not mix integer and double scalar types."""
    if not _number_list(value, length):
        return False
    types = {float if isinstance(item, float) else int for item in value}
    return len(types) == 1


def _primitive_list(value) -> bool:
    if not isinstance(value, list) or not value:
        return False
    for primitive in value:
        if not isinstance(primitive, dict) or primitive.get('type') not in ('box', 'cylinder'):
            return False
        if not _number_list(primitive.get('xyz'), 3) or not _number_list(primitive.get('quat_xyzw', [0, 0, 0, 1]), 4):
            return False
        if primitive['type'] == 'box' and not (_number_list(primitive.get('size'), 3) and all(x > 0.0 for x in primitive['size'])):
            return False
        if primitive['type'] == 'cylinder' and not (_positive(primitive.get('radius')) and _positive(primitive.get('length'))):
            return False
    return True


def _tool_error(errors: list[str], path: str, condition: bool, description: str) -> None:
    if not condition:
        errors.append(f'{path}: {description}')


def validate_tools(config: dict) -> list[str]:
    errors: list[str] = []
    tools = config.get('tools')
    if not isinstance(tools, dict):
        return ['tools.yaml.tools: required map']
    real = config.get('real_docking')
    if not isinstance(real, dict):
        return ['tools.yaml.real_docking: required map']
    state_ids = config.get('state_ids')
    if not isinstance(state_ids, dict):
        errors.append('tools.yaml.state_ids: required map')
        state_ids = {}
    _tool_error(errors, 'tools.yaml.state_ids.no_tool',
                state_ids.get('no_tool') == NO_TOOL_ID,
                f'must be reserved ID {NO_TOOL_ID}')
    _tool_error(errors, 'tools.yaml.state_ids.unknown',
                state_ids.get('unknown') == UNKNOWN_TOOL_ID,
                f'must be reserved ID {UNKNOWN_TOOL_ID}')
    registered_ids = set()
    for key in tools:
        try:
            registered_ids.add(int(key))
        except (TypeError, ValueError):
            errors.append(f'tools.yaml.tools.{key}: integer tool ID required')
    _tool_error(errors, 'tools.yaml.tools',
                not registered_ids.intersection((NO_TOOL_ID, UNKNOWN_TOOL_ID)),
                'reserved NO_TOOL/UNKNOWN IDs must not be physical tools')
    _tool_error(errors, 'tools.yaml.tools',
                registered_ids == SUPPORTED_TOOL_IDS,
                'must contain exactly tool IDs 0 (gripper) and 1 (drill); IDs >= 2 are unsupported')
    for tool_id, expected_tag_id in TOOL_TAG_IDS.items():
        spec = tools.get(tool_id, tools.get(str(tool_id)))
        _tool_error(errors, f'tools.yaml.tools.{tool_id}.tag_id',
                    isinstance(spec, dict) and spec.get('tag_id') == expected_tag_id,
                    f'must be {expected_tag_id} for tool ID {tool_id}')

    detach_supported = real.get('physical_detach_supported')
    _tool_error(errors, 'tools.yaml.real_docking.physical_detach_supported',
                isinstance(detach_supported, bool), 'boolean required')
    requested = real.get('enabled_tool_ids')
    if not isinstance(requested, list) or not requested:
        errors.append('tools.yaml.real_docking.enabled_tool_ids: non-empty list of tool IDs required')
        requested = []
    elif any(not isinstance(tool_id, int) for tool_id in requested) or len(set(requested)) != len(requested):
        errors.append('tools.yaml.real_docking.enabled_tool_ids: unique integer tool IDs required')
        requested = []

    if any(tool_id in (NO_TOOL_ID, UNKNOWN_TOOL_ID) for tool_id in requested):
        errors.append('tools.yaml.real_docking.enabled_tool_ids: reserved state IDs are not tools')
    if any(tool_id not in SUPPORTED_TOOL_IDS for tool_id in requested):
        errors.append('tools.yaml.real_docking.enabled_tool_ids: only IDs 0 (gripper) and 1 (drill) are supported')

    fixtures = config.get('fixtures')
    if not isinstance(fixtures, dict) or not fixtures:
        errors.append('tools.yaml.fixtures: measured fixture map required')
        fixtures = {}

    enabled_ids = {int(key) for key, spec in tools.items()
                   if isinstance(key, (int, str)) and str(key).lstrip('-').isdigit()
                   and isinstance(spec, dict) and spec.get('enabled') is True}
    requested_ids = set(requested)
    for tool_id in sorted(enabled_ids - requested_ids):
        errors.append(f'tools.yaml.tools.{tool_id}.enabled: enabled tool must be listed in real_docking.enabled_tool_ids')
    for tool_id in requested:
        spec = tools.get(tool_id, tools.get(str(tool_id)))
        path = f'tools.yaml.tools.{tool_id}'
        if not isinstance(spec, dict):
            errors.append(f'{path}: requested tool is missing')
            continue
        _tool_error(errors, f'{path}.enabled', spec.get('enabled') is True, 'must be true for real docking')
        _tool_error(errors, f'{path}.tag_id', isinstance(spec.get('tag_id'), int), 'measured AprilTag integer required')
        _tool_error(errors, f'{path}.coarse_joint_goal', _number_list(spec.get('coarse_joint_goal'), 6),
                    'six homogeneous measured joint radians required')

        tcp = spec.get('tcp')
        _tool_error(errors, f'{path}.tcp', isinstance(tcp, dict) and bool(tcp.get('frame')) and bool(tcp.get('parent')) and
                    _number_list(tcp.get('xyz'), 3) and _number_list(tcp.get('quat_xyzw'), 4),
                    'frame, parent, xyz[m], quat_xyzw required')
        collision = spec.get('collision')
        _tool_error(errors, f'{path}.collision.primitives',
                    isinstance(collision, dict) and _primitive_list(collision.get('primitives')),
                    'measured tool collision primitives required')
        fixture_id = spec.get('dock_fixture')
        fixture = fixtures.get(fixture_id) if isinstance(fixture_id, str) else None
        _tool_error(errors, f'{path}.dock_fixture',
                    isinstance(fixture, dict) and bool(fixture.get('frame')) and _primitive_list(fixture.get('primitives')),
                    'must reference measured fixtures.<name>')

        lock = spec.get('lock')
        joint_path = lock.get('joint_path') if isinstance(lock, dict) else None
        path_valid = (isinstance(joint_path, list) and len(joint_path) >= 2 and
                      all(isinstance(step, dict) and isinstance(step.get('name'), str) and
                          _number_list(step.get('positions_rad'), 6) for step in joint_path))
        delta_valid = isinstance(lock, dict) and _number(lock.get('attach_yaw_delta_rad'))
        _tool_error(errors, f'{path}.lock',
                    isinstance(lock, dict) and lock.get('joint') == 'wrist_yaw_joint' and
                    (path_valid or delta_valid),
                    'measured joint_path or signed attach_yaw_delta_rad required')
        if detach_supported:
            _tool_error(errors, f'{path}.lock.joint_path', path_valid,
                        'encoder-taught path with at least unlock and lock steps required for reversible detach')
            rack_collision = spec.get('rack_collision')
            _tool_error(errors, f'{path}.rack_collision',
                        isinstance(rack_collision, dict) and bool(rack_collision.get('frame')) and
                        _primitive_list(rack_collision.get('primitives')),
                        'measured frame and primitives required for the released rack tool')

        docking = spec.get('docking')
        if not isinstance(docking, dict):
            errors.append(f'{path}.docking: measured descent/retreat map required')
            continue
        for phase in ('descent', 'retreat'):
            prefix = f'{path}.docking.{phase}'
            _tool_error(errors, f'{prefix}_frame', isinstance(docking.get(f'{phase}_frame'), str) and bool(docking.get(f'{phase}_frame')),
                        'coordinate frame required')
            _tool_error(errors, f'{prefix}_direction', _number_list(docking.get(f'{phase}_direction'), 3) and
                        math.sqrt(sum(x * x for x in docking[f'{phase}_direction'])) > 1e-9,
                        'non-zero unitless direction vector required')
            _tool_error(errors, f'{prefix}_distance_m', _number(docking.get(f'{phase}_distance_m')) and docking[f'{phase}_distance_m'] >= 0.0,
                        'measured distance in m required')
            _tool_error(errors, f'{prefix}_speed_mps', _positive(docking.get(f'{phase}_speed_mps')),
                        'positive measured speed in m/s required')
            _tool_error(errors, f'{prefix}_timeout_sec', _positive(docking.get(f'{phase}_timeout_sec')),
                        'positive timeout in s required')
    configured_tag_ids = []
    for tool_id in requested:
        spec = tools.get(tool_id, tools.get(str(tool_id)))
        if isinstance(spec, dict) and isinstance(spec.get('tag_id'), int):
            configured_tag_ids.append(spec['tag_id'])
    _tool_error(errors, 'tools.yaml.tools.<selected>.tag_id',
                len(configured_tag_ids) == len(set(configured_tag_ids)),
                'selected tools must use unique measured AprilTag IDs')
    return errors


def validate_docking(config: dict) -> list[str]:
    errors: list[str] = []
    vision = config.get('vision_apriltag', {}).get('ros__parameters', {})
    tag = config.get('tag_pose_filter', {}).get('ros__parameters', {})
    servo = config.get('visual_servo_node', {}).get('ros__parameters', {})
    executor = config.get('docking_motion_executor', {}).get('ros__parameters', {})
    if not isinstance(vision, dict):
        errors.append('docking.yaml.vision_apriltag.ros__parameters: required map')
        vision = {}
    if not isinstance(tag, dict):
        return errors + ['docking.yaml.tag_pose_filter.ros__parameters: required map']
    if not isinstance(servo, dict):
        errors.append('docking.yaml.visual_servo_node.ros__parameters: required map')
        servo = {}
    if not isinstance(executor, dict):
        errors.append('docking.yaml.docking_motion_executor.ros__parameters: required map')
        executor = {}

    def require(path, condition, description):
        _tool_error(errors, f'docking.yaml.{path}', condition, description)

    for key in ('input_topic', 'camera_info_topic', 'centers_topic'):
        require(f'vision_apriltag.ros__parameters.{key}', isinstance(vision.get(key), str) and bool(vision.get(key)), 'non-empty ROS topic required')
    require('vision_apriltag.ros__parameters.tag_size_cm', _positive(vision.get('tag_size_cm')),
            'measured printed tag side in cm required')
    allowed_tag_ids = vision.get('allowed_tag_ids')
    require('vision_apriltag.ros__parameters.allowed_tag_ids',
            isinstance(allowed_tag_ids, list) and allowed_tag_ids == [0, 1],
            'must be exactly [0, 1] (gripper, drill); tag IDs >= 2 are unsupported')

    for key in ('source_topic', 'depth_topic', 'selected_tool_id_topic', 'valid_pose_topic', 'status_topic', 'expected_frame_id'):
        require(f'tag_pose_filter.ros__parameters.{key}', isinstance(tag.get(key), str) and bool(tag.get(key)), 'non-empty topic/frame required')
    require('tag_pose_filter.ros__parameters.max_age_sec', _positive(tag.get('max_age_sec')), 'positive s required')
    require('tag_pose_filter.ros__parameters.min_decision_margin', _positive(tag.get('min_decision_margin')), 'measured positive margin required')
    depth_range = tag.get('depth_range_m')
    require('tag_pose_filter.ros__parameters.depth_range_m', _ros_number_list(depth_range, 2) and 0.0 < depth_range[0] < depth_range[1],
            'measured homogeneous numeric [min,max] m required')
    require('tag_pose_filter.ros__parameters.max_depth_age_sec', _positive(tag.get('max_depth_age_sec')), 'positive s required')
    require('tag_pose_filter.ros__parameters.max_depth_pnp_error_m', _positive(tag.get('max_depth_pnp_error_m')),
            'measured positive m tolerance required')
    require('tag_pose_filter.ros__parameters.depth_window_radius_px',
            isinstance(tag.get('depth_window_radius_px'), int) and tag['depth_window_radius_px'] >= 0,
            'nonnegative pixel radius required')

    for key in ('valid_pose_topic', 'twist_topic'):
        require(f'visual_servo_node.ros__parameters.{key}', isinstance(servo.get(key), str) and bool(servo.get(key)), 'non-empty topic required')
    require('visual_servo_node.ros__parameters.desired_tag_position_m', _ros_number_list(servo.get('desired_tag_position_m'), 3),
            'homogeneous numeric target [x,y,z] m in expected_frame_id required')
    for key in ('kp_xy', 'max_linear_velocity_mps', 'rate_hz', 'xy_tolerance_m', 'detection_timeout_sec'):
        require(f'visual_servo_node.ros__parameters.{key}', _positive(servo.get(key)), 'positive configured value required')
    require('visual_servo_node.ros__parameters.stable_frame_count',
            isinstance(servo.get('stable_frame_count'), int) and servo['stable_frame_count'] > 0, 'positive frame count required')

    for key in ('tool_id_topic', 'motion_command_topic', 'active_tool_id_topic', 'cancel_topic', 'status_topic', 'detection_topic',
                'servo_enable_service', 'servo_aligned_topic', 'servo_twist_topic', 'hardware_fault_topic',
                'planning_group', 'base_link', 'end_effector_link', 'joint_state_topic'):
        require(f'docking_motion_executor.ros__parameters.{key}', isinstance(executor.get(key), str) and bool(executor.get(key)),
                'non-empty ROS name required')
    require('docking_motion_executor.ros__parameters.joint_names',
            isinstance(executor.get('joint_names'), list) and len(executor['joint_names']) == 6 and
            all(isinstance(x, str) and x for x in executor['joint_names']),
            'six joint names required')
    require('docking_motion_executor.ros__parameters.docking_wait_joint_goal',
            _ros_number_list(executor.get('docking_wait_joint_goal'), 6), 'six measured joint radians required')
    mission = executor.get('mission_wait_joint_goal_yaml')
    try:
        mission_values = yaml.safe_load(mission)
    except yaml.YAMLError:
        mission_values = None
    require('docking_motion_executor.ros__parameters.mission_wait_joint_goal_yaml',
            _number_list(mission_values, 6), 'six measured joint radians YAML list required')
    for key in ('servo_rate_hz', 'coarse_timeout_sec', 'xy_align_timeout_sec', 'detection_timeout_sec',
                'lock_timeout_sec', 'lock_feedback_min_delta_rad', 'lock_feedback_timeout_sec',
                'attachment_timeout_sec', 'post_lock_motion_timeout_sec'):
        require(f'docking_motion_executor.ros__parameters.{key}', _positive(executor.get(key)), 'positive s/Hz required')
    require('topic_contract.vision_to_tag_filter',
            vision.get('centers_topic') == tag.get('source_topic'), 'AprilTag centers topic must match')
    require('topic_contract.tag_pose_filter_to_visual_servo',
            tag.get('valid_pose_topic') == servo.get('valid_pose_topic'), 'valid pose topics must match')
    require('topic_contract.visual_servo_to_executor',
            servo.get('twist_topic') == executor.get('servo_twist_topic'), 'twist topics must match')
    require('topic_contract.tag_filter_to_executor',
            tag.get('valid_pose_topic') == executor.get('detection_topic'), 'detection topics must match')
    return errors


def validate_all(hardware: dict, tools: dict, docking: dict) -> list[str]:
    return validate_hardware(hardware) + validate_tools(tools) + validate_docking(docking)


def _load(path: Path) -> dict:
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def main() -> int:
    parser = argparse.ArgumentParser(description='Fail-closed preflight for real arm control and attachment docking.')
    parser.add_argument('--hardware', required=True, type=Path)
    parser.add_argument('--tools', required=True, type=Path)
    parser.add_argument('--docking', required=True, type=Path)
    args = parser.parse_args()
    try:
        errors = validate_all(_load(args.hardware), _load(args.tools), _load(args.docking))
    except (OSError, yaml.YAMLError) as exc:
        print(f'CONFIG NOT READY: {exc}', file=sys.stderr)
        return 2
    if errors:
        print('CONFIG NOT READY — real control and docking launch are blocked:', file=sys.stderr)
        print('\n'.join(f'- {error}' for error in errors), file=sys.stderr)
        return 1
    print('CONFIG READY: hardware, selected tools, tag/depth gate, servo, and attachment motion are complete.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
