#!/usr/bin/env python3
"""Parameter-driven horizontal gravity screening; no hardware commands."""
import copy
import importlib.util
import csv
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from ament_index_python.packages import get_package_share_directory, get_package_prefix
# symlink-install에서는 Python의 검색 경로가 원본 scripts/를 가리킨다.
# 따라서 패키지 설치 위치에서 계산 모듈을 명시적으로 로드한다.
_sizing_path = Path(get_package_prefix('tool_manipulator_bringup')) / 'lib/tool_manipulator_bringup/design_sizing.py'
_spec = importlib.util.spec_from_file_location('tool_design_sizing', _sizing_path)
_sizing = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_sizing)
calculate = _sizing.calculate
configure_extra_pitch = _sizing.configure_extra_pitch
resize_link = _sizing.resize_link


class Monitor(Node):
    def __init__(self):
        super().__init__('design_load_monitor')
        share = Path(get_package_share_directory('tool_manipulator_bringup'))
        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter('config', str(share / 'design/current_arm.example.json'), readonly)
        self.declare_parameter('csv_path', str(Path.cwd() / f'design_load_{time.time_ns()}.csv'), readonly)
        self.base = json.loads(Path(self.get_parameter('config').value).read_text())
        for name, value in [('L1', .18), ('L2', .22), ('L3', .15), ('extra_pitch', 0.),
                            ('safety_factor', float(self.base['safety_factor']))]:
            self.declare_parameter(name, value)
        self.values = {name: self.get_parameter(name).value for name in
                       ('L1','L2','L3','extra_pitch','safety_factor')}
        # CSV는 시작과 파라미터 변경 시에만 기록한다.
        path = Path(self.get_parameter('csv_path').value).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open('x', newline='')  # 기존 기록 덮어쓰기를 방지한다.
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time_ns','L1_m','L2_m','L3_m','extra_pitch_m','safety_factor',
                              'joint','gravity_nm','factored_nm','limit_nm','basis','exceeded'])
        self.emit(self.values, self.evaluate(self.values))
        self.add_on_set_parameters_callback(self.change)
        self.get_logger().info(f'CSV: {path}; static estimates, not measured operating torque')

    def evaluate(self, values):
        for name, lo, hi in [('L1',.16,.20),('L2',.18,.26),('L3',.13,.17),('extra_pitch',0.,.1)]:
            if not lo <= values[name] <= hi:
                raise ValueError(f'{name}: allowed {lo}..{hi} m')
        config = copy.deepcopy(self.base)
        for parameter, joint in [('L1','shoulder'),('L2','elbow'),('L3','wrist')]:
            config = resize_link(config, joint, values[parameter], 'fixed')
        config = configure_extra_pitch(config, values['extra_pitch'])
        config['safety_factor'] = values['safety_factor']
        for joint in config['segments']:
            if joint['name'] == 'extra_pitch':
                joint['continuous_output_nm'] = 4.2  # User limit, NOT manufacturer continuous rating.
        return calculate(config)

    def emit(self, values, report):
        stamp = time.time_ns()
        lines = [f"lengths(m): L1={values['L1']:.3f} L2={values['L2']:.3f} L3={values['L3']:.3f} extra={values['extra_pitch']:.3f}"]
        for row in report['joints']:
            limit = row['continuous_output_nm']
            basis = 'USER_0.5_STALL' if row['joint'] == 'extra_pitch' else 'USER_REPORTED_RATED'
            exceeded = 'UNKNOWN' if limit is None else str(row['factored_static_nm'] > limit)
            lines.append(f"{row['joint']}: gravity={row['gravity_nm']:.3f} Nm SF={row['factored_static_nm']:.3f} Nm limit={limit} [{basis}] exceeded={exceeded}")
            self.writer.writerow([stamp,*[values[n] for n in ('L1','L2','L3','extra_pitch','safety_factor')],
                                  row['joint'],row['gravity_nm'],row['factored_static_nm'],limit,basis,exceeded])
        self.file.flush()
        self.get_logger().info('\n'.join(lines))

    def change(self, parameters):
        proposed = dict(self.values)
        for parameter in parameters:
            if parameter.name in proposed:
                proposed[parameter.name] = parameter.value
        try:
            report = self.evaluate(proposed)
        except (ValueError, TypeError, KeyError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        if proposed != self.values:
            self.emit(proposed, report)
            self.values = proposed
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    finally:
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
