#!/usr/bin/env python3
"""Commission a taught docking joint path from a small fail-closed UI."""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from pathlib import Path

import yaml


ARM_JOINTS = (
    "base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint",
    "wrist_roll_joint", "wrist_yaw_joint",
)
ALL_JOINTS = ARM_JOINTS + ("ee_joint",)


def package_config(filename: str) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("tool_manipulator_bringup")) / "config" / filename
    except Exception:
        return Path(__file__).resolve().parents[1] / "config" / filename


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: YAML root must be a mapping")
    return data


def load_and_validate(path_file: Path, hardware_file: Path, sequence_name: str) -> dict:
    paths = load_yaml(path_file)
    hardware = load_yaml(hardware_file)
    if tuple(paths.get("joint_names", ())) != ALL_JOINTS:
        raise ValueError(f"joint_names must be exactly {list(ALL_JOINTS)}")
    sequence = (paths.get("sequences") or {}).get(sequence_name)
    if not isinstance(sequence, dict):
        raise ValueError(f"unknown sequence '{sequence_name}'")
    waypoints = sequence.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError(f"{sequence_name}.waypoints must be a non-empty list")
    speed = float(sequence.get("max_joint_speed_rad_s", 0.0))
    minimum = float(sequence.get("min_segment_duration_sec", 0.0))
    tolerance = float(sequence.get("start_tolerance_rad", 0.0))
    if not all(math.isfinite(value) and value > 0.0 for value in (speed, minimum, tolerance)):
        raise ValueError("speed, minimum duration, and start tolerance must be finite and positive")

    limits = {}
    hardware_joints = hardware.get("joints") or {}
    for joint_name in ALL_JOINTS:
        raw_limit = (hardware_joints.get(joint_name) or {}).get("soft_limit_rad")
        if not isinstance(raw_limit, list) or len(raw_limit) != 2:
            raise ValueError(f"hardware limit missing: {joint_name}.soft_limit_rad")
        lower, upper = map(float, raw_limit)
        if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
            raise ValueError(f"invalid hardware limit: {joint_name}={raw_limit}")
        limits[joint_name] = (lower, upper)

    violations = []
    names = set()
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict) or not str(waypoint.get("name", "")).strip():
            raise ValueError(f"waypoint {index}: non-empty name required")
        name = str(waypoint["name"])
        if name in names:
            raise ValueError(f"duplicate waypoint name: {name}")
        names.add(name)
        positions = waypoint.get("positions_rad")
        if not isinstance(positions, list) or len(positions) != len(ALL_JOINTS):
            raise ValueError(f"{name}: exactly {len(ALL_JOINTS)} positions required")
        positions = [float(value) for value in positions]
        if not all(math.isfinite(value) for value in positions):
            raise ValueError(f"{name}: all positions must be finite")
        waypoint["positions_rad"] = positions
        for joint_name, value in zip(ALL_JOINTS, positions):
            lower, upper = limits[joint_name]
            if value < lower or value > upper:
                violations.append(
                    f"{name}.{joint_name}={value:.6f} outside [{lower:.6f}, {upper:.6f}]"
                )
    if violations:
        raise ValueError("path exceeds hardware soft limits:\n  " + "\n  ".join(violations))
    stages = sequence.get("stages", [])
    if not isinstance(stages, list):
        raise ValueError("stages must be a list")
    expected_start = 1
    for stage in stages:
        if not isinstance(stage, dict):
            raise ValueError("each stage must be a mapping")
        start = int(stage.get("start_index", -1))
        end = int(stage.get("end_index", -1))
        if start != expected_start or end < start or end >= len(waypoints):
            raise ValueError(
                f"invalid/non-contiguous stage {stage.get('name')}: start={start}, end={end}"
            )
        if not str(stage.get("label", "")).strip():
            raise ValueError(f"stage {stage.get('name')}: label required")
        expected_start = end + 1
    if stages and expected_start != len(waypoints):
        raise ValueError("stages must cover every waypoint after home_start")
    sequence["name"] = sequence_name
    return sequence


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a taught docking path with manual UI controls.")
    parser.add_argument("--sequence", default="tagid0_gripper")
    parser.add_argument("--paths", type=Path, default=None)
    parser.add_argument("--hardware", type=Path, default=None)
    parser.add_argument("--wait-for-server", type=float, default=10.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def seconds_to_duration(seconds: float):
    from builtin_interfaces.msg import Duration
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1_000_000_000)))


def future_result(future, timeout: float):
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    if not done.wait(timeout):
        raise TimeoutError("ROS action response timeout")
    return future.result()


class PathRunner:
    def __init__(self, sequence: dict, wait_for_server: float, events: queue.Queue):
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        class RunnerNode(Node):
            pass

        self.rclpy = rclpy
        self.action_type = FollowJointTrajectory
        self.node = RunnerNode("manual_docking_path_ui")
        self.arm_client = ActionClient(
            self.node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.ee_client = ActionClient(
            self.node, FollowJointTrajectory, "/ee_controller/follow_joint_trajectory")
        self.node.create_subscription(JointState, "/joint_states", self._joint_state_cb, 20)
        self.sequence = sequence
        self.wait_for_server = wait_for_server
        self.events = events
        self.latest = None
        self.latest_time = None
        self.latest_lock = threading.Lock()
        self.run_lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.goal_handles = []
        self.next_index = 0

    def _joint_state_cb(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions and math.isfinite(float(positions[name])) for name in ALL_JOINTS):
            with self.latest_lock:
                self.latest = [float(positions[name]) for name in ALL_JOINTS]
                self.latest_time = time.monotonic()

    def emit(self, text: str):
        self.events.put(text)
        self.node.get_logger().info(text)

    def start(self, mode: str, bounds=None):
        if not self.run_lock.acquire(blocking=False):
            self.emit("이미 경로를 실행 중입니다")
            return
        threading.Thread(target=self._run, args=(mode, bounds), daemon=True).start()

    def cancel(self):
        self.cancel_event.set()
        for handle in list(self.goal_handles):
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        self.emit("중지 요청 전송: controller 결과 확인 중")

    def _run(self, mode: str, bounds=None):
        try:
            self.cancel_event.clear()
            waypoints = self.sequence["waypoints"]
            if mode == "start":
                indices = [0]
            elif mode == "next":
                indices = [self.next_index] if self.next_index < len(waypoints) else []
            elif mode == "remaining":
                if self.next_index == 0:
                    raise RuntimeError("먼저 '시작 자세로 이동'을 실행해야 합니다")
                indices = list(range(self.next_index, len(waypoints)))
            elif mode == "stage":
                start, end = bounds
                if self.next_index != start:
                    raise RuntimeError(
                        f"단계 순서 오류: 현재 다음 자세={self.next_index}, 필요={start}"
                    )
                indices = list(range(start, end + 1))
            else:
                raise ValueError(f"unknown run mode: {mode}")
            if not indices:
                self.emit("모든 웨이포인트 실행 완료")
                return
            for index in indices:
                if self.cancel_event.is_set():
                    raise RuntimeError("사용자 중지")
                self._move(index)
                self.next_index = index + 1
            self.emit(f"완료: 다음 단계 {self.next_index}/{len(waypoints)}")
        except Exception as exc:
            self.cancel()
            self.emit(f"실패/중지: {exc}")
        finally:
            self.goal_handles = []
            self.run_lock.release()

    def _move(self, index: int):
        from action_msgs.msg import GoalStatus
        from trajectory_msgs.msg import JointTrajectoryPoint

        with self.latest_lock:
            current = None if self.latest is None else list(self.latest)
            sample_time = self.latest_time
        if current is None:
            raise RuntimeError("/joint_states의 7개 관절값을 아직 받지 못했습니다")
        if sample_time is None or time.monotonic() - sample_time > 1.0:
            raise RuntimeError("/joint_states가 1초 이상 갱신되지 않았습니다")
        if not self.arm_client.wait_for_server(timeout_sec=self.wait_for_server):
            raise RuntimeError("arm_controller action server가 없습니다")
        if not self.ee_client.wait_for_server(timeout_sec=self.wait_for_server):
            raise RuntimeError("ee_controller action server가 없습니다")

        waypoint = self.sequence["waypoints"][index]
        target = waypoint["positions_rad"]
        speed = float(self.sequence["max_joint_speed_rad_s"])
        duration = max(
            float(self.sequence["min_segment_duration_sec"]),
            max(abs(goal - actual) for goal, actual in zip(target, current)) / speed,
        )
        self.emit(
            f"실행 {index + 1}/{len(self.sequence['waypoints'])}: "
            f"{waypoint['name']} ({duration:.2f}s)"
        )

        arm_goal = self.action_type.Goal()
        arm_goal.trajectory.joint_names = list(ARM_JOINTS)
        arm_goal.trajectory.points = [JointTrajectoryPoint(
            positions=target[:6], time_from_start=seconds_to_duration(duration))]
        ee_goal = self.action_type.Goal()
        ee_goal.trajectory.joint_names = ["ee_joint"]
        ee_goal.trajectory.points = [JointTrajectoryPoint(
            positions=[target[6]], time_from_start=seconds_to_duration(duration))]

        arm_future = self.arm_client.send_goal_async(arm_goal)
        ee_future = self.ee_client.send_goal_async(ee_goal)
        arm_handle = future_result(arm_future, self.wait_for_server)
        self.goal_handles = [arm_handle] if arm_handle is not None else []
        ee_handle = future_result(ee_future, self.wait_for_server)
        if ee_handle is not None:
            self.goal_handles.append(ee_handle)
        if arm_handle is None or not arm_handle.accepted or ee_handle is None or not ee_handle.accepted:
            self.cancel()
            raise RuntimeError("arm 또는 ee controller가 목표를 거부했습니다")
        timeout = duration + 5.0
        arm_result = future_result(arm_handle.get_result_async(), timeout)
        ee_result = future_result(ee_handle.get_result_async(), timeout)
        self.goal_handles = []
        if arm_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"arm trajectory 실패(status={arm_result.status})")
        if ee_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"ee trajectory 실패(status={ee_result.status})")


def run_ui(sequence: dict, wait_for_server: float) -> int:
    import tkinter as tk
    from tkinter import messagebox
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    events = queue.Queue()
    rclpy.init()
    runner = PathRunner(sequence, wait_for_server, events)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(runner.node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title("수동 학습 경로 검증")
    status = tk.StringVar(value="준비: 먼저 시작 자세 이동을 실행하세요")
    identity = sequence["name"]
    if "tool_id" in sequence and "tag_id" in sequence:
        identity += f" / tool {sequence['tool_id']} / tag {sequence['tag_id']}"
    tk.Label(root, text=identity,
             font=("Sans", 13, "bold")).pack(padx=18, pady=(16, 5))
    tk.Label(root, text="캡처된 관절각을 순차 실행합니다. 장애물을 제거하고 비상정지를 준비하세요.",
             fg="#9b1c1c").pack(padx=18, pady=5)
    tk.Label(root, textvariable=status, width=72, anchor="w", justify="left").pack(padx=18, pady=10)

    def start_remaining():
        if messagebox.askyesno("경로 실행 확인", "현재 단계부터 남은 전체 경로를 실행할까요?"):
            runner.start("remaining")

    tk.Button(root, text="1. 시작 자세로 이동", width=34,
              command=lambda: runner.start("start")).pack(pady=3)
    for stage in sequence.get("stages", []):
        tk.Button(
            root, text=stage["label"], width=34,
            command=lambda item=stage: runner.start(
                "stage", (int(item["start_index"]), int(item["end_index"]))
            ),
        ).pack(pady=3)
    tk.Button(root, text="웨이포인트 한 개 실행", width=34,
              command=lambda: runner.start("next")).pack(pady=3)
    tk.Button(root, text="검증 후 남은 전체 경로 실행", width=34,
              command=start_remaining).pack(pady=3)
    tk.Button(root, text="중지 / 현재 목표 취소", width=34, bg="#c62828", fg="white",
              command=runner.cancel).pack(pady=(10, 16))

    def pump_events():
        try:
            while True:
                status.set(events.get_nowait())
        except queue.Empty:
            pass
        if rclpy.ok():
            root.after(100, pump_events)

    def close():
        runner.cancel()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    root.after(100, pump_events)
    try:
        root.mainloop()
    finally:
        executor.shutdown(timeout_sec=2.0)
        runner.node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    return 0


def main() -> int:
    args = parse_args()
    if args.wait_for_server < 0.0:
        print("--wait-for-server must be >= 0", file=sys.stderr)
        return 2
    paths = (args.paths or package_config("manual_docking_paths.yaml")).resolve()
    hardware = (args.hardware or package_config("hardware.yaml")).resolve()
    try:
        sequence = load_and_validate(paths, hardware, args.sequence)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f"경로 검증 실패: {exc}", file=sys.stderr)
        return 2
    print(f"sequence: {sequence['name']} ({len(sequence['waypoints'])} waypoints)")
    print(f"paths: {paths}\nhardware: {hardware}")
    if args.dry_run:
        for index, waypoint in enumerate(sequence["waypoints"], 1):
            print(f"{index:02d} {waypoint['name']}: {waypoint['positions_rad']}")
        return 0
    return run_ui(sequence, args.wait_for_server)


if __name__ == "__main__":
    raise SystemExit(main())
