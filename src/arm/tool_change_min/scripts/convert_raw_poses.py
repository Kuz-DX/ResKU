#!/usr/bin/env python3
"""Offline capture conversion. Never connects to hardware or overwrites a file."""
import argparse
import math
from pathlib import Path
import sys

import yaml


def convert(capture, hardware, names, margin):
    if not math.isfinite(margin) or margin < 0:
        raise ValueError("ik_limit_margin_rad must be finite and nonnegative")
    expected = [*names, "ee_joint"]
    if capture["joint_names"] != expected:
        raise ValueError("capture joint order must match poses joint_names plus ee_joint")
    units = ["raw_pulse" if hardware["joints"][n]["vendor"] == "dynamixel"
             else "raw_deg" for n in expected]
    if capture["raw_units"] != units:
        raise ValueError("capture raw units do not match hardware vendors")
    converted, errors = {}, []
    for label, sample in capture["captures"].items():
        if len(sample["raw"]) != len(expected):
            raise ValueError(f"{label}: incorrect number of raw values")
        joints = {}
        for name, raw in zip(expected, sample["raw"]):
            spec = hardware["joints"][name]
            raw = float(raw)
            if not math.isfinite(raw):
                raise ValueError(f"{label}.{name}: nonfinite raw value")
            if spec["vendor"] == "dynamixel":
                if not raw.is_integer():
                    raise ValueError(f"{label}.{name}: raw_pulse must be an integer")
                sign = 1 if spec["raw_increases_ccw"] else -1
                q = sign * (raw - spec["zero_raw"]) * 2 * math.pi / 4096
                lo, hi = spec["soft_limit_raw"]
                if not lo <= raw <= hi:
                    errors.append(f"{label}.{name}: raw={raw:g} outside [{lo}, {hi}]")
            elif spec["vendor"] == "rmd":
                q = spec["sign"] * (math.radians(raw) - spec["q_offset_rad"])
            else:
                raise ValueError(f"unsupported vendor for {name}")
            lo, hi = spec["soft_limit_rad"]
            padding = margin if name in names else 0
            if not math.isfinite(q) or not lo + padding <= q <= hi - padding:
                errors.append(f"{label}.{name}: q={q:.6f} outside "
                              f"[{lo + padding:.6f}, {hi - padding:.6f}]")
            joints[name] = q
        converted[label] = joints
    return converted, errors


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", type=Path, default=package / "config/measured_raw_poses.yaml")
    parser.add_argument("--poses", type=Path, default=package / "config/poses.yaml")
    parser.add_argument("--hardware", type=Path, default=package.parent /
                        "tool_manipulator_bringup/config/hardware.yaml")
    parser.add_argument("--urdf", type=Path, default=package.parent /
                        "tool_manipulator_description/urdf/tool_manipulator.urdf.xacro")
    parser.add_argument("--output", type=Path, help="create a new candidate YAML only if checks pass")
    args = parser.parse_args()
    poses = yaml.safe_load(args.poses.read_text())
    converted, errors = convert(yaml.safe_load(args.captures.read_text()),
                                yaml.safe_load(args.hardware.read_text()),
                                poses["joint_names"], float(poses["motion"]["ik_limit_margin_rad"]))
    print(yaml.safe_dump({"converted_rad_for_review": converted}, sort_keys=False))
    if errors:
        for error in errors:
            print("BLOCKED: " + error, file=sys.stderr)
        print("No candidate written. Verify calibration and re-teach within limits.", file=sys.stderr)
        return 2
    # Import FK dependencies only after raw/radian validation succeeds.
    sys.path.insert(0, str(package))
    from tool_change_min.kinematics import ArmKinematics
    from scipy.spatial.transform import Rotation
    model = ArmKinematics(args.urdf, args.hardware)
    for label, joints in converted.items():
        poses["named_poses"][label] = {n: joints[n] for n in model.joint_names}
    docking = converted["tool1_docking"]
    transform = model.fk([docking[n] for n in model.joint_names])
    poses["cartesian_targets"]["tool1_target"] = {
        "frame_id": "base_actuator", "position_m": transform[:3, 3].tolist(),
        "orientation_xyzw": Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
    }
    lock_yaw = converted["tool_lock1"]["wrist_yaw_joint"]
    poses["named_poses"]["tool_lock"] = {
        "wrist_yaw_joint": lock_yaw,
        "delta_rad": lock_yaw - docking["wrist_yaw_joint"],
    }
    # Preserve staged execution, EE alignment settings and the commanded yaw delta.
    if args.output:
        with args.output.open("x", encoding="utf-8") as stream:
            yaml.safe_dump(poses, stream, sort_keys=False)
        print(f"Candidate written: {args.output}")
    print("Capture limits/FK checked; motion paths, EE alignment and collision clearance are not validated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
