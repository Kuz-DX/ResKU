"""EE alignment: q=s*(pulse-zero)*2*pi/4096; candidates are ref+n*pi.

Select the nearest candidate inside the agreed raw/URDF/radian limits.
The current position must itself be inside those limits; no recovery move is
inferred for an out-of-range encoder. This checks joint bounds, not collision.
"""
import math
import xml.etree.ElementTree as ET

import xacro
import yaml


class EeAlignment:
    def __init__(self, urdf, hardware):
        root = ET.fromstring(xacro.process_file(
            str(urdf), mappings={"use_mesh": "false", "tool_id": "0"}).toxml())
        limit = root.find("./joint[@name='ee_joint']/limit")
        if limit is None:
            raise ValueError("URDF missing ee_joint limits")
        with open(hardware, encoding="utf-8") as stream:
            spec = yaml.safe_load(stream)["joints"]["ee_joint"]
        if spec["vendor"] != "dynamixel" or not isinstance(spec["raw_increases_ccw"], bool):
            raise ValueError("invalid EE vendor/direction")
        self.zero = float(spec["zero_raw"])
        self.sign = 1.0 if spec["raw_increases_ccw"] else -1.0
        self.lower, self.upper = float(limit.get("lower")), float(limit.get("upper"))
        raw_limits = sorted(self.to_rad(float(p)) for p in spec["soft_limit_raw"])
        for limits in (raw_limits, spec["soft_limit_rad"]):
            if not all(math.isclose(float(a), b, abs_tol=1e-10, rel_tol=0)
                       for a, b in zip(limits, (self.lower, self.upper))):
                raise ValueError(f"EE raw/hardware/URDF limit mismatch: {limits}")
        self.velocity = float(spec["velocity_limit_rad_s"])
        if not 0 < self.velocity <= float(limit.get("velocity")):
            raise ValueError("EE velocity limit invalid")

    def to_rad(self, pulse):
        return self.sign * (pulse - self.zero) * math.tau / 4096

    def target(self, current, reference_raw, margin):
        if not all(math.isfinite(v) for v in (current, reference_raw, margin)) or margin < 0:
            raise ValueError("invalid EE alignment input")
        lower, upper = self.lower + margin, self.upper - margin
        if not lower <= current <= upper:
            raise ValueError("current EE position outside soft limits; no recovery motion")
        reference = self.to_rad(reference_raw)
        first = math.ceil((lower - reference) / math.pi)
        last = math.floor((upper - reference) / math.pi)
        if first > last:
            raise ValueError("no EE alignment candidate inside limits")
        index = min(max(round((current - reference) / math.pi), first), last)
        return reference + index * math.pi
