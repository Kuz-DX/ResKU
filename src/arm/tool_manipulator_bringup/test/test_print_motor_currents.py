import math
import pathlib
import sys


SCRIPTS = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from print_motor_currents import dxl_raw_to_amp, format_currents, signed_int16


def test_signed_int16():
    assert signed_int16(0x0001) == 1
    assert signed_int16(0xFFFF) == -1
    assert signed_int16(0x8000) == -32768


def test_model_specific_dxl_current_conversion():
    assert math.isclose(dxl_raw_to_amp(100, 5506.43, 2047), 0.269)
    assert math.isclose(dxl_raw_to_amp(100, 6877.92, 2047), 0.336)


def test_format_currents_uses_absolute_total_and_marks_missing():
    line = format_currents({"base_joint": -0.2, "shoulder_joint": 0.3})
    assert "base_joint=-0.200 A" in line
    assert "shoulder_joint=+0.300 A" in line
    assert "elbow_joint=N/A" in line
    assert "TOTAL(abs)=0.500 A (2/7)" in line
