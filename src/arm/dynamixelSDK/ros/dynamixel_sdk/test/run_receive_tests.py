"""Standalone ASan/UBSan checks; compiles test binaries only, not ROS packages."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def main():
    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=0', UBSAN_OPTIONS='halt_on_error=1')
    # LeakSanitizer cannot run in ptrace-based environments; address and UB
    # checking remain enabled. No serial hardware is accessed.
    with tempfile.TemporaryDirectory(prefix='dxl-receive-tests-') as tmp:
        for test, source, libs in (
            ('protocol2_receive_bounds', 'protocol2_packet_handler', []),
            ('linux_port_receive', 'port_handler_linux', ['-lutil']),
        ):
            output = str(Path(tmp) / test)
            subprocess.run([
                'g++', '-std=c++11', '-g', '-fsanitize=address,undefined',
                '-fno-omit-frame-pointer', '-I', str(ROOT/'include/dynamixel_sdk'),
                str(ROOT/'test'/f'{test}.cpp'),
                str(ROOT/'src/dynamixel_sdk'/f'{source}.cpp'),
                *libs, '-o', output,
            ], check=True)
            subprocess.run([output], env=env, check=True)

if __name__ == '__main__':
    main()
