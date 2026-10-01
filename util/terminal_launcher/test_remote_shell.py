"""Exercise Ctrl+C and readline through a real PTY without SSH or ROS."""

import os
from pathlib import Path
import pty
import select
import signal
import tempfile
import time
import unittest

from remote_shell import remote_rc


class RemoteShellTest(unittest.TestCase):
    def test_interrupt_preserves_environment_and_up_repeats_command(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / '.bashrc').write_text(
                "PS1='RESKU_READY> '\n"
                "alias sr='export TEST_ROS=loaded'\n"
                "alias si='export TEST_WS=loaded'\n"
            )
            rc = root / 'rc'
            command = "printf 'NODE_RUNNING\\n'; sleep 60"
            rc.write_text(remote_rc(f'cd {directory} && sr && si', command))
            pid, fd = pty.fork()
            if pid == 0:
                os.environ.update(HOME=directory, RESKU_REMOTE_RC=str(rc), TERM='xterm')
                os.execv('/bin/bash', ['bash', '--noprofile', '--rcfile', str(rc), '-i'])

            def wait_for(expected):
                output = b''
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    if select.select([fd], [], [], 0.1)[0]:
                        output += os.read(fd, 65536)
                        if expected in output:
                            return output
                self.fail(f'Missing {expected!r}: {output!r}')

            try:
                self.assertIn(b'\x1b]11;#008700\x07', wait_for(b'NODE_RUNNING\r\n'))
                time.sleep(0.15)  # Allow sleep to take foreground ownership.
                os.write(fd, b'\x03')
                self.assertIn(b'\x1b]11;#5f0000\x07', wait_for(b'RESKU_READY> '))
                self.assertFalse(rc.exists())
                os.write(fd, b'\x1b[A')
                wait_for(command.encode())
                os.write(fd, b'\r')
                self.assertIn(b'\x1b]11;#008700\x07', wait_for(b'NODE_RUNNING\r\n'))
                time.sleep(0.15)  # Allow sleep to take foreground ownership.
                os.write(fd, b'\x03')
                self.assertIn(b'\x1b]11;#5f0000\x07', wait_for(b'RESKU_READY> '))
                os.write(fd, b'printf "ENV=%s/%s CWD=%s\\n" "$TEST_ROS" "$TEST_WS" "$PWD"\r')
                wait_for(f'ENV=loaded/loaded CWD={directory}\r\n'.encode())
                self.assertIn(command, (root / '.bash_history').read_text())
            finally:
                os.kill(pid, signal.SIGKILL)
                os.waitpid(pid, 0)
                os.close(fd)


if __name__ == '__main__':
    unittest.main()
