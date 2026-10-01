"""Exercise Ctrl+C and readline through a real PTY without SSH or ROS."""

import os
from pathlib import Path
import pty
import select
import signal
import tempfile
import time
import unittest
from unittest import mock

from remote_shell import PaneColourForwarder, remote_rc, run_ssh


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
            colours = root / 'colours'
            tmux = root / 'tmux'
            tmux.write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$TEST_COLOUR_LOG"\n')
            tmux.chmod(0o755)
            command = "printf 'NODE_RUNNING\\n'; sleep 60"
            rc.write_text(remote_rc(f'cd {directory} && sr && si', command))
            pid, fd = pty.fork()
            if pid == 0:
                os.environ.update(HOME=directory, RESKU_REMOTE_RC=str(rc), TERM='xterm')
                os.environ.update(
                    TMUX='test', TMUX_PANE='%42', TEST_COLOUR_LOG=str(colours),
                    PATH=directory + ':' + os.environ['PATH'],
                )
                # Use the actual local PTY observer, replacing SSH only with Bash.
                status = run_ssh(['/bin/bash', '--noprofile', '--rcfile', str(rc), '-i'])
                os._exit(status)

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
                changes = colours.read_text()
                self.assertIn('window-style bg=colour52', changes)
                self.assertIn('window-active-style bg=colour52', changes)
                self.assertIn('window-style bg=colour28', changes)
                self.assertGreaterEqual(changes.count('window-style bg=colour52'), 2)
            finally:
                os.kill(pid, signal.SIGKILL)
                os.waitpid(pid, 0)
                os.close(fd)


class ColourForwarderTest(unittest.TestCase):
    def test_escape_split_between_reads_and_repeated_status(self):
        forwarder = PaneColourForwarder('%7')
        with mock.patch('remote_shell.subprocess.run') as run:
            forwarder.feed(b'log\x1b]11;#5f')
            run.assert_not_called()
            forwarder.feed(b'0000\x07prompt')
            forwarder.feed(b'\x1b]11;#5f0000\x07')
            self.assertEqual(run.call_count, 2)
            forwarder.feed(b'\x1b]11;#008700\x07')
            self.assertEqual(run.call_count, 4)
            self.assertEqual(run.call_args.args[0][-1], 'bg=colour28')


if __name__ == '__main__':
    unittest.main()
