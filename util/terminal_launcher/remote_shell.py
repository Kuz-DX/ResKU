#!/usr/bin/env python3
"""Run a command over SSH, then keep its initialized interactive Bash alive."""

import argparse
import os
import fcntl
import pty
import re
import shlex
import signal
import struct
import subprocess
import sys
import termios


class PaneColourForwarder:
    """Apply remote shell status to local tmux, even with explicit pane styles."""

    pattern = re.compile(rb'\x1b\]11;#(5f0000|008700)\x07')

    def __init__(self, pane: str):
        self.pane = pane
        self.tail = b''
        self.colour = None

    def feed(self, data: bytes) -> None:
        buffered = self.tail + data
        for match in self.pattern.finditer(buffered):
            colour = 'colour52' if match[1] == b'5f0000' else 'colour28'
            if colour == self.colour:
                continue
            for option in ('window-style', 'window-active-style'):
                subprocess.run(
                    ['tmux', 'set-option', '-p', '-q', '-t', self.pane,
                     option, f'bg={colour}'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    check=False,
                )
            self.colour = colour
        # Keep only an incomplete escape sequence across read boundaries.
        start = buffered.rfind(b'\x1b')
        self.tail = buffered[start:] if start >= 0 and b'\x07' not in buffered[start:] else b''
        self.tail = self.tail[-32:]


def run_ssh(argv: list[str]) -> int:
    pane = os.environ.get('TMUX_PANE')
    if not pane or not os.environ.get('TMUX') or not sys.stdin.isatty():
        os.execvp(argv[0], argv)
    forwarder = PaneColourForwarder(pane)
    master_fd = None

    def resize(*_):
        if master_fd is not None:
            try:
                size = fcntl.ioctl(0, termios.TIOCGWINSZ, struct.pack('HHHH', 0, 0, 0, 0))
                fcntl.ioctl(master_fd, termios.TIOCSWINSZ, size)
            except OSError:
                pass

    def read_remote(fd):
        nonlocal master_fd
        if master_fd is None:
            master_fd = fd
            resize()
        data = os.read(fd, 65536)
        forwarder.feed(data)
        return data

    previous_handler = signal.signal(signal.SIGWINCH, resize)
    try:
        # Raw PTY forwarding sends Ctrl+C to the remote foreground process,
        # while the local observer stays alive to see the next shell prompt.
        return os.waitstatus_to_exitcode(pty.spawn(argv, master_read=read_remote))
    finally:
        signal.signal(signal.SIGWINCH, previous_handler)
        forwarder.feed(b'\x1b]11;#5f0000\x07')


def remote_rc(setup: str, command: str) -> str:
    return f'''# The bootstrap file is private and only needed while sourcing it.
command rm -f -- "$RESKU_REMOTE_RC"
unset RESKU_REMOTE_RC
[[ ! -r ~/.bashrc ]] || source ~/.bashrc
shopt -s histappend
set -o history
# The local SSH observer applies these OSC 11 colours to the tmux pane styles.
# PS1 also runs after Ctrl+C interrupts PROMPT_COMMAND's initial command.
PS1='\\[\\e]11;#5f0000\\a\\]'"${{PS1-}}"
PS0=$'\\e]11;#008700\\a'"${{PS0-}}"
if ! eval {shlex.quote(setup)}; then
    printf '\\nResKU: environment setup failed; command was not started.\\n' >&2
    return
fi
__resku_remote_start_once() {{
    # Remove this hook before running the foreground command: Ctrl+C must
    # return to a normal prompt, without restarting the command automatically.
    PROMPT_COMMAND=("${{PROMPT_COMMAND[@]:1}}")
    builtin history -s -- {shlex.quote(command)}
    builtin history -a
    printf '\\033]11;#008700\\007'
    eval {shlex.quote(command)}
}}
PROMPT_COMMAND=(__resku_remote_start_once "${{PROMPT_COMMAND[@]}}")
'''


def ssh_argv(host: str, setup: str, command: str) -> list[str]:
    script = (
        'resku_rc=$(mktemp /tmp/resku-shell.XXXXXXXX) || exit 1\n'
        'trap \'rm -f -- "$resku_rc"\' EXIT\n'
        f'printf %s {shlex.quote(remote_rc(setup, command))} > "$resku_rc" || exit 1\n'
        'export RESKU_REMOTE_RC="$resku_rc"\n'
        'bash --rcfile "$resku_rc" -i\n'
    )
    return ['ssh', '-t', host, 'bash -c ' + shlex.quote(script)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='jecs@192.168.0.100')
    parser.add_argument('--setup', required=True)
    parser.add_argument('--command', required=True)
    args = parser.parse_args()
    argv = ssh_argv(args.host, args.setup, args.command)
    raise SystemExit(run_ssh(argv))


if __name__ == '__main__':
    main()
