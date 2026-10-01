#!/usr/bin/env python3
"""Run a command over SSH, then keep its initialized interactive Bash alive."""

import argparse
import os
import shlex


def remote_rc(setup: str, command: str) -> str:
    return f'''# The bootstrap file is private and only needed while sourcing it.
command rm -f -- "$RESKU_REMOTE_RC"
unset RESKU_REMOTE_RC
[[ ! -r ~/.bashrc ]] || source ~/.bashrc
shopt -s histappend
set -o history
# tmux handles OSC 11 per pane, including output received through SSH.
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
    os.execvp(argv[0], argv)


if __name__ == '__main__':
    main()
