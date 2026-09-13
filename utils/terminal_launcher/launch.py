#!/usr/bin/env python3
"""Open commands as prefilled input in portable tmux panes."""

from __future__ import annotations

import argparse
import csv
import hashlib
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Sequence


MODULE_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = MODULE_DIR.parent.parent
DEFAULT_COMMAND_FILE = MODULE_DIR / "commands.csv"
PANE_RC_FILE = MODULE_DIR / "pane_shell.bash"


class LauncherError(RuntimeError):
    """An error that can be presented directly to the user."""


@dataclass(frozen=True)
class Command:
    name: str
    source_row: int
    text: str


@dataclass(frozen=True)
class Terminal:
    name: str
    argv: tuple[str, ...]


TERMINAL_PROCESS_NAMES = {
    "alacritty": "alacritty",
    "foot": "foot",
    "gnome-terminal": "gnome-terminal",
    "gnome-terminal-server": "gnome-terminal",
    "kgx": "kgx",
    "kitty": "kitty",
    "konsole": "konsole",
    "terminator": "terminator",
    "tilix": "tilix",
    "wezterm": "wezterm",
    "wezterm-gui": "wezterm",
    "xfce4-terminal": "xfce4-terminal",
    "xterm": "xterm",
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "CSV 파일의 명령을 tmux 분할 창에 입력해 둔 상태로 터미널을 엽니다. "
            "각 창에서 Enter를 눌러 실행하세요."
        )
    )
    parser.add_argument(
        "commands",
        nargs="?",
        type=Path,
        default=DEFAULT_COMMAND_FILE,
        help=f"CSV 명령 파일 (기본값: {DEFAULT_COMMAND_FILE})",
    )
    parser.add_argument(
        "--panes-per-window",
        type=int,
        default=None,
        metavar="N",
        help=(
            "tmux 창 하나당 최대 분할 수 "
            "(기본값: CSV의 비어 있지 않은 행 수, 0도 모든 명령을 한 창에 배치)"
        ),
    )
    parser.add_argument(
        "--terminal",
        default="auto",
        metavar="PROGRAM",
        help="사용할 터미널 실행 파일. auto이면 현재/기본 터미널을 탐지 (기본값: auto)",
    )
    parser.add_argument(
        "--session",
        help="tmux 세션 이름 (생략 시 충돌하지 않는 이름을 자동 생성)",
    )
    parser.add_argument(
        "--working-directory",
        type=Path,
        default=WORKSPACE_DIR,
        help=f"각 셸의 시작 디렉터리 (기본값: {WORKSPACE_DIR})",
    )
    parser.add_argument(
        "--shell",
        type=Path,
        default=Path("/bin/bash"),
        help="분할 창에서 사용할 Bash 경로 (기본값: /bin/bash)",
    )
    parser.add_argument(
        "--state-dir",
        type=Path,
        default=Path.home() / ".local" / "state" / "dolbotz-terminal-launcher",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="세션을 만들지 않고 파싱/탐지 결과만 출력",
    )
    args = parser.parse_args(argv)
    if args.panes_per_window is not None and args.panes_per_window < 0:
        parser.error("--panes-per-window는 0 이상의 정수여야 합니다.")
    return args


def read_commands(path: Path) -> list[Command]:
    try:
        stream = path.expanduser().open(encoding="utf-8-sig", newline="")
    except OSError as exc:
        raise LauncherError(f"CSV 명령 파일을 읽을 수 없습니다: {path}: {exc}") from exc

    commands: list[Command] = []
    try:
        with stream:
            reader = csv.reader(stream)
            for row in reader:
                source_row = reader.line_num
                if not row or all(not value.strip() for value in row):
                    continue
                if len(row) != 2:
                    raise LauncherError(
                        f"CSV {source_row}행은 이름과 명령, 정확히 2열이어야 합니다. "
                        "쉼표가 포함된 값은 큰따옴표로 감싸세요."
                    )
                name = row[0].strip()
                command = row[1].strip()
                if not name:
                    raise LauncherError(f"CSV {source_row}행의 이름이 비어 있습니다.")
                if not command:
                    raise LauncherError(f"CSV {source_row}행의 명령이 비어 있습니다.")
                if "\n" in name or "\r" in name or "\n" in command or "\r" in command:
                    raise LauncherError(
                        f"CSV {source_row}행은 여러 줄 값을 포함할 수 없습니다."
                    )
                commands.append(Command(name=name, source_row=source_row, text=command))
    except (csv.Error, UnicodeDecodeError) as exc:
        raise LauncherError(f"CSV 형식이 올바르지 않습니다: {path}: {exc}") from exc

    if not commands:
        raise LauncherError(f"실행할 명령이 없습니다: {path}")
    return commands


def _process_ancestry() -> Iterable[str]:
    """Yield Linux process names from the parent towards the desktop process."""
    pid = os.getppid()
    visited: set[int] = set()
    while pid > 1 and pid not in visited:
        visited.add(pid)
        proc_dir = Path("/proc") / str(pid)
        try:
            name = (proc_dir / "comm").read_text(encoding="utf-8").strip().lower()
            stat = (proc_dir / "stat").read_text(encoding="utf-8")
            # The process name is parenthesized and can contain spaces.
            pid = int(stat.rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            return
        yield name


def _terminal_name(executable: str) -> str | None:
    base = Path(executable).name.lower()
    try:
        resolved = Path(executable).resolve().name.lower()
    except OSError:
        resolved = base
    for candidate in (base, resolved):
        for process_name, terminal_name in TERMINAL_PROCESS_NAMES.items():
            if candidate == process_name or candidate.startswith(process_name + "."):
                return terminal_name
    return None


def _available_terminal(program: str, extra_args: Sequence[str] = ()) -> Terminal | None:
    executable = shutil.which(program)
    if executable is None:
        return None
    name = _terminal_name(executable)
    if name is None:
        return None
    return Terminal(name=name, argv=(executable, *extra_args))


def detect_terminal(requested: str, env: dict[str, str] | None = None) -> Terminal:
    env = os.environ if env is None else env

    if requested != "auto":
        parts = shlex.split(requested)
        if not parts:
            raise LauncherError("--terminal 값이 비어 있습니다.")
        terminal = _available_terminal(parts[0], parts[1:])
        if terminal is None:
            raise LauncherError(f"지원하지 않거나 찾을 수 없는 터미널입니다: {parts[0]}")
        return terminal

    configured = env.get("TERMINAL", "").strip()
    if configured:
        parts = shlex.split(configured)
        if parts:
            terminal = _available_terminal(parts[0], parts[1:])
            if terminal is not None:
                return terminal

    term_program = env.get("TERM_PROGRAM", "").strip()
    if term_program:
        mapped = TERMINAL_PROCESS_NAMES.get(term_program.lower(), term_program)
        terminal = _available_terminal(mapped)
        if terminal is not None:
            return terminal

    for process_name in _process_ancestry():
        mapped = TERMINAL_PROCESS_NAMES.get(process_name)
        if mapped:
            terminal = _available_terminal(mapped)
            if terminal is not None:
                return terminal

    # Debian/Ubuntu's alternative tracks the user's preferred external terminal.
    terminal = _available_terminal("x-terminal-emulator")
    if terminal is not None:
        return terminal

    desktop = env.get("XDG_CURRENT_DESKTOP", "").lower()
    if "gnome" in desktop:
        preferred = ("kgx", "gnome-terminal", "tilix")
    elif "kde" in desktop:
        preferred = ("konsole",)
    elif "xfce" in desktop:
        preferred = ("xfce4-terminal",)
    else:
        preferred = ()

    candidates = (
        *preferred,
        "gnome-terminal",
        "konsole",
        "xfce4-terminal",
        "kitty",
        "wezterm",
        "alacritty",
        "foot",
        "tilix",
        "terminator",
        "xterm",
    )
    for program in dict.fromkeys(candidates):
        terminal = _available_terminal(program)
        if terminal is not None:
            return terminal

    raise LauncherError(
        "지원되는 터미널을 찾지 못했습니다. --terminal PROGRAM 또는 TERMINAL 환경 변수를 지정하세요."
    )


def terminal_argv(terminal: Terminal, command: Sequence[str]) -> list[str]:
    argv = list(terminal.argv)
    if terminal.name in {"gnome-terminal", "kgx"}:
        return [*argv, "--", *command]
    if terminal.name == "terminator":
        return [*argv, "-x", *command]
    if terminal.name == "tilix":
        # Tilix requires -e to be the last option; everything after it is the command.
        return [*argv, "-e", *command]
    if terminal.name == "xfce4-terminal":
        return [*argv, "--disable-server", "--execute", *command]
    if terminal.name == "wezterm":
        return [*argv, "start", "--", *command]
    if terminal.name in {"alacritty", "konsole", "xterm"}:
        return [*argv, "-e", *command]
    if terminal.name in {"foot", "kitty"}:
        return [*argv, *command]
    raise LauncherError(f"터미널 실행 방식을 알 수 없습니다: {terminal.name}")


def _run_tmux(*args: str, capture: bool = False) -> str:
    try:
        result = subprocess.run(
            ["tmux", *args],
            check=True,
            text=True,
            stdout=subprocess.PIPE if capture else subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError as exc:
        raise LauncherError("tmux가 필요합니다. 먼저 tmux를 설치해 주세요.") from exc
    except subprocess.CalledProcessError as exc:
        detail = exc.stderr.strip() if exc.stderr else str(exc)
        raise LauncherError(f"tmux 명령 실패: {detail}") from exc
    return result.stdout.strip() if capture and result.stdout else ""


def unique_session_name(requested: str | None) -> str:
    base = requested or f"dolbotz-commands-{time.strftime('%Y%m%d-%H%M%S')}"
    name = base
    suffix = 2
    while subprocess.run(
        ["tmux", "has-session", "-t", f"={name}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0:
        if requested:
            raise LauncherError(
                f"tmux 세션 '{requested}'이 이미 존재합니다. 다른 --session 이름을 사용하세요."
            )
        name = f"{base}-{suffix}"
        suffix += 1
    return name


def cleanup_stale_launcher_sessions() -> tuple[list[str], list[str]]:
    """Remove detached sessions created by this launcher and report active ones."""
    try:
        result = subprocess.run(
            [
                "tmux",
                "list-sessions",
                "-F",
                "#{session_name}\t#{session_attached}\t#{@dolbotz_terminal_launcher}",
            ],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError as exc:
        raise LauncherError("tmux가 필요합니다. 먼저 tmux를 설치해 주세요.") from exc

    # tmux returns 1 when there is no running server/session; that is normal.
    # Depending on the tmux version and socket state, a server that has never
    # been started is reported as an absent socket instead of "no server running".
    if result.returncode != 0:
        stderr = result.stderr.lower()
        no_sessions = (
            "no server running" in stderr
            or "no sessions" in stderr
            or (
                "error connecting to " in stderr
                and "no such file or directory" in stderr
            )
        )
        if result.returncode == 1 and no_sessions:
            return [], []
        raise LauncherError(f"tmux 세션 조회 실패: {result.stderr.strip()}")

    detached: list[str] = []
    active: list[str] = []
    for line in result.stdout.splitlines():
        fields = line.split("\t")
        if len(fields) != 3:
            continue
        name, attached_text, marker = fields
        is_launcher_session = marker == "1" or name.startswith("dolbotz-commands-")
        if not is_launcher_session:
            continue
        try:
            attached = int(attached_text)
        except ValueError:
            continue
        if attached > 0:
            active.append(name)
            continue
        _run_tmux("kill-session", "-t", f"={name}")
        detached.append(name)
    return detached, active


def _pane_command(shell: Path, history_file: Path, index: int) -> str:
    env_command = [
        "env",
        f"HISTFILE={history_file}",
        f"DOLBOTZ_HISTORY_FILE={history_file}",
        str(shell),
        "--rcfile",
        str(PANE_RC_FILE),
        "-i",
    ]
    return shlex.join(env_command)


def _chunks(items: Sequence[Command], size: int) -> Iterable[Sequence[Command]]:
    if size == 0:
        yield items
        return
    for start in range(0, len(items), size):
        yield items[start : start + size]


def _pane_ids_in_visual_order(target_window: str) -> list[str]:
    """Return pane IDs in row-major order: top-to-bottom, left-to-right."""
    output = _run_tmux(
        "list-panes",
        "-t",
        target_window,
        "-F",
        "#{pane_id}\t#{pane_top}\t#{pane_left}",
        capture=True,
    )
    panes: list[tuple[int, int, str]] = []
    for line in output.splitlines():
        try:
            pane_id, top, left = line.split("\t")
            panes.append((int(top), int(left), pane_id))
        except (ValueError, TypeError) as exc:
            raise LauncherError(f"tmux pane 좌표를 해석할 수 없습니다: {line!r}") from exc
    panes.sort(key=lambda pane: (pane[0], pane[1]))
    return [pane_id for _, _, pane_id in panes]


def create_tmux_session(
    session: str,
    commands: Sequence[Command],
    panes_per_window: int,
    working_directory: Path,
    shell: Path,
    state_dir: Path,
    command_file: Path,
) -> list[str]:
    if shutil.which("tmux") is None:
        raise LauncherError("tmux가 필요합니다. 먼저 tmux를 설치해 주세요.")
    if not shell.is_file():
        raise LauncherError(f"셸을 찾을 수 없습니다: {shell}")
    if not PANE_RC_FILE.is_file():
        raise LauncherError(f"분할 창 초기화 파일을 찾을 수 없습니다: {PANE_RC_FILE}")
    if not working_directory.is_dir():
        raise LauncherError(f"시작 디렉터리가 존재하지 않습니다: {working_directory}")

    digest = hashlib.sha256(str(command_file.resolve()).encode()).hexdigest()[:12]
    history_dir = state_dir.expanduser() / "history" / digest
    history_dir.mkdir(parents=True, exist_ok=True)

    pane_ids: list[str] = []
    command_index = 0
    try:
        for window_index, group in enumerate(_chunks(commands, panes_per_window), start=1):
            window_name = f"commands-{window_index:02d}"
            window_target = f"{session}:{window_name}"
            first_history = history_dir / f"pane-{command_index + 1:03d}.history"
            first_shell = _pane_command(shell, first_history, command_index + 1)
            if window_index == 1:
                pane_id = _run_tmux(
                    "new-session",
                    "-d",
                    "-P",
                    "-F",
                    "#{pane_id}",
                    "-s",
                    session,
                    "-n",
                    window_name,
                    "-c",
                    str(working_directory),
                    first_shell,
                    capture=True,
                )
            else:
                pane_id = _run_tmux(
                    "new-window",
                    "-d",
                    "-P",
                    "-F",
                    "#{pane_id}",
                    "-t",
                    session,
                    "-n",
                    window_name,
                    "-c",
                    str(working_directory),
                    first_shell,
                    capture=True,
                )
            command_index += 1

            for _ in group[1:]:
                history_file = history_dir / f"pane-{command_index + 1:03d}.history"
                pane_shell = _pane_command(shell, history_file, command_index + 1)
                pane_id = _run_tmux(
                    "split-window",
                    "-d",
                    "-P",
                    "-F",
                    "#{pane_id}",
                    "-t",
                    window_target,
                    "-c",
                    str(working_directory),
                    pane_shell,
                    capture=True,
                )
                command_index += 1

                # `split-window -d` keeps the original pane active. Without
                # redistributing here, tmux repeatedly halves that same pane
                # and eventually reports "no space for new pane" even though
                # the complete tiled layout would fit (for example, 4x4).
                _run_tmux("select-layout", "-t", window_target, "tiled")

            _run_tmux("select-layout", "-t", window_target, "tiled")
            pane_ids.extend(_pane_ids_in_visual_order(window_target))

        _run_tmux("set-option", "-t", session, "pane-border-status", "top")
        _run_tmux("set-option", "-t", session, "mouse", "on")
        _run_tmux("set-option", "-t", session, "@dolbotz_terminal_launcher", "1")
        _run_tmux(
            "set-option",
            "-t",
            session,
            "pane-border-format",
            " #[bold]#{pane_title}#[default] ",
        )

        # Input sent before Bash finishes its startup remains queued on the pane's pty.
        for index, (pane_id, command) in enumerate(zip(pane_ids, commands), start=1):
            _run_tmux(
                "select-pane",
                "-t",
                pane_id,
                "-T",
                f"{command.name} (row {command.source_row})",
            )
            _run_tmux("send-keys", "-t", pane_id, "-l", "--", command.text)

        # tmux's base-index is user-configurable, so select by our stable name.
        _run_tmux("select-window", "-t", f"{session}:commands-01")
        _run_tmux("select-pane", "-t", pane_ids[0])
        # A closed terminal normally only detaches its tmux client. Destroy the
        # session once the last client has detached so pane processes receive a
        # hangup and release serial devices. A detached session that has never
        # been attached remains available until the next-launch stale cleanup.
        kill_command = f"kill-session -t {shlex.quote('=' + session)}"
        hook_command = (
            "if-shell -F '#{==:#{session_attached},0}' "
            f"{shlex.quote(kill_command)} ''"
        )
        _run_tmux("set-hook", "-t", session, "client-detached", hook_command)
    except Exception:
        subprocess.run(
            ["tmux", "kill-session", "-t", f"={session}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        raise
    return pane_ids


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        command_file = args.commands.expanduser().resolve()
        working_directory = args.working_directory.expanduser().resolve()
        commands = read_commands(command_file)
        panes_per_window = args.panes_per_window or len(commands)
        terminal = detect_terminal(args.terminal)
        session = args.session or "(자동 생성)"

        if args.dry_run:
            windows = (len(commands) + panes_per_window - 1) // panes_per_window
            print(f"commands : {command_file}")
            print(f"count    : {len(commands)}")
            print(
                f"layout   : {windows} window(s), "
                f"up to {panes_per_window} pane(s) each"
            )
            print(f"terminal : {terminal.name} ({shlex.join(terminal.argv)})")
            print(f"session  : {session}")
            return 0

        if shutil.which("tmux") is None:
            raise LauncherError("tmux가 필요합니다. 먼저 tmux를 설치해 주세요.")
        removed_sessions, active_sessions = cleanup_stale_launcher_sessions()
        if removed_sessions:
            print("종료된 이전 tmux 세션: " + ", ".join(removed_sessions))
        if active_sessions:
            raise LauncherError(
                "이미 연결되어 실행 중인 런처 세션이 있습니다: "
                + ", ".join(active_sessions)
            )
        session = unique_session_name(args.session)
        create_tmux_session(
            session=session,
            commands=commands,
            panes_per_window=panes_per_window,
            working_directory=working_directory,
            shell=args.shell.expanduser().resolve(),
            state_dir=args.state_dir,
            command_file=command_file,
        )
        attach_command = ["tmux", "attach-session", "-t", f"={session}"]
        launch_command = terminal_argv(terminal, attach_command)
        try:
            subprocess.Popen(launch_command, start_new_session=True)
        except OSError as exc:
            subprocess.run(
                ["tmux", "kill-session", "-t", f"={session}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            raise LauncherError(f"터미널을 실행하지 못했습니다: {exc}") from exc

        print(f"{terminal.name}에서 tmux 세션 '{session}'을 열었습니다.")
        print("각 분할 창에서 Enter를 눌러 명령을 실행하세요.")
        print("tmux 창 이동: Ctrl+b 후 n(다음) / p(이전)")
        return 0
    except LauncherError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
