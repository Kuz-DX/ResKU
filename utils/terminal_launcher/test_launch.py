#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path
import subprocess
from unittest import mock

try:
    from . import launch
except ImportError:  # Allow `python3 test_launch.py` from this directory.
    import launch


class ReadCommandsTest(unittest.TestCase):
    def test_reads_name_command_and_preserves_source_row(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "commands.csv"
            path.write_text(
                'First,"echo one, two"\nSecond,echo three\n',
                encoding="utf-8",
            )
            self.assertEqual(
                launch.read_commands(path),
                [
                    launch.Command("First", 1, "echo one, two"),
                    launch.Command("Second", 2, "echo three"),
                ],
            )

    def test_rejects_more_than_two_columns(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "commands.csv"
            path.write_text("First,echo one,unexpected\n", encoding="utf-8")
            with self.assertRaisesRegex(launch.LauncherError, "정확히 2열"):
                launch.read_commands(path)


class ArgumentTest(unittest.TestCase):
    def test_panes_per_window_defaults_to_auto(self) -> None:
        self.assertIsNone(launch.parse_args([]).panes_per_window)

    def test_explicit_panes_per_window_is_preserved(self) -> None:
        self.assertEqual(launch.parse_args(["--panes-per-window", "6"]).panes_per_window, 6)


class TerminalTest(unittest.TestCase):
    def test_explicit_terminal_has_priority(self) -> None:
        with mock.patch.object(launch.shutil, "which", return_value="/usr/bin/kitty"):
            terminal = launch.detect_terminal("kitty", {"TERMINAL": "xterm"})
        self.assertEqual(terminal.name, "kitty")

    def test_gnome_terminal_command_uses_separator(self) -> None:
        terminal = launch.Terminal("gnome-terminal", ("/usr/bin/gnome-terminal",))
        self.assertEqual(
            launch.terminal_argv(terminal, ["tmux", "attach", "-t", "demo"]),
            ["/usr/bin/gnome-terminal", "--", "tmux", "attach", "-t", "demo"],
        )

    def test_tilix_command_puts_execute_option_last(self) -> None:
        terminal = launch.Terminal("tilix", ("/usr/bin/tilix", "--maximize"))
        self.assertEqual(
            launch.terminal_argv(terminal, ["tmux", "attach", "-t", "demo"]),
            ["/usr/bin/tilix", "--maximize", "-e", "tmux", "attach", "-t", "demo"],
        )


class PaneOrderingTest(unittest.TestCase):
    def test_visual_order_is_top_to_bottom_and_left_to_right(self) -> None:
        geometry = "\n".join(
            (
                "%1\t0\t0",
                "%4\t20\t40",
                "%3\t20\t0",
                "%2\t0\t40",
            )
        )
        with mock.patch.object(launch, "_run_tmux", return_value=geometry):
            self.assertEqual(
                launch._pane_ids_in_visual_order("demo:commands-01"),
                ["%1", "%2", "%3", "%4"],
            )


class SessionCleanupTest(unittest.TestCase):
    def test_missing_tmux_socket_means_there_are_no_sessions(self) -> None:
        listed = subprocess.CompletedProcess(
            args=["tmux"],
            returncode=1,
            stdout="",
            stderr=(
                "error connecting to /tmp/tmux-1000/default "
                "(No such file or directory)\n"
            ),
        )
        with mock.patch.object(launch.subprocess, "run", return_value=listed):
            self.assertEqual(launch.cleanup_stale_launcher_sessions(), ([], []))

    def test_connection_permission_error_is_not_hidden(self) -> None:
        listed = subprocess.CompletedProcess(
            args=["tmux"],
            returncode=1,
            stdout="",
            stderr="error connecting to /tmp/tmux-1000/default (Permission denied)\n",
        )
        with (
            mock.patch.object(launch.subprocess, "run", return_value=listed),
            self.assertRaisesRegex(launch.LauncherError, "Permission denied"),
        ):
            launch.cleanup_stale_launcher_sessions()

    def test_removes_only_detached_launcher_sessions(self) -> None:
        listed = subprocess.CompletedProcess(
            args=["tmux"],
            returncode=0,
            stdout=(
                "unrelated\t0\t\n"
                "marked-launcher\t0\t1\n"
                "dolbotz-commands-old\t0\t\n"
                "dolbotz-commands-active\t1\t1\n"
            ),
            stderr="",
        )
        with (
            mock.patch.object(launch.subprocess, "run", return_value=listed),
            mock.patch.object(launch, "_run_tmux") as run_tmux,
        ):
            removed, active = launch.cleanup_stale_launcher_sessions()

        self.assertEqual(removed, ["marked-launcher", "dolbotz-commands-old"])
        self.assertEqual(active, ["dolbotz-commands-active"])
        self.assertEqual(
            run_tmux.call_args_list,
            [
                mock.call("kill-session", "-t", "=marked-launcher"),
                mock.call("kill-session", "-t", "=dolbotz-commands-old"),
            ],
        )


if __name__ == "__main__":
    unittest.main()
