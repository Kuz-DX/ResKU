#!/usr/bin/env python3
"""Plot and summarize detector CSV logs without ROS dependencies."""

from __future__ import annotations

import argparse
import csv
import glob
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, median

import matplotlib.pyplot as plt
import numpy as np


CAMERAS = ('drive', 'arm', 'left', 'right')
STAGES = (
    ('decode_avg_ms', 'decode'),
    ('pre_avg_ms', 'pre'),
    ('req_wait_avg_ms', 'req_wait'),
    ('infer_avg_ms', 'infer'),
    ('post_avg_ms', 'post'),
    ('draw_enc_avg_ms', 'draw+enc'),
    ('pub_avg_ms', 'pub'),
)


def _number(row: dict[str, str], column: str) -> float | None:
    value = row.get(column, '').strip()
    try:
        return float(value) if value else None
    except ValueError:
        return None


@dataclass
class Run:
    """One filtered CSV run and its display settings."""

    path: Path
    label: str
    rows: list[dict[str, str]]
    settings: dict[str, str]

    def values(self, camera: str, column: str) -> list[float]:
        values = [
            value
            for row in self.rows
            if row.get('camera') == camera
            for value in [_number(row, column)]
            if value is not None
        ]
        return values

    def camera_mean(self, camera: str, column: str) -> float:
        values = self.values(camera, column)
        return mean(values) if values else 0.0

    def camera_max(self, camera: str, column: str) -> float:
        values = self.values(camera, column)
        return max(values) if values else 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', help='CSV files or quoted glob patterns')
    parser.add_argument('-o', '--output', type=Path, default=Path('perf_comparison.png'))
    parser.add_argument('--title', default='Person detection performance')
    parser.add_argument(
        '--exclude-short',
        action=argparse.BooleanOptionalAction,
        default=True,
        help='exclude intervals shorter than 80%% of the median (default: on)',
    )
    parser.add_argument(
        '--skip-first',
        type=int,
        default=1,
        metavar='N',
        help='discard the first N reporting intervals from each run',
    )
    parser.add_argument(
        '--summary',
        action='store_true',
        help='print a run summary and write summary.csv beside the PNG',
    )
    return parser.parse_args()


def resolve_inputs(patterns: list[str]) -> list[Path]:
    paths: set[Path] = set()
    for pattern in patterns:
        matches = glob.glob(str(Path(pattern).expanduser()))
        if matches:
            paths.update(Path(item).resolve() for item in matches)
        else:
            candidate = Path(pattern).expanduser().resolve()
            if candidate.is_file():
                paths.add(candidate)
    if not paths:
        raise FileNotFoundError('No CSV inputs matched.')
    return sorted(paths)


def _filter_intervals(
    rows: list[dict[str, str]], skip_first: int, exclude_short: bool
) -> list[dict[str, str]]:
    keys: list[str] = []
    periods: dict[str, float] = {}
    for index, row in enumerate(rows):
        key = row.get('timestamp', '') or f'row-{index}'
        if key not in periods:
            keys.append(key)
            periods[key] = _number(row, 'period_s') or 0.0
    retained = keys[max(skip_first, 0):]
    if exclude_short and retained:
        typical = median([periods[key] for key in retained])
        retained = [key for key in retained if periods[key] >= typical * 0.8]
    keep = set(retained)
    return [
        row
        for index, row in enumerate(rows)
        if (row.get('timestamp', '') or f'row-{index}') in keep
    ]


def load_run(path: Path, skip_first: int, exclude_short: bool) -> Run:
    with path.open('r', encoding='utf-8', newline='') as stream:
        original = list(csv.DictReader(stream))
    if not original:
        raise ValueError(f'CSV contains no data rows: {path}')
    rows = _filter_intervals(original, skip_first, exclude_short)
    if not rows:
        raise ValueError(
            f'No intervals remain after filtering {path}; reduce --skip-first '
            'or use --no-exclude-short.'
        )
    first = rows[0]
    tag = first.get('run_tag', '').strip()
    fallback = (
        f'{first.get("backend", "?")}/{first.get("perf_hint", "?")}/'
        f'r{first.get("num_requests", "?")}/{first.get("scheduler", "?")}/'
        f'{"async" if first.get("async_infer", "").lower() == "true" else "sync"}'
    )
    return Run(path, tag or fallback, rows, first)


def fps_bars(axis: plt.Axes, runs: list[Run]) -> None:
    """Show actual camera FPS beside processed FPS for every run."""
    x = np.arange(len(CAMERAS), dtype=float)
    width = min(0.8 / max(len(runs) * 2, 1), 0.18)
    offset = (len(runs) * 2 - 1) * width / 2.0
    colors = plt.get_cmap('tab20').colors
    for index, run in enumerate(runs):
        color = colors[(index * 2) % len(colors)]
        cam = [run.camera_mean(camera, 'cam_fps') for camera in CAMERAS]
        proc = [run.camera_mean(camera, 'proc_fps') for camera in CAMERAS]
        base = x + index * 2 * width - offset
        axis.bar(
            base,
            cam,
            width,
            facecolor='none',
            edgecolor=color,
            hatch='//',
            label=f'{run.label} cam',
        )
        axis.bar(base + width, proc, width, color=color, label=f'{run.label} proc')
    axis.set_xticks(x, CAMERAS)
    axis.set_ylabel('frames/s')
    axis.set_title('Camera input FPS vs processed FPS')
    axis.grid(axis='y', alpha=0.25)


def grouped_bars(
    axis: plt.Axes,
    runs: list[Run],
    column: str,
    ylabel: str,
    title: str,
    aggregate: str = 'mean',
) -> None:
    x = np.arange(len(CAMERAS), dtype=float)
    width = min(0.8 / max(len(runs), 1), 0.24)
    offset = (len(runs) - 1) * width / 2.0
    for index, run in enumerate(runs):
        function = run.camera_max if aggregate == 'max' else run.camera_mean
        values = [function(camera, column) for camera in CAMERAS]
        axis.bar(x + index * width - offset, values, width, label=run.label)
    axis.set_xticks(x, CAMERAS)
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(axis='y', alpha=0.25)


def stage_bars(axis: plt.Axes, runs: list[Run]) -> None:
    labels: list[str] = []
    stage_values: defaultdict[str, list[float]] = defaultdict(list)
    for run in runs:
        for camera in CAMERAS:
            if not any(row.get('camera') == camera for row in run.rows):
                continue
            labels.append(f'{run.label}\n{camera}')
            for column, stage in STAGES:
                stage_values[stage].append(run.camera_mean(camera, column))
    x = np.arange(len(labels), dtype=float)
    bottom = np.zeros(len(labels), dtype=float)
    for _, stage in STAGES:
        values = np.asarray(stage_values[stage], dtype=float)
        axis.bar(x, values, bottom=bottom, label=stage)
        bottom += values
    axis.set_xticks(x, labels, rotation=30, ha='right', fontsize=7)
    axis.set_ylabel('average ms')
    axis.set_title('Stage time by run and camera')
    axis.grid(axis='y', alpha=0.25)
    axis.legend(ncol=4, fontsize=8)


def resource_lines(axis: plt.Axes, runs: list[Run]) -> None:
    rss_axis = axis.twinx()
    for run in runs:
        total_rows = [row for row in run.rows if row.get('camera') == 'total']
        elapsed = np.cumsum(
            [_number(row, 'period_s') or 0.0 for row in total_rows]
        ) / 60.0
        cpu = [_number(row, 'cpu_percent') or 0.0 for row in total_rows]
        rss = [_number(row, 'rss_mb') or 0.0 for row in total_rows]
        axis.plot(elapsed, cpu, label=f'{run.label} CPU')
        rss_axis.plot(elapsed, rss, '--', label=f'{run.label} RSS')
    axis.set_xlabel('elapsed minutes')
    axis.set_ylabel('CPU % (100% = one core)')
    rss_axis.set_ylabel('RSS MiB')
    axis.set_title('Process CPU and RSS over time')
    axis.grid(alpha=0.25)
    handles_a, labels_a = axis.get_legend_handles_labels()
    handles_b, labels_b = rss_axis.get_legend_handles_labels()
    axis.legend(handles_a + handles_b, labels_a + labels_b, fontsize=7, ncol=2)


def summary_rows(runs: list[Run]) -> tuple[list[str], list[dict[str, str | float]]]:
    fields = ['run_tag', 'separated', 'scheduler', 'async', 'requests', 'hint']
    for camera in CAMERAS:
        fields.extend(
            [
                f'{camera}_proc_fps',
                f'{camera}_cam_fps',
                f'{camera}_latency_p95_ms',
                f'{camera}_max_gap_ms',
                f'{camera}_infer_avg_ms',
            ]
        )
    fields.extend(('cpu_percent', 'rss_mb'))
    summaries: list[dict[str, str | float]] = []
    for run in runs:
        settings = run.settings
        row: dict[str, str | float] = {
            'run_tag': settings.get('run_tag', '').strip() or run.label,
            'separated': settings.get('separated', ''),
            'scheduler': settings.get('scheduler', ''),
            'async': settings.get('async_infer', ''),
            'requests': settings.get('num_requests', ''),
            'hint': settings.get('perf_hint', ''),
        }
        for camera in CAMERAS:
            row[f'{camera}_proc_fps'] = run.camera_mean(camera, 'proc_fps')
            row[f'{camera}_cam_fps'] = run.camera_mean(camera, 'cam_fps')
            row[f'{camera}_latency_p95_ms'] = run.camera_mean(
                camera, 'latency_p95_ms'
            )
            row[f'{camera}_max_gap_ms'] = run.camera_max(camera, 'max_gap_ms')
            row[f'{camera}_infer_avg_ms'] = run.camera_mean(
                camera, 'infer_avg_ms'
            )
        total_rows = [item for item in run.rows if item.get('camera') == 'total']
        cpu = [value for item in total_rows for value in [_number(item, 'cpu_percent')] if value is not None]
        rss = [value for item in total_rows for value in [_number(item, 'rss_mb')] if value is not None]
        row['cpu_percent'] = mean(cpu) if cpu else 0.0
        row['rss_mb'] = mean(rss) if rss else 0.0
        summaries.append(row)
    return fields, summaries


def write_summary(runs: list[Run], output: Path) -> None:
    fields, rows = summary_rows(runs)
    summary_path = output.parent / 'summary.csv'
    with summary_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print('\nSummary (averages after interval filtering):')
    print('\t'.join(fields))
    for row in rows:
        print(
            '\t'.join(
                f'{row[field]:.3f}' if isinstance(row[field], float) else str(row[field])
                for field in fields
            )
        )
    print(f'Wrote {summary_path}')


def main() -> int:
    args = parse_args()
    if args.skip_first < 0:
        raise ValueError('--skip-first must not be negative.')
    output = args.output.expanduser().resolve()
    runs = [
        load_run(path, args.skip_first, args.exclude_short)
        for path in resolve_inputs(args.csv)
    ]
    figure = plt.figure(figsize=(18, 15), constrained_layout=True)
    grid = figure.add_gridspec(3, 2)
    fps_axis = figure.add_subplot(grid[0, 0])
    latency_axis = figure.add_subplot(grid[0, 1])
    stage_axis = figure.add_subplot(grid[1, :])
    resource_axis = figure.add_subplot(grid[2, 0])
    gap_axis = figure.add_subplot(grid[2, 1])

    fps_bars(fps_axis, runs)
    grouped_bars(
        latency_axis,
        runs,
        'latency_p95_ms',
        'p95 ms',
        'End-to-end latency p95',
    )
    stage_bars(stage_axis, runs)
    resource_lines(resource_axis, runs)
    grouped_bars(
        gap_axis,
        runs,
        'max_gap_ms',
        'ms',
        'Maximum publish gap',
        aggregate='max',
    )
    fps_axis.legend(fontsize=7, ncol=2)
    latency_axis.legend(fontsize=7)
    gap_axis.legend(fontsize=7)
    figure.suptitle(args.title, fontsize=16)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    plt.close(figure)
    print(f'Wrote {output}')
    if args.summary:
        write_summary(runs, output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
