#!/usr/bin/env python3
"""Plot performance comparisons from one or more detector CSV logs."""

from __future__ import annotations

import argparse
import csv
import glob
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

import matplotlib.pyplot as plt
import numpy as np


CAMERAS = ('drive', 'arm', 'left', 'right')
STAGES = (
    ('decode_avg_ms', 'decode'),
    ('pre_avg_ms', 'pre'),
    ('infer_avg_ms', 'infer'),
    ('post_avg_ms', 'post'),
    ('draw_enc_avg_ms', 'draw+enc'),
    ('pub_avg_ms', 'pub'),
)


@dataclass
class Run:
    """One CSV log and the display label derived from its settings."""

    path: Path
    label: str
    rows: list[dict[str, str]]

    def camera_mean(self, camera: str, column: str) -> float:
        values = [
            float(row[column])
            for row in self.rows
            if row['camera'] == camera and row.get(column, '') != ''
        ]
        return mean(values) if values else 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', help='CSV files or quoted glob patterns')
    parser.add_argument('-o', '--output', type=Path, default=Path('perf_comparison.png'))
    parser.add_argument('--title', default='Person detection performance')
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


def load_run(path: Path) -> Run:
    with path.open('r', encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'CSV contains no data rows: {path}')
    first = rows[0]
    tag = first.get('run_tag', '').strip()
    label = (
        f'{first.get("backend", "?")}/{first.get("perf_hint", "?")}/'
        f'r{first.get("num_requests", "?")}/{first.get("scheduler", "?")}/'
        f'{"async" if first.get("async_infer", "").lower() == "true" else "sync"}'
        f'{"/" + tag if tag else ""}'
    )
    return Run(path, label, rows)


def grouped_bars(
    axis: plt.Axes,
    runs: list[Run],
    column: str,
    ylabel: str,
    title: str,
) -> None:
    x = np.arange(len(CAMERAS), dtype=float)
    width = min(0.8 / max(len(runs), 1), 0.24)
    offset = (len(runs) - 1) * width / 2.0
    for index, run in enumerate(runs):
        values = [run.camera_mean(camera, column) for camera in CAMERAS]
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
            if not any(row['camera'] == camera for row in run.rows):
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
    axis.legend(ncol=3, fontsize=8)


def resource_lines(axis: plt.Axes, runs: list[Run]) -> None:
    rss_axis = axis.twinx()
    for run in runs:
        total_rows = [row for row in run.rows if row['camera'] == 'total']
        elapsed = np.cumsum([float(row['period_s']) for row in total_rows]) / 60.0
        cpu = [float(row['cpu_percent']) for row in total_rows]
        rss = [float(row['rss_mb']) for row in total_rows]
        axis.plot(elapsed, cpu, label=f'{run.label} CPU')
        rss_axis.plot(elapsed, rss, '--', label=f'{run.label} RSS')
    axis.set_xlabel('elapsed minutes')
    axis.set_ylabel('CPU %')
    rss_axis.set_ylabel('RSS MiB')
    axis.set_title('CPU and RSS over time')
    axis.grid(alpha=0.25)
    handles_a, labels_a = axis.get_legend_handles_labels()
    handles_b, labels_b = rss_axis.get_legend_handles_labels()
    axis.legend(handles_a + handles_b, labels_a + labels_b, fontsize=7, ncol=2)


def main() -> int:
    args = parse_args()
    runs = [load_run(path) for path in resolve_inputs(args.csv)]
    figure = plt.figure(figsize=(18, 15), constrained_layout=True)
    grid = figure.add_gridspec(3, 2)
    fps_axis = figure.add_subplot(grid[0, 0])
    latency_axis = figure.add_subplot(grid[0, 1])
    stage_axis = figure.add_subplot(grid[1, :])
    resource_axis = figure.add_subplot(grid[2, 0])
    gap_axis = figure.add_subplot(grid[2, 1])

    grouped_bars(fps_axis, runs, 'proc_fps', 'frames/s', 'Processed FPS')
    grouped_bars(latency_axis, runs, 'latency_p95_ms', 'p95 ms', 'End-to-end latency p95')
    stage_bars(stage_axis, runs)
    resource_lines(resource_axis, runs)
    grouped_bars(gap_axis, runs, 'max_gap_ms', 'ms', 'Maximum publish gap')
    fps_axis.legend(fontsize=7)
    latency_axis.legend(fontsize=7)
    gap_axis.legend(fontsize=7)
    figure.suptitle(args.title, fontsize=16)
    args.output.expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output.expanduser().resolve(), dpi=160)
    plt.close(figure)
    print(f'Wrote {args.output.expanduser().resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
