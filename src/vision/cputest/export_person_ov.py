#!/usr/bin/env python3
"""Export person.pt to a static end-to-end OpenVINO model on jecs."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from cputest.ov_person_detector import OpenVINOPersonDetector


def parse_imgsz(value: str) -> int | tuple[int, int]:
    """Parse either one square size or a height,width pair."""
    try:
        dimensions = tuple(int(item.strip()) for item in value.split(','))
    except ValueError as exc:
        raise argparse.ArgumentTypeError('--imgsz must contain integers.') from exc
    if len(dimensions) not in {1, 2} or any(item <= 0 for item in dimensions):
        raise argparse.ArgumentTypeError('--imgsz must be SIZE or HEIGHT,WIDTH.')
    return dimensions[0] if len(dimensions) == 1 else dimensions


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pt', type=Path, required=True, help='source person.pt')
    parser.add_argument(
        '--imgsz', type=parse_imgsz, default=parse_imgsz('480,640')
    )
    parser.add_argument('--half', action='store_true', help='export FP16 weights')
    parser.add_argument('--int8', action='store_true', help='export INT8 model')
    parser.add_argument('--data', type=Path, help='INT8 calibration dataset YAML')
    parser.add_argument('--out', type=Path, help='destination model directory')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.pt.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f'PT model not found: {source}')
    if args.half and args.int8:
        raise ValueError('--half and --int8 cannot be enabled together.')
    if args.int8 and args.data is None:
        raise ValueError('--data is required when --int8 is enabled.')
    data = None
    if args.data is not None:
        data_path = args.data.expanduser().resolve()
        if not data_path.is_file():
            raise FileNotFoundError(f'Calibration data YAML not found: {data_path}')
        data = str(data_path)

    from ultralytics import YOLO

    model = YOLO(str(source), task='detect')
    export_options = {
        'format': 'openvino',
        'imgsz': args.imgsz,
        'half': args.half,
        'int8': args.int8,
        'dynamic': False,
        'batch': 1,
        'nms': True,
        'device': 'cpu',
    }
    if data is not None:
        export_options['data'] = data
    exported_value = model.export(**export_options)
    exported = Path(str(exported_value)).expanduser().resolve()
    if not exported.exists():
        raise RuntimeError(f'Ultralytics returned a missing export path: {exported}')

    output = exported
    if args.out is not None:
        destination = args.out.expanduser().resolve()
        if destination != exported:
            if destination.exists():
                raise FileExistsError(
                    f'Output already exists; refusing to overwrite: {destination}'
                )
            if exported.is_dir():
                shutil.copytree(exported, destination)
            else:
                destination.mkdir(parents=True)
                shutil.copy2(exported, destination / exported.name)
                bin_path = exported.with_suffix('.bin')
                if bin_path.is_file():
                    shutil.copy2(bin_path, destination / bin_path.name)
            output = destination

    detector = OpenVINOPersonDetector(
        output,
        confidence_threshold=0.5,
        infer_size=(args.imgsz if isinstance(args.imgsz, int) else args.imgsz[0]),
        device='CPU',
        perf_hint='LATENCY',
        num_requests=1,
    )
    print(f'Exported model: {output}')
    print(f'Input shape: {detector.model_input_shape}')
    print(f'Output shape: {detector.output_shape}')
    print(f'Precision: {detector.precision}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
