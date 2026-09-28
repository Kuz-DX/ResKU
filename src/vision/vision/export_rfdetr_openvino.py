#!/usr/bin/env python3
"""One-time converter from an RF-DETR 1.9.3 checkpoint to OpenVINO IR."""

import argparse
import tempfile
from pathlib import Path


MODEL_TYPES = {
    'auto': None,
    'seg-small': 'RFDETRSegSmall',
    'seg-medium': 'RFDETRSegMedium',
    'small': 'RFDETRSmall',
    'medium': 'RFDETRMedium',
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('checkpoint', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--model-type', choices=MODEL_TYPES, default='auto')
    parser.add_argument('--output-name', default='mando-dummy-v1')
    parser.add_argument('--height', type=int, default=None)
    parser.add_argument('--width', type=int, default=None)
    parser.add_argument('--precision', choices=('float16', 'float32'), default='float16')
    return parser.parse_args()


def main():
    args = parse_args()
    checkpoint = args.checkpoint.expanduser().resolve()
    if not checkpoint.is_file() or checkpoint.stat().st_size == 0:
        raise FileNotFoundError(f'Checkpoint is missing or empty: {checkpoint}')
    if (args.height is None) != (args.width is None):
        raise ValueError('--height and --width must be specified together.')

    try:
        import openvino as ov
        import rfdetr
    except ImportError as exc:
        raise SystemExit(
            'Export dependency missing. Install requirements-export.txt first.'
        ) from exc

    if args.model_type == 'auto':
        model = rfdetr.RFDETR.from_checkpoint(checkpoint)
    else:
        model_class = getattr(rfdetr, MODEL_TYPES[args.model_type])
        model = model_class(pretrain_weights=str(checkpoint))

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f'{args.output_name}.xml'

    # RF-DETR 1.9.3 (the checkpoint's training version) has no native
    # OpenVINO exporter. Keep model loading/export on the matching release,
    # then let OpenVINO consume the temporary ONNX graph.
    with tempfile.TemporaryDirectory(prefix='rfdetr-onnx-', dir=output_dir) as temp_dir:
        export_kwargs = {
            'format': 'onnx',
            'output_dir': temp_dir,
            'verbose': False,
        }
        if args.height is not None:
            export_kwargs['shape'] = (args.height, args.width)
        onnx_path = Path(model.export(**export_kwargs))
        openvino_model = ov.convert_model(onnx_path)

        outputs_by_name = {}
        for output in openvino_model.outputs:
            for expected_name in ('dets', 'labels'):
                if any(expected_name in name for name in output.get_names()):
                    outputs_by_name[expected_name] = output
        missing_outputs = {'dets', 'labels'} - outputs_by_name.keys()
        if missing_outputs:
            raise RuntimeError(
                'Converted model is missing required outputs: '
                f'{sorted(missing_outputs)}'
            )
        # Segmentation checkpoints also export a large mask head. The ROS node
        # only consumes detection boxes/logits, so prune that branch before
        # serializing and compiling the deployment graph.
        openvino_model = ov.Model(
            [outputs_by_name['dets'], outputs_by_name['labels']],
            openvino_model.get_parameters(),
            'rfdetr_detection',
        )
        ov.save_model(
            openvino_model,
            output_path,
            compress_to_fp16=args.precision == 'float16',
        )

    weights_path = output_path.with_suffix('.bin')
    if not output_path.is_file() or not weights_path.is_file():
        raise RuntimeError(f'OpenVINO export did not create {output_path} and {weights_path}.')
    print(f'OpenVINO IR written to {output_path} and {weights_path}')


if __name__ == '__main__':
    main()

