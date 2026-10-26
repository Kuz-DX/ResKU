"""Synthetic CPU microbenchmark, NOT a hardware recognition/accuracy test."""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
from visual_landmark.localization import features, match
from visual_landmark.adaptive import candidate_window


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=10)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error('iterations must be positive')
    # Fixed thread count makes the experiment easier to repeat; no runtime defaults change.
    cv2.setNumThreads(1)
    image = np.random.default_rng(47).integers(0, 256, (480, 640), dtype=np.uint8)
    keypoints, descriptors = features(image)
    k = np.array([[500.,0,320.],[0,500.,240.],[0,0,1.]])
    depth = np.full(image.shape, 2., np.float32)
    reference = (keypoints, descriptors, k, np.zeros(5))
    arcs = {str(i):float(i) for i in range(100)}
    records = []
    for _ in range(args.iterations):
        wall, cpu = time.perf_counter(), time.process_time()
        current = features(image)
        sift = time.perf_counter()-wall
        attempts = []
        for lid, _arc in candidate_window(arcs, 50., limit=3):
            diag = {'id':lid}
            match(reference, current, depth, k, np.zeros(5), diagnostics=diag)
            attempts.append(diag)
        records.append(dict(total_s=time.perf_counter()-wall, cpu_s=time.process_time()-cpu,
                            current_sift_s=sift, candidates=len(attempts),
                            matching_and_depth_s=sum(a['matching_and_depth_s'] for a in attempts),
                            pnp_s=sum(a.get('pnp_s',0.) for a in attempts)))
    report = dict(kind='synthetic_single_frame_microbenchmark_not_full_two_frame_check',
                  opencv=cv2.__version__, threads=1, reference_count=100,
                  current_features=len(keypoints), iterations=args.iterations,
                  note='identical random-texture references, constant synthetic depth; not recognition validation',
                  mean={key:float(np.mean([r[key] for r in records])) for key in records[0]},
                  max={key:max(r[key] for r in records) for key in records[0]}, records=records)
    Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))


if __name__ == '__main__':
    main()
