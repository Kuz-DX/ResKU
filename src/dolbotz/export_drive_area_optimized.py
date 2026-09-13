#!/usr/bin/env python3
"""
dolbotZ drivable-area segmentation — 재학습 없이 추론 속도 개선용 재export 스크립트.

train_drive_area.py는 매번 처음부터 재학습(150 epoch)까지 하는 무거운
스크립트라, "이미 학습된 best.pt를 다른 설정으로 다시 export만" 하고 싶을
때 쓰기엔 너무 느리다. 이 스크립트는 재학습 없이 config/models/dolbotz_seg_v1/
best.pt 하나로 두 가지 최적화를 export한다:

  1. imgsz 320: 실제 카메라 해상도(424x240)가 640보다 훨씬
     작은데도 모델이 정적 640x640으로 export돼 있어서 불필요하게 업스케일
     + 큰 연산을 하고 있었다. 320으로 낮추면 그 낭비가 줄어든다.
     이 export는 이미 실행되어
     config/models/dolbotz_seg_v1/best_320_openvino_model/ 에 결과물이
     들어가 있다(재실행해도 같은 결과, 멱등).
  2. INT8 양자화(OpenVINO NNCF, PTQ): OpenVINO가 CPU에서 INT8 연산을 FP32보다
     훨씬 빠르게 처리할 수 있어서(특히 AVX-VNNI 지원 CPU), 보통 몇 배 속도
     향상이 남. ultralytics의 export(quantize=8, data=...)가 내부적으로
     NNCF PTQ(재학습 없이 대표 이미지로 양자화 파라미터만 캘리브레이션)를
     돌려주고, 세그멘테이션 head의 민감한 연산(Add/Sub/Mul/Div, Sigmoid)은
     자동으로 양자화에서 제외한다(ultralytics 자체 구현, exporter.py 참고).

     [중요] 이 부분은 실행이 안 됐다 — 캘리브레이션에 실제 학습 데이터셋이
     필요한데, 그건 ROBOFLOW_API_KEY 환경변수가 있어야 다운로드된다(이
     sandbox엔 그 키가 없음). ROBOFLOW_API_KEY를 설정한 환경(원래
     train_drive_area.py를 돌렸던 GPU 환경 등)에서 이 스크립트를 실행하면
     INT8 export까지 마저 진행된다.

사용법:
  export ROBOFLOW_API_KEY=<Roboflow 대시보드에서 발급한 키>
  python export_drive_area_optimized.py

끝나면 생성/갱신되는 것 (config/models/dolbotz_seg_v1/ 아래):
  best_320_openvino_model/       <- imgsz=320, FP32 (이미 존재함)
  best_320_int8_openvino_model/  <- imgsz=320, INT8 (ROBOFLOW_API_KEY 있어야 생성됨)

기존 best_openvino_model/(원래 640 FP32)은 안 건드림 — 세 개를 나란히 두고
직접 A/B 비교(속도/정확도) 가능.
"""

import os
import shutil
import tempfile
from pathlib import Path

from ultralytics import YOLO

MODELS_DIR = Path(__file__).parent / 'config' / 'models' / 'dolbotz_seg_v1'
BEST_PT = MODELS_DIR / 'best.pt'
IMGSZ = 320


def _rename_ir_files(export_dir: Path, target_stem: str = 'best') -> None:
    """ultralytics는 export 폴더 안 .bin/.xml 파일명을 소스 .pt 파일명 그대로
    쓴다(예: best_320.pt -> best_320.bin/best_320.xml) — 기존
    best_openvino_model/의 best.bin/best.xml 네이밍 관례에 맞추기 위해
    'best.*'로 통일한다. OpenVINO IR은 .xml/.bin을 같은 basename으로 짝
    맞추는 구조라 이름을 맞춰 같이 바꾼다. 변경 후 재로드도 검증했다."""
    for f in export_dir.iterdir():
        if f.suffix in ('.bin', '.xml') and f.stem != target_stem:
            f.rename(export_dir / f'{target_stem}{f.suffix}')


def export_variant(suffix: str, quantize: int | None, data_yaml: str | None = None) -> Path:
    """best.pt를 MODELS_DIR 밖의 임시 디렉터리에 <suffix>.pt로 복사해서
    거기서 export한다 — ultralytics가 export 폴더를 소스 .pt와 같은
    디렉터리에 '<suffix>_openvino_model/'로 만드는데, 그게 최종 목적지
    (MODELS_DIR 아래 같은 이름)와 경로가 겹치면 옮기다가 자기 자신을
    지워버리는 충돌이 난다. 임시 디렉터리에서 작업한 뒤 결과만 최종 위치로
    옮기면 이 문제를 피할 수 있다."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp_pt = Path(tmp) / f'{suffix}.pt'
        shutil.copy(BEST_PT, tmp_pt)
        model = YOLO(str(tmp_pt), task='segment')
        kwargs = {'format': 'openvino', 'imgsz': IMGSZ, 'half': False}
        if quantize is not None:
            kwargs['quantize'] = quantize
            kwargs['data'] = data_yaml
        export_path = Path(model.export(**kwargs))

        _rename_ir_files(export_path)
        final_dir = MODELS_DIR / f'{suffix}_openvino_model'
        if final_dir.exists():
            shutil.rmtree(final_dir)
        shutil.move(str(export_path), str(final_dir))
    print(f'  -> {final_dir}')
    return final_dir


def main():
    if not BEST_PT.exists():
        raise SystemExit(f'{BEST_PT}가 없습니다 — 학습된 가중치부터 확보할 것.')

    print(f'[1/2] imgsz={IMGSZ} FP32 재export...')
    export_variant(f'best_{IMGSZ}', quantize=None)

    api_key = os.environ.get('ROBOFLOW_API_KEY')
    if not api_key:
        print(
            '\n[2/2] SKIP — INT8 export는 캘리브레이션용 데이터셋이 필요합니다.\n'
            '  ROBOFLOW_API_KEY 환경변수를 설정한 뒤 이 스크립트를 다시 실행하세요:\n'
            '    export ROBOFLOW_API_KEY=<Roboflow 대시보드에서 발급한 키>\n'
            '    python export_drive_area_optimized.py'
        )
        return

    print(f'[2/2] imgsz={IMGSZ} INT8 재export (캘리브레이션 데이터셋 다운로드 포함)...')
    from roboflow import Roboflow
    rf = Roboflow(api_key=api_key)
    project = rf.workspace('s-workspace-a8lvp').project('drive-area-slvqh-1gauq')
    version = project.version(1)
    dataset = version.download('yolo26')
    data_yaml = str(Path(dataset.location) / 'data.yaml')
    print(f'  calibration dataset: {data_yaml}')

    export_variant(f'best_{IMGSZ}_int8', quantize=8, data_yaml=data_yaml)

    print(
        '\n완료. config/models/dolbotz_seg_v1/ 아래 best_openvino_model(기존 640 FP32), '
        f'best_{IMGSZ}_openvino_model, best_{IMGSZ}_int8_openvino_model 세 개를 비교해보세요 — '
        '속도는 직접 추론 시간을 재보고, 정확도는 아래로 비교 가능합니다:\n'
        f'  yolo val task=segment model=config/models/dolbotz_seg_v1/best_{IMGSZ}_int8_openvino_model '
        f'imgsz={IMGSZ} data={data_yaml}'
    )


if __name__ == '__main__':
    main()
