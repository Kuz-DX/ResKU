"""
리포/패키지 경로 해석 공용 유틸리티 — 실행 위치(cwd)나 사용자 홈 경로에
관계없이 config/와 config/models/ 디렉토리를 안정적으로
찾는다.

model_path를 절대경로로 하드코딩하면 사용자나 머신이 바뀔 때 깨지므로,
이 모듈에서 실행 환경에 맞는 경로를 해석한다.

두 가지 실행 환경을 모두 지원하는 하이브리드 방식을 쓴다:

  1. colcon install 환경 (심볼릭이든 일반 설치든): ament_index_python으로
     'dolbotz' 패키지의 share 디렉토리(예: install/dolbotz/share/dolbotz)를
     정확히 찾는다. config/는 setup.py의 data_files로 이 share 디렉토리
     아래 설치되므로, 실제 로봇 배포(정식 colcon install) 환경에서 항상
     안전하게 동작하는 유일한 방법이다 — 일반(비심볼릭) install에서는
     설치된 .py 파일이 site-packages 트리에, config/는 share 트리에 있어
     서로 부모-자식 관계가 아니므로, __file__ 기준 상위 탐색으로는 원리적으로
     찾을 수 없다.
  2. ROS 빌드 없이 소스에서 바로 실행하는 개발/테스트 환경(PYTHONPATH=src/dolbotz):
     ament_index가 'dolbotz'를 찾지 못하면(ImportError 또는
     PackageNotFoundError), 이 파일(__file__) 위치에서 위로 올라가며
     config/와 dolbotz/가 모두 있는 지점(=이 ROS2 패키지 폴더 자체)을
     리포 루트로 판정한다. (colcon --symlink-install 환경에서도 설치된
     .py가 소스로 심볼릭 연결되어 있으므로 이 폴백이 마침 같이 동작한다.)

두 방법이 모두 실패하면, 무엇을 시도했는지 전부 포함한 에러를 낸다.
"""

from __future__ import annotations

from pathlib import Path


def get_package_share_dir() -> Path:
    """dolbotz 패키지의 공유 데이터 루트. 모듈 docstring의 우선순위를 따른다."""
    tried: list[str] = []

    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError as exc:
        tried.append(f'ament_index_python import 실패: {exc}')
    else:
        try:
            return Path(get_package_share_directory('dolbotz'))
        except PackageNotFoundError as exc:
            tried.append(f"ament_index_python.get_package_share_directory('dolbotz') 실패: {exc}")

    try:
        return get_repo_root()
    except RuntimeError as exc:
        tried.append(str(exc))

    raise RuntimeError(
        'dolbotz 패키지의 공유 디렉토리를 찾지 못했습니다. 시도한 방법:\n  - '
        + '\n  - '.join(tried)
    )


def get_repo_root() -> Path:
    """소스 트리 기준 리포 루트를 __file__ 위치부터 상위로 올라가며 찾는다.

    config/와 dolbotz(패키지 코드 루트)가 모두 존재하는 첫 조상 디렉토리를
    리포 루트로 판정한다. 일반(비심볼릭) colcon install 환경에서는 config/가
    아예 다른 트리(share/)에 있으므로 이 함수만으로는 못 찾는다 — 그 경우엔
    get_package_share_dir()를 쓸 것 (내부적으로 이 함수를 폴백으로 사용함).
    """
    here = Path(__file__).resolve()
    for candidate in here.parents:
        if (candidate / 'config').is_dir() and (candidate / 'dolbotz').is_dir():
            return candidate
    raise RuntimeError(
        '__file__ 상위 탐색으로 리포 루트를 찾지 못했습니다 (config/, dolbotz/가 '
        f'모두 있는 상위 디렉토리 없음). 시도한 경로: {[str(p) for p in here.parents]}'
    )


def get_config_dir() -> Path:
    return get_package_share_dir() / 'config'


def get_models_dir() -> Path:
    return get_config_dir() / 'models'

