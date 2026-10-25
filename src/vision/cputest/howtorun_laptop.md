# 노트북: 로그 가져오기와 분석

노트북에는 ROS, OpenVINO, Ultralytics가 필요 없다. jecs에서 생성한 CSV만 가져와
표준 Python, NumPy, Matplotlib로 분석한다. Windows, macOS, Linux에서 사용할 수
있다.

## 1. 필요한 프로그램

```bash
python3 -m pip install matplotlib numpy
```

Windows에서는 `python` 명령을 사용해도 된다. 로그 전송에는 SSH 접속이 가능한
`scp`가 필요하며, macOS/Linux에서는 `rsync`가 있으면 자동으로 우선 사용한다.

## 2. jecs 로그 가져오기

macOS/Linux/Git Bash:

```bash
chmod +x fetch_logs.sh
./fetch_logs.sh
```

원격 주소와 로컬 폴더를 바꾸려면:

```bash
./fetch_logs.sh user@robot-address ./my_logs
```

Windows PowerShell:

```powershell
.\fetch_logs.ps1
```

원격 주소와 로컬 폴더를 바꾸려면:

```powershell
.\fetch_logs.ps1 -RemoteHost user@robot-address -LocalDirectory .\my_logs
```

직접 가져오는 명령:

```bash
scp -r jecs@192.168.0.100:~/perf_logs ./perf_logs
```

스크립트는 기본적으로 `jecs@192.168.0.100:~/perf_logs`를 현재 폴더의
`./perf_logs`로 가져온 뒤 파일 수와 최신 파일명을 출력한다.

## 3. 그래프와 summary.csv 만들기

`plot_perf.py`와 가져온 `perf_logs`가 현재 폴더에 있다고 가정한다.

전체 비교:

```bash
python3 plot_perf.py "./perf_logs/*.csv" \
  -o ./perf_logs/compare.png --summary
```

기본값으로 각 실행의 첫 구간 하나를 제외하고, 나머지 구간 중앙값의 80%보다 짧은
시작/종료 구간도 제외한다. 옵션은 다음과 같다.

```text
--skip-first 1        첫 구간 하나 제외, 숫자 변경 가능
--exclude-short       짧은 구간 제외, 기본 켜짐
--no-exclude-short    짧은 구간도 포함
--summary             터미널 표와 summary.csv 생성
```

V0~V2 backend 비교만 그리기:

```bash
python3 plot_perf.py \
  "./perf_logs/*_V0.csv" "./perf_logs/*_V1.csv" "./perf_logs/*_V2.csv" \
  -o ./perf_logs/backend_compare.png --summary
```

S1~S6 구조 비교만 그리기:

```bash
python3 plot_perf.py \
  "./perf_logs/*_S1_sep.csv" "./perf_logs/*_S2_rr.csv" \
  "./perf_logs/*_S3_async.csv" "./perf_logs/*_S4_r2.csv" \
  "./perf_logs/*_S5_r4.csv" "./perf_logs/*_S6_prio.csv" \
  -o ./perf_logs/structure_compare.png --summary
```

범례는 `run_tag`를 우선 사용한다. FPS 그래프는 실제 `cam_fps`와 처리된
`proc_fps`를 나란히 보여준다. `summary.csv`에는 설정, 카메라별 FPS/latency/
max-gap/infer, 프로세스 CPU와 RSS 평균이 저장된다.

## 4. 결과 해석 체크리스트

1. `summary.json`에서 결과 일치가 PASS인지 확인한다. FAIL 설정은 제외한다.
2. 카메라별 `proc_fps / cam_fps` 비율이 높은 설정을 찾는다.
3. latency p95가 낮고 순간적으로 크게 튀지 않는지 확인한다.
4. 카메라별 max_gap 최대값이 작아 화면 갱신이 고른지 확인한다.
5. `lost`, `drop`, `stale_drop`, `err`가 낮은지 확인한다.
6. `req_wait`가 크면 동시 요청이 계속 포화된 상태인지 확인한다.
7. 같은 처리 성능이면 CPU%와 RSS가 낮은 쪽을 선택한다.
8. priority 설정에서는 drive 개선 때문에 arm/left/right가 굶지 않는지 확인한다.

최종 선택 우선순위는 정확도 PASS, `proc_fps` 추종률, latency p95, max-gap,
CPU/RSS 순서다.
