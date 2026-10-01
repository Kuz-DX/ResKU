# ResKU 터미널 런처

`Mando2026_ws/utils/terminal_launcher`를 ResKU용으로 옮긴 유틸리티다.
13개 명령을 tmux 분할 창에 미리 입력한다. **각 pane에서 Enter를 눌러야
해당 명령이 실행된다.** ROS 패키지 빌드는 필요 없다.

```bash
cd ~/ResKU
./util/terminal_launcher/launch.py
```

Python 3.10 이상, Bash, tmux, 그래픽 터미널이 필요하다.
GNOME Terminal, Tilix, Terminator, Konsole, Kitty 등을 자동 탐지한다.

```bash
# 창을 열거나 명령을 실행하지 않고 구성 확인
./util/terminal_launcher/launch.py --dry-run

# 한 창에 6개씩 배치 (기본값은 13개 모두 한 창)
./util/terminal_launcher/launch.py --panes-per-window 6

# 터미널 직접 지정
./util/terminal_launcher/launch.py --terminal tilix
```

## 명령 구성

| pane | 실행 위치 | 실행 내용 |
|---|---|---|
| Drive launch | 로봇 | Wi-Fi 연결 → CAN 설정 → manual_return_bringup |
| Drive_CAM | 로봇 | drive D435i |
| Arm_CAM | 로봇 | arm D435i |
| Side_CAM | 로봇 | C922 좌측 / C920 우측 |
| ROS_Bridge | 로봇 | rosbridge WebSocket |
| Person_Detect | 로봇 | 사람 감지, 사이드캠 추론 제외 |
| Supply_BOX | 로봇 | supply.launch.py |
| JOY_NODE | 로컬 | manual_control.launch.py |
| UI_Server | 로컬 | Docker Compose → HTTP 8080 |
| Drive_MTX | 로컬 | drive 영상 → RTSP main |
| Arm_MTX | 로컬 | arm 영상 → RTSP arm |
| Left_MTX | 로컬 | left 영상 → RTSP sub1 |
| Right_MTX | 로컬 | right 영상 → RTSP sub2 |

로봇 명령은 `ssh -t jecs@192.168.0.100`으로 접속한다. SSH/sudo 비밀번호는
해당 pane에서 입력한다. `remote_shell.py`가 원격 대화형 Bash를 열고 로봇의 `.bashrc`를 읽으므로
로컬과 로봇 양쪽에 기존 `sr`, `si` alias 또는 함수가 있어야 한다.
`sr`의 ROS_DOMAIN_ID, RMW, CycloneDDS 설정을 그대로 사용한다.
양쪽 `~/ResKU/install/setup.bash`는 기존 설치 결과가 있어야 한다.
원격 셸 초기화 코드는 SSH로 전달하므로 로봇에 `remote_shell.py`를 복사할 필요는 없다.

SSH pane에서 처음 Enter를 누르면 환경 준비 후 ROS 명령을 실행한다.
**Ctrl+C는 ROS 명령만 중단하며, SSH 연결과 `cd`·`sr`·`si`가 적용된 원격 셸은 유지된다.**
이어서 **↑ → Enter**를 누르면 마지막 ROS 명령만 다시 실행한다.
Wi-Fi/CAN 설정과 `sr`·`si`는 반복하지 않는다. 원격에서 직접 수정해 실행한 명령도
Bash 이력에 남으며, ↑는 가장 최근 입력부터 불러온다. SSH에서 나가려면 `exit` 또는 Ctrl+D를 누른다.

모든 ROS 명령은 `cd ~/ResKU && sr && si` 이후 실행한다.
UI도 ResKU에서 환경을 로드한 뒤 `~/DolbotZ-Center/deploy`로 이동한다.
사이드캠의 `readlink`와 `~` 확장은 원격 로봇에서 수행된다.
RTSP 브리지는 이 저장소의 `~/ResKU/util/ros_compressed_to_rtsp.py`를 사용하며,
카메라별 고유 노드 이름과 15 FPS / 1500 kbps / 키프레임 간격 15를 적용한다.

Drive launch의 Wi-Fi 연결이 바뀌면 SSH가 끊길 수 있다. 연결을 확인한 뒤
해당 명령을 다시 실행한다. Wi-Fi가 이미 연결되어 있거나 CAN 설정을 마친
상태에서는 pane의 입력을 편집해 필요한 부분만 실행할 수 있다.
UI_Server의 Compose 기동 후 MTX pane들을 실행하면 된다.

## 사용 및 수정

- 마우스로 pane 선택 → Enter 실행. Ctrl+C로 중단 후 ↑로 같은 명령 재입력.
- 빨간 배경은 셸 입력 대기, 녹색은 명령 실행 중이다. SSH에서도 Ctrl+C로 ROS 명령을 중단하면 빨간색으로 돌아오고, ↑ → Enter로 다시 실행하면 녹색으로 바뀐다. 노드의 정상 동작을 판정하는 표시가 아니다.
- `Ctrl+b` 다음 `z`: 선택 pane 확대/복원. `n`/`p`: 다음/이전 tmux 창.
- 로그 드래그 후 놓으면 복사된다. 시스템 클립보드는 `wl-copy`, `xclip`, `xsel` 또는 터미널의 OSC 52 지원을 사용한다.
- 마지막 터미널을 닫거나 detach하면 런처 세션이 종료된다. 재실행 시 남은 미접속 ResKU 런처 세션을 정리하며, 연결 중인 세션이 있으면 중복 실행을 거부한다.

Docker Compose는 `-d`로 실행하므로 터미널 종료 후에도 계속 실행된다.
종료하려면 별도로 `cd ~/DolbotZ-Center/deploy && sudo docker compose down`을 실행한다.
SSH 연결 종료만으로 원격 프로세스가 모두 정리됐다고 보장할 수는 없으므로,
재실행 시 장치 점유 문제가 있으면 기존 런북의 정리 절차를 참고한다.

`commands.csv`는 헤더 없이 `창 제목,명령` 두 열이다. IP, Wi-Fi 이름,
카메라 장치, 경로 등을 여기서 수정한다. 다른 CSV도 지정할 수 있다.

```bash
./util/terminal_launcher/launch.py ./my_commands.csv
```

세션/이력 이름은 `resku-commands-*`, `~/.local/state/resku-terminal-launcher`로
구분되어 Mando 런처 세션을 정리하지 않는다. 마우스 복사 키 설정은
참고 런처와 동일하게 현재 tmux 서버 전체에 적용된다.
