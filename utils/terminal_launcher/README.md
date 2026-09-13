# Terminal command launcher

`commands.csv`의 명령을 tmux 분할 창에 입력만 해 둔 상태로 연다. 각 pane에서
명령을 확인한 뒤 Enter를 누르면 해당 명령만 실행된다.

## 실행

```bash
cd ~/dolbotZ
./utils/terminal_launcher/launch.py
```

기본값은 `utils/terminal_launcher/commands.csv`이며, 첫 번째 열은 pane 제목,
두 번째 열은 실행할 명령이다. 행 순서는 자유롭게 바꿔도 된다.

```csv
Drive Camera,"cd ~/dolbotZ && ros2 launch dolbotz drive_cam.launch.py"
Visualization,"cd ~/dolbotZ && ros2 launch dolbotz visualization.launch.py"
```

## 명령 선택 시 주의

- `Autonomous Full`은 `Reduced Odom Only`, `Nav2 MPPI Only`, `Path Control Only`를
  이미 포함한다. Full을 실행할 때 세 하위 명령은 따로 실행하지 않는다.
- `Summer Drive`는 여름 주행 하드웨어 체인이고, `Mission Summer Perception`은
  여름 인지 체인이다. 여름 미션에서는 두 명령을 함께 사용할 수 있다.
- 미션 launch는 공용 인지와 카메라 일부를 내부에서 포함한다. 동일 노드를
  개별 명령으로 다시 실행하지 않는다.
- `Manual Drive Joystick`과 `Manual Arm`은 기본적으로 각각 `joy_node`를 띄운다.
  동시에 쓸 때 한쪽 명령에 `launch_joy:=false`를 추가한다.
- `Only`가 붙은 행은 부분 시험과 디버깅용이다.

## 옵션

```bash
# 모든 명령을 한 tmux 창에 배치
./utils/terminal_launcher/launch.py --panes-per-window 0

# 창마다 최대 6개씩 배치
./utils/terminal_launcher/launch.py --panes-per-window 6

# 실제 창을 열지 않고 CSV 파싱과 터미널 탐지만 확인
./utils/terminal_launcher/launch.py --dry-run
```

다른 CSV를 쓰려면 첫 번째 인자로 전달한다. 터미널 자동 탐지가 맞지 않으면
`--terminal gnome-terminal`처럼 지정할 수 있다.

## 요구 사항

- Python 3.10 이상
- tmux
- GNOME Terminal, Tilix, Konsole, Kitty 등 지원되는 그래픽 터미널

tmux 창 이동은 `Ctrl+b` 후 `n`/`p`, detach는 `Ctrl+b` 후 `d`다.
