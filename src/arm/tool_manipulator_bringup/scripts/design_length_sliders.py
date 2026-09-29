#!/usr/bin/env python3
"""Separate length sliders; Apply restarts only the mock launch owned by this UI."""
import os
import signal
import subprocess
import tkinter as tk


def main():
    root = tk.Tk()  # 길이 전용 창; 기존 관절각 UI와 독립적이다.
    root.title("Design link lengths (mock MoveIt)")
    values = {}  # 길이 슬라이더 값 [mm].
    process = None  # 이 창이 시작한 launch만 종료 대상으로 삼는다.
    closing = False  # 종료 요청 상태.
    restarting = False  # 중복 적용 방지.
    tk.Label(root, text="링크 길이와 베이스 전방 위치 (mm). 적용하면 mock MoveIt/RViz가 재시작됩니다.").pack()
    tk.Label(root, text="기존 mock/display launch는 먼저 종료하세요. 토크/과부하 실시간 표시는 미포함.").pack()
    for name, initial, lower, upper in (("L1", 180, 160, 200), ("L2", 220, 180, 260), ("L3", 150, 130, 170)):
        value = tk.IntVar(value=initial)  # 초기 길이 [mm].
        values[name] = value
        tk.Scale(root, label=name, from_=lower, to=upper, orient=tk.HORIZONTAL,
                 variable=value, length=420, resolution=1).pack()
    mount_x = tk.IntVar(value=278)
    tk.Scale(root, label="base x / chassis_mount_x", from_=220, to=390,
             orient=tk.HORIZONTAL, variable=mount_x, length=420, resolution=1).pack()
    # 0 mm는 미장착. 양수는 extra MX-106T에서 EE까지의 최소 110 mm 구간이다.
    extra = tk.IntVar(value=110)
    tk.Scale(root, label="extra MX-106T → EE (0 = 없음)", from_=0, to=200,
             orient=tk.HORIZONTAL, variable=extra, length=420, resolution=1).pack()
    angle_mode = tk.BooleanVar(value=False)  # 별도 각도 GUI 또는 mock trajectory 실행 선택.
    tk.Checkbutton(root, text="복제 설계 모델의 라디안 슬라이더 (계획만; 실행 불가)", variable=angle_mode).pack()
    status = tk.StringVar(value="길이 선택 후 적용")  # 실행 상태.
    tk.Label(root, textvariable=status).pack()

    def start():
        nonlocal process, restarting
        if closing:
            root.destroy()
            return
        command = ["ros2", "launch", "tool_manipulator_bringup", "design_lengths.launch.py"]
        command.extend([f"extra_pitch:={extra.get()/1000:.3f}", f"angle_sliders:={str(angle_mode.get()).lower()}"])
        command.extend(f"{name}:={value.get()/1000:.3f}" for name, value in values.items())
        command.append(f"chassis_mount_x:={mount_x.get()/1000:.3f}")
        try:
            process = subprocess.Popen(command, start_new_session=True)  # 소유 프로세스 그룹 분리.
            status.set("mock 실행 중; 실행 오류는 터미널 로그 확인")
        except OSError as exc:
            status.set(str(exc))
        restarting = False

    def wait_stop():
        if process is not None and process.poll() is None:
            root.after(150, wait_stop)  # GUI를 차단하지 않고 launch 종료를 기다린다.
        else:
            start()

    def apply():
        nonlocal restarting
        if restarting:
            return
        restarting = True
        if process is not None and process.poll() is None:
            status.set("기존 설계 모델 종료 대기 중")
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            wait_stop()
        else:
            start()

    def close():
        nonlocal closing
        closing = True
        if not restarting:
            apply()

    tk.Button(root, text="적용 / mock MoveIt 시작", command=apply).pack()
    root.protocol("WM_DELETE_WINDOW", close)
    root.mainloop()


if __name__ == "__main__":
    main()
