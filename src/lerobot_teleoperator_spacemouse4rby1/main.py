"""SpaceMouse 두 대로 RB-Y1 양팔을 텔레옵한다. 명령: rby1-spacemouse-teleop

순서: 마우스 초기화(터미널) → 로봇 연결·레디자세 → 양팔 Cartesian 스트림.
설정은 settings.yaml에서 바꾼다. 창을 닫거나 Ctrl+C로 끝낸다.
키보드 r: 레디자세로 돌아가기, i: 재초기화 (특이점 등으로 멈춘 뒤 fault 지우기 → unlimited → 레디자세),
m: 미러링, t: 트래킹, x·y·z: 따르는 팔의 그 축 위치를 메인 팔에 맞추기 (y는 좌우 대칭)
(모니터 창에서 바로 누르거나, 터미널에 입력 후 Enter).
log_enabled면 제어 주기마다 입력·목표·실제 자세·관절 각도를 프로젝트의 logs/에 CSV로 남긴다.
"""

import argparse
import csv
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from . import settings
from .config_spacemouse4rby1 import SpaceMouse4Rby1Config
from .events import EventBus, LogMessage, StageChanged, StatusUpdated, Stopped
from .robot import Rby1Controller, RobotStopped
from .spacemouse4rby1 import SpaceMouse4Rby1

SIDES = ("right", "left")
RAW_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")
TWIST_KEYS = ("x", "y", "z", "rx", "ry", "rz")
POSE_KEYS = ("x", "y", "z", "rx", "ry", "rz")
FLUSH_EVERY = 100
# 제어 루프가 처리하는 로봇 동작 키. r: 레디자세, x·y·z: 정렬. (i: 재초기화는 따로 처리)
ALIGN_KEYS = ("x", "y", "z")
MOTION_KEYS = ("r", *ALIGN_KEYS)


def _side_header(side):
    return (
        [f"{side}_raw_{key}" for key in RAW_KEYS]
        + [f"{side}_raw_fresh"]
        + [f"{side}_input_{key}" for key in TWIST_KEYS]
        + [f"{side}_target_{key}" for key in POSE_KEYS]
        + [f"{side}_actual_{key}" for key in POSE_KEYS]
        + [f"{side}_q{index}_deg" for index in range(7)]
        + [f"{side}_lock", f"{side}_frame", f"{side}_gripper_closed"]
    )


HEADER = ["time_s", "arm_mode", "blocked", "min_distance_m"] + _side_header("right") + _side_header("left") + ["events"]


def _pose_values(pose):
    return [*pose[:3, 3], *Rotation.from_matrix(pose[:3, :3]).as_rotvec()]


class RunLog:
    """실행 기록 CSV. 이 PC에만 저장한다. 위치는 base 기준 [m], 방향은 회전 벡터 [rad], 관절은 [deg]다."""

    def __init__(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.path = (directory / f"teleop_{time.strftime('%Y%m%d_%H%M%S')}.csv").resolve()
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADER)
        self._start = time.perf_counter()
        self._rows = 0
        # 이벤트는 키 입력처럼 다른 스레드에서도 오므로 잠금 아래 모았다가 다음 줄에 붙인다.
        self._lock = threading.Lock()
        self._events = []

    def on_event(self, event):
        if not isinstance(event, StatusUpdated):
            with self._lock:
                self._events.append(repr(event))

    def write(self, teleop, robot):
        cycle = robot.last_cycle

        with self._lock:
            events, self._events = self._events, []

        row = [
            f"{time.perf_counter() - self._start:.4f}",
            teleop.arm_mode,
            int(cycle["blocked"]),
            f"{cycle['min_distance']:.4f}",
        ]

        for side in SIDES:
            row += [f"{v:.4f}" for v in teleop.raw_axes[side]]
            row.append(int(teleop.raw_fresh[side]))
            row += [f"{v:.4f}" for v in cycle["inputs"][side]]
            row += [f"{v:.5f}" for v in _pose_values(cycle["targets"][side])]
            row += [f"{v:.5f}" for v in _pose_values(cycle["actuals"][side])]
            row += [f"{v:.2f}" for v in np.rad2deg(cycle["arm_q"][side])]
            row += [teleop.locks[side], teleop.frames[side], int(teleop.gripper_closed[side])]

        row.append(" | ".join(events))
        self._writer.writerow(row)
        self._rows += 1

        if self._rows % FLUSH_EVERY == 0:
            self._file.flush()

    def close(self):
        self._file.close()


def print_log_messages(event):
    """실행 로그(LogMessage)는 UI가 있어도 터미널에 같이 출력한다."""
    if isinstance(event, LogMessage):
        print(event.text, flush=True)


def print_events(event):
    """UI가 없을 때 나머지 상태 변화를 터미널에 출력한다."""
    if not isinstance(event, (StatusUpdated, LogMessage)):
        print(f"[event] {event}", flush=True)


def control_loop(teleop, events, stop, requests, run_log):
    robot = Rby1Controller(events)
    events.emit(StageChanged(f"로봇 연결, 레디자세 이동 중 ({settings.ROBOT_ADDRESS})"))

    try:
        robot.connect()
        teleop.send_feedback(robot.get_feedback())
        teleop.emit_state()
        events.emit(StageChanged("정상"))
        stopped = False  # 로봇이 스스로 제어를 끝내 재초기화(i)를 기다리는 중
        next_time = time.perf_counter()

        while not stop.is_set():
            if requests["i"].is_set() and not stopped:
                requests["i"].clear()
                events.emit(LogMessage("제어가 정상이라 재초기화할 필요가 없습니다. (i 키 무시)"))

            if requests["i"].is_set():
                # 멈춰 있던 동안 눌린 r, x, y, z는 버린다.
                for request in requests.values():
                    request.clear()

                events.emit(StageChanged("재초기화 중: fault 지우기 → unlimited → 레디자세 (마우스 입력 무시)"))

                try:
                    robot.reinitialize()
                except RuntimeError as exc:
                    stopped = True
                    events.emit(LogMessage(f"재초기화 실패: {exc}"))
                    events.emit(StageChanged("중단: 재초기화 실패 (i 키로 다시 시도)"))
                    continue

                stopped = False
                teleop.send_feedback(robot.get_feedback())
                events.emit(StageChanged("정상"))
                next_time = time.perf_counter()

            if stopped:
                if any(requests[key].is_set() for key in MOTION_KEYS):
                    for key in MOTION_KEYS:
                        requests[key].clear()

                    events.emit(LogMessage("제어가 멈춰 있어 실행하지 않습니다. i 키로 재초기화하세요."))

                time.sleep(settings.CONTROL_PERIOD_S)
                continue

            if requests["r"].is_set():
                requests["r"].clear()
                events.emit(StageChanged("레디자세로 이동 중 (마우스 입력 무시)"))
                robot.return_to_ready()
                teleop.send_feedback(robot.get_feedback())
                events.emit(StageChanged("정상"))
                next_time = time.perf_counter()

            for axis in ALIGN_KEYS:
                if requests[axis].is_set():
                    requests[axis].clear()
                    robot.align_axis(teleop.leader, axis)

            try:
                robot.send_action(teleop.get_action())
            except RobotStopped as exc:
                stopped = True
                events.emit(LogMessage(f"{exc} i 키로 재초기화할 수 있습니다."))
                events.emit(StageChanged(f"중단: {exc} (i 키로 재초기화)"))
                continue

            teleop.send_feedback(robot.get_feedback())

            if run_log is not None:
                run_log.write(teleop, robot)

            next_time += settings.CONTROL_PERIOD_S
            delay = next_time - time.perf_counter()

            if delay > 0:
                time.sleep(delay)
            else:
                next_time = time.perf_counter()
    finally:
        robot.disconnect()


def read_terminal_keys(on_key):
    """터미널에 입력한 줄(r, i, m, t, x, y, z)을 키 입력으로 넘긴다."""
    for line in sys.stdin:
        on_key(line.strip())


def run_control(teleop, events, stop, requests, run_log):
    try:
        control_loop(teleop, events, stop, requests, run_log)
    except Exception as exc:
        traceback.print_exc()
        events.emit(Stopped(f"오류: {exc}"))
    else:
        events.emit(Stopped("사용자 종료"))


def main():
    # 옵션은 없다(설정은 settings.yaml). --help로 설명만 볼 수 있게 남겨 둔다.
    argparse.ArgumentParser(description=__doc__).parse_args()

    monitor = None

    if settings.UI_ENABLED:
        try:
            from . import ui as monitor
        except ImportError as exc:
            print(f"경고: PySide6를 불러오지 못해 UI 없이 실행합니다 ({exc}).")

    events = EventBus()
    events.subscribe(print_log_messages)

    if monitor is None:
        events.subscribe(print_events)

    teleop = SpaceMouse4Rby1(SpaceMouse4Rby1Config(id="spacemouse4rby1"), events)

    try:
        teleop.connect()
    except KeyboardInterrupt:
        print("\n마우스 초기화를 중단했습니다.")
        teleop.disconnect()
        return 1

    stop = threading.Event()
    requests = {key: threading.Event() for key in ("i", *MOTION_KEYS)}
    run_log = RunLog(settings.LOG_DIR) if settings.LOG_ENABLED else None
    # 창이 뜨기 전에 나온 내용은 창이 열릴 때 실행 로그 첫 줄로 넣는다.
    startup_lines = ["마우스 초기화 완료"]

    if run_log is not None:
        events.subscribe(run_log.on_event)
        print(f"실행 기록: {run_log.path}", flush=True)
        startup_lines.append(f"실행 기록: {run_log.path}")

    def on_key(key):
        # r, i, x, y, z는 제어 루프가, 나머지(m, t)는 teleoperator가 처리한다.
        if key.lower() in requests:
            requests[key.lower()].set()
        else:
            teleop.handle_key(key)

    # 창이 있어도 터미널 입력(키 + Enter)을 받는다. 창에서는 Enter 없이 바로 받는다.
    threading.Thread(target=read_terminal_keys, args=(on_key,), daemon=True).start()

    try:
        if monitor is None:
            try:
                run_control(teleop, events, stop, requests, run_log)
            except KeyboardInterrupt:
                stop.set()
        else:
            worker = threading.Thread(target=run_control, args=(teleop, events, stop, requests, run_log))
            worker.start()
            monitor.run(events, on_close=stop.set, on_key=on_key, startup_lines=startup_lines)
            worker.join()
    finally:
        teleop.disconnect()
        if run_log is not None:
            run_log.close()
            print(f"실행 기록 저장: {run_log.path}", flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
