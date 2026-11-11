"""SpaceMouse가 인식되고 입력이 읽히는지 확인한다. 로봇 명령은 보내지 않는다."""

import argparse
import os
import sys
import time
from contextlib import ExitStack

import pyspacemouse

AXES = ("x", "y", "z", "roll", "pitch", "yaw")
ACTIVE_THRESHOLD = 0.08
PRINT_PERIOD_S = 0.1


def open_devices(stack, paths):
    devices = {}

    for path, name in paths.items():
        try:
            devices[path] = stack.enter_context(
                pyspacemouse.open_by_path(
                    path,
                    nonblocking=True,
                    axis_convention=pyspacemouse.AxisConvention.HID_Z_UP,
                )
            )
            print(f"열기 성공: {path} ({name})")
        except Exception as exc:
            print(f"열기 실패: {path} ({name}) — {exc}")

            if not os.access(path, os.R_OK | os.W_OK):
                print(
                    f"  → {path}에 읽기/쓰기 권한이 없습니다. "
                    "udev 규칙(hidraw 권한)을 확인하세요."
                )

    return devices


def read_latest(device):
    # 대기 중인 입력 report를 최대 64개 처리하고 마지막 상태를 쓴다.
    last_t = None

    for _ in range(64):
        state = device.read()

        if state.t == last_t:
            break

        last_t = state.t

    return state


def format_state(state):
    axes = " ".join(f"{axis}={getattr(state, axis):+.2f}" for axis in AXES)
    pressed = [i for i, value in enumerate(state.buttons) if value]
    active = max(abs(getattr(state, axis)) for axis in AXES) > ACTIVE_THRESHOLD

    return f"{axes} buttons={pressed} {'입력 중' if active else ''}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="실시간 값을 보여줄 시간 [s]. 0이면 Ctrl+C까지 계속",
    )
    args = parser.parse_args()

    paths = pyspacemouse.get_connected_devices_by_path()

    if not paths:
        print("연결된 SpaceMouse를 찾지 못했습니다. 수신기/케이블 연결을 확인하세요.")
        return 1

    print(f"감지된 SpaceMouse: {len(paths)}개")

    for path, name in paths.items():
        print(f"  {path}: {name}")

    with ExitStack() as stack:
        devices = open_devices(stack, paths)

        if not devices:
            print("열 수 있는 장치가 없습니다.")
            return 1

        print("\n마우스를 움직이거나 버튼을 눌러보세요. Ctrl+C로 종료합니다.\n")

        deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
        next_print = 0.0

        try:
            while deadline is None or time.monotonic() < deadline:
                states = {path: read_latest(device) for path, device in devices.items()}
                now = time.monotonic()

                if now >= next_print:
                    for path, state in states.items():
                        print(f"{path}: {format_state(state)}")
                    print(flush=True)
                    next_print = now + PRINT_PERIOD_S

                time.sleep(0.005)
        except KeyboardInterrupt:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
