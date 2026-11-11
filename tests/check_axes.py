"""SpaceMouse 각 축의 부호가 HID_Z_UP 정의와 맞는지 실제 동작으로 확인한다.

안내에 따라 마우스 하나를 정해진 방향으로 밀거나 기울이면, 그동안 들어온
6축 평균값과 기대한 축의 부호가 맞는지 출력한다. 로봇 명령은 보내지 않는다.
방향은 마우스 앞에 앉은 사용자 기준이다.
"""

import sys
import time
from contextlib import ExitStack

import numpy as np
import pyspacemouse

from lerobot_teleoperator_spacemouse4rby1.mouse import (
    AXES,
    open_spacemouse,
    read_latest,
)

ACTIVE_THRESHOLD = 0.2
HOLD_S = 0.5
RELEASE_THRESHOLD = 0.08

# (안내, 확인할 축, 기대 부호) — HID_Z_UP: +X 오른쪽, +Y 앞쪽, +Z 위쪽, 오른손 회전
STEPS = (
    ("캡을 오른쪽으로 밀기", "x", +1),
    ("캡을 앞쪽(내 반대쪽)으로 밀기", "y", +1),
    ("캡을 위로 들어올리기", "z", +1),
    ("캡 윗부분을 앞쪽(내 반대쪽)으로 기울이기", "roll", -1),
    ("캡 윗부분을 오른쪽으로 기울이기", "pitch", +1),
    ("위에서 봤을 때 캡을 반시계 방향으로 비틀기", "yaw", +1),
)


def wait_for_axis(devices, axis):
    """한 마우스의 axis 입력이 HOLD_S 동안 유지되면 그 경로와 6축 평균을 반환한다."""
    held_path = None
    held_since = None
    samples = []

    while True:
        now = time.monotonic()
        active = None

        for path, device in devices.items():
            state = read_latest(device)

            if abs(state[axis]) > ACTIVE_THRESHOLD:
                active = path, [state[name] for name in AXES]
                break

        if active is None:
            held_path = None
        else:
            path, values = active

            if path != held_path:
                held_path = path
                held_since = now
                samples = []

            samples.append(values)

            if now - held_since >= HOLD_S:
                return path, np.mean(samples, axis=0)

        time.sleep(0.01)


def wait_for_release(devices):
    while True:
        states = [read_latest(device) for device in devices.values()]

        if all(
            max(abs(state[axis]) for axis in AXES) < RELEASE_THRESHOLD
            for state in states
        ):
            return

        time.sleep(0.01)


def main():
    detected = pyspacemouse.get_connected_devices_by_path()

    if not detected:
        print("연결된 SpaceMouse를 찾지 못했습니다.")
        return 1

    results = []

    with ExitStack() as stack:
        devices = {path: open_spacemouse(stack, path) for path in sorted(detected)}

        # 두 마우스의 결과가 섞이지 않도록 처음 움직인 마우스 하나만 측정한다.
        print("\n확인할 마우스의 캡을 오른쪽으로 밀어 선택하세요.", flush=True)
        path, _ = wait_for_axis(devices, "x")
        devices = {path: devices[path]}
        print(f"  선택: {path}. 손을 떼주세요.", flush=True)
        wait_for_release(devices)

        for i, (prompt, axis, expected) in enumerate(STEPS, 1):
            print(f"\n[{i}/{len(STEPS)}] {prompt} (0.5초 이상 유지)", flush=True)
            path, mean = wait_for_axis(devices, axis)
            ok = np.sign(mean[AXES.index(axis)]) == expected
            results.append((prompt, axis, ok))

            print(f"  {path}: " + " ".join(f"{a}={v:+.2f}" for a, v in zip(AXES, mean)))
            print(f"  {axis} 부호 {'일치' if ok else '반대'} (기대 {expected:+d})")
            print("  손을 떼주세요.", flush=True)
            wait_for_release(devices)

    print("\n--- 결과 ---")

    for prompt, axis, ok in results:
        print(f"{'OK ' if ok else 'NG '} {axis:5s} {prompt}")

    return 0 if all(ok for _, _, ok in results) else 2


if __name__ == "__main__":
    sys.exit(main())
