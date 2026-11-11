"""Identify SpaceMouse HID paths by moving each mouse separately."""

import time
from contextlib import ExitStack

import pyspacemouse


AXES = ("x", "y", "z", "roll", "pitch", "yaw")
TEST_SECONDS = 10
THRESHOLD = 0.15


def find_candidates(label):
    input(
        f"\n{label} SpaceMouse를 준비하고 Enter를 누르세요. "
        "다른 마우스는 건드리지 마세요."
    )

    found = set()
    paths = pyspacemouse.get_connected_devices_by_path()

    if not paths:
        print("연결된 SpaceMouse 후보를 찾지 못했습니다.")
        return found

    with ExitStack() as stack:
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

        if not devices:
            print("열 수 있는 장치가 없습니다. HID 권한을 확인하세요.")
            return found

        print(
            f"\n{TEST_SECONDS}초 동안 {label} 마우스만 "
            "여러 방향으로 움직여주세요!"
        )

        deadline = time.monotonic() + TEST_SECONDS

        while time.monotonic() < deadline:
            for path, device in devices.items():
                try:
                    state = device.read()
                    values = {
                        axis: float(getattr(state, axis))
                        for axis in AXES
                    }

                    if (
                        path not in found
                        and max(abs(value) for value in values.values()) > THRESHOLD
                    ):
                        found.add(path)
                        print(
                            f"입력 감지: {path} "
                            f"{ {key: round(value, 2) for key, value in values.items()} }",
                            flush=True,
                        )
                except Exception as exc:
                    print(f"읽기 실패: {path} — {exc}", flush=True)

            time.sleep(0.005)

    return found


def main():
    print("SpaceMouse 경로 확인")
    print("왼쪽과 오른쪽 마우스를 차례대로 확인합니다.")

    try:
        left = find_candidates("왼쪽")
        right = find_candidates("오른쪽")
    except KeyboardInterrupt:
        print("\n확인을 중단했습니다.")
        return

    print("\n--- 결과 ---")
    print("왼쪽 후보:", ", ".join(sorted(left)) or "감지되지 않음")
    print("오른쪽 후보:", ", ".join(sorted(right)) or "감지되지 않음")

    if len(left) == 1 and len(right) == 1 and left != right:
        print("\n실행 옵션:")
        print(f"--teleop.left_device={next(iter(left))}")
        print(f"--teleop.right_device={next(iter(right))}")
    else:
        print(
            "\n각 마우스에서 서로 다른 경로가 하나씩 나올 때까지 "
            "다시 확인해주세요."
        )
        print(
            "확인 중에는 해당 마우스만 움직이고, "
            "다른 마우스는 손대지 마세요."
        )


if __name__ == "__main__":
    main()
