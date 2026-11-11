"""Read two SpaceMouse devices; no robot commands."""

from contextlib import ExitStack
from pathlib import Path

import pyspacemouse

AXES = ("x", "y", "z", "roll", "pitch", "yaw")


class DualSpaceMouse:
    def __init__(self, left_path: str, right_path: str):
        self.paths = {
            "left": str(Path(left_path).resolve()),
            "right": str(Path(right_path).resolve()),
        }

        if self.paths["left"] == self.paths["right"]:
            raise ValueError("Left and right must use different devices.")

        self._stack = None
        self._devices = {}

    def connect(self):
        if self._stack is not None:
            raise RuntimeError("SpaceMouse devices are already connected.")

        stack = ExitStack()
        devices = {}

        try:
            for side, path in self.paths.items():
                devices[side] = stack.enter_context(
                    pyspacemouse.open_by_path(
                        path,
                        nonblocking=True,
                        axis_convention=pyspacemouse.AxisConvention.HID_Z_UP,
                    )
                )
        except BaseException:
            # 오른쪽 연결에 실패해도 먼저 열린 왼쪽 장치를 닫는다.
            stack.close()
            raise

        self._stack = stack
        self._devices = devices

    def read(self) -> dict[str, dict]:
        if self._stack is None:
            raise RuntimeError("Call connect() before read().")

        result = {}

        for side, device in self._devices.items():
            if not device.connected:
                raise ConnectionError(f"{side} SpaceMouse is closed.")

            # 대기 중인 입력 report를 최대 64개 처리한다.
            last_t = None

            for _ in range(64):
                state = device.read()

                if state.t == last_t:
                    break

                last_t = state.t

            # 드라이버 내부 state가 변경되어도 반환값은 유지되도록 복사한다.
            result[side] = {
                axis: float(getattr(state, axis))
                for axis in AXES
            }
            result[side]["buttons"] = tuple(state.buttons)
            result[side]["t"] = float(state.t)

        return result

    def disconnect(self):
        try:
            if self._stack is not None:
                self._stack.close()
        finally:
            self._stack = None
            self._devices = {}

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()


class SingleLeftSpaceMouse(DualSpaceMouse):
    """마우스 한 대를 왼팔에 연결하고 오른쪽 입력은 0으로 유지."""

    def __init__(self, left_path: str):
        self.paths = {
            "left": str(Path(left_path).resolve()),
        }

        self._stack = None
        self._devices = {}

    def read(self) -> dict[str, dict]:
        import time

        result = super().read()

        result["right"] = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "buttons": (),
            "t": time.perf_counter(),
        }

        return result


if __name__ == "__main__":
    import argparse
    import time

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--list",
        action="store_true",
        help="List device paths",
    )
    parser.add_argument("--left", help="Left SpaceMouse HID path")
    parser.add_argument("--right", help="Right SpaceMouse HID path")
    args = parser.parse_args()

    if args.list:
        devices = pyspacemouse.get_connected_devices_by_path()

        for path, name in devices.items():
            print(f"{path}: {name}")

    else:
        if not args.left or not args.right:
            parser.error(
                "Provide both --left and --right, or use --list."
            )

        try:
            with DualSpaceMouse(args.left, args.right) as mice:
                next_print = 0.0

                while True:
                    states = mice.read()
                    now = time.monotonic()

                    if now >= next_print:
                        print(states, flush=True)
                        next_print = now + 0.1

                    time.sleep(0.005)

        except KeyboardInterrupt:
            pass