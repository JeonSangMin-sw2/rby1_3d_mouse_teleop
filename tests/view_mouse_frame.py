"""SpaceMouse 입력대로 3D 공간의 구가 움직이는지 눈으로 확인한다.

로봇 명령은 보내지 않는다. 시작할 때 마우스 초기화(왼팔/오른팔 매핑,
+X/+Y 축 설정)를 진행하고, 그 결과로 변환한 입력만큼 각 구를
로봇 base 기준(앞 +X, 왼쪽 +Y, 위 +Z)으로 상대 이동·회전시킨다.
창에서 r 키를 누르면 구를 시작 위치로 되돌린다.
"""

import argparse
import sys
import time
from contextlib import ExitStack

import matplotlib.pyplot as plt
import numpy as np
import pyspacemouse
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation

from lerobot_teleoperator_spacemouse4rby1.mouse import (
    AXES,
    SIDES,
    initialize,
    open_spacemouse,
    read_latest,
)

DEADZONE = 0.08
MAX_DT_S = 0.05
SPHERE_RADIUS_M = 0.05
FRAME_LENGTH_M = 0.12
VIEW_LIMIT_M = 0.5
START_POSITIONS = {
    "right": np.array([0.0, -0.25, 0.0]),
    "left": np.array([0.0, 0.25, 0.0]),
}
SIDE_COLORS = {"right": "tab:orange", "left": "tab:purple"}
AXIS_COLORS = ("tab:red", "tab:green", "tab:blue")
# 로봇 base 기준 이동(x, y, z)과 회전축(rx, ry, rz) 입력
BASE_AXES = ("x", "y", "z", "rx", "ry", "rz")


def apply_deadzone(values, deadzone=DEADZONE):
    values = np.clip(np.asarray(values, dtype=float), -1.0, 1.0)

    # deadzone 바깥 구간을 다시 0~1 범위로 조정한다.
    return (
        np.sign(values)
        * np.maximum(np.abs(values) - deadzone, 0.0)
        / (1.0 - deadzone)
    )


def to_base(axes6, device_to_base):
    """마우스 6축 입력을 초기화 결과로 보정해 로봇 base 기준 값으로 바꾼다."""
    filtered = apply_deadzone(axes6)
    translation = filtered[:3]
    angular = filtered[3:]

    # 대각선 입력에서도 전체 속도가 설정값을 넘지 않게 한다.
    translation = translation / max(1.0, np.linalg.norm(translation))
    angular = angular / max(1.0, np.linalg.norm(angular))

    return device_to_base @ translation, device_to_base @ angular


def step(
    position,
    rotation,
    translation,
    angular,
    dt,
    translation_speed,
    rotation_speed,
):
    """base 기준 입력으로 위치와 자세를 dt만큼 적분한다."""
    position = position + translation * translation_speed * dt

    # base 기준 회전이므로 왼쪽에서 곱한다.
    rotation = (
        Rotation.from_rotvec(angular * rotation_speed * dt).as_matrix()
        @ rotation
    )

    return position, rotation


class SphereBody:
    """좌표계가 달린 구. 구의 자세는 좌표계 방향으로 보인다."""

    def __init__(self, ax, label, color):
        self.ax = ax
        self.color = color

        u, v = np.meshgrid(
            np.linspace(0.0, 2.0 * np.pi, 16),
            np.linspace(0.0, np.pi, 9),
        )
        self.mesh = SPHERE_RADIUS_M * np.stack(
            [np.cos(u) * np.sin(v), np.sin(u) * np.sin(v), np.cos(v)]
        )
        self.surface = None
        self.lines = [
            ax.plot([], [], [], color=axis_color, linewidth=3)[0]
            for axis_color in AXIS_COLORS
        ]
        self.label = ax.text(0.0, 0.0, 0.0, label, color=color)

    def draw(self, position, rotation):
        if self.surface is not None:
            self.surface.remove()

        mesh = (
            np.einsum("ij,jkl->ikl", rotation, self.mesh)
            + position[:, None, None]
        )
        self.surface = self.ax.plot_surface(
            *mesh,
            color=self.color,
            alpha=0.35,
            linewidth=0,
        )

        for i, line in enumerate(self.lines):
            end = position + rotation[:, i] * FRAME_LENGTH_M
            line.set_data_3d(*zip(position, end))

        self.label.set_position_3d(
            position + np.array([0.0, 0.0, 2.0 * SPHERE_RADIUS_M])
        )


def setup_axes(ax):
    ax.set_xlim(-VIEW_LIMIT_M, VIEW_LIMIT_M)
    ax.set_ylim(-VIEW_LIMIT_M, VIEW_LIMIT_M)
    ax.set_zlim(-VIEW_LIMIT_M, VIEW_LIMIT_M)
    ax.set_box_aspect((1, 1, 1))

    # 로봇 오른쪽 뒤에서 앞을 보는 조작자 시점. 기본 시점은 로봇을 앞에서
    # 보므로 화면의 좌우와 회전 방향이 거울처럼 반대로 보인다.
    ax.view_init(elev=25, azim=-150)
    ax.set_xlabel("X forward [m]")
    ax.set_ylabel("Y left [m]")
    ax.set_zlabel("Z up [m]")

    # 고정된 base 좌표계(점선)
    for i, color in enumerate(AXIS_COLORS):
        end = np.zeros(3)
        end[i] = VIEW_LIMIT_M
        ax.plot(*zip(np.zeros(3), end), color=color, linestyle=":", alpha=0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--translation-speed", type=float, default=0.1, help="[m/s]")
    parser.add_argument("--rotation-speed", type=float, default=0.8, help="[rad/s]")
    parser.add_argument(
        "--input-timeout",
        type=float,
        default=0.08,
        help="이보다 오래된 축 입력은 0으로 본다 [s]. 0이면 검사하지 않는다",
    )
    args = parser.parse_args()

    detected = pyspacemouse.get_connected_devices_by_path()

    if not detected:
        print("연결된 SpaceMouse를 찾지 못했습니다.")
        return 1

    with ExitStack() as stack:
        devices = {}

        for path in sorted(detected):
            try:
                devices[path] = open_spacemouse(stack, path)
            except Exception as exc:
                print(f"열기 실패: {path} — {exc}")
                print("check_mouse.py로 장치와 권한을 먼저 확인하세요.")
                return 1

        try:
            calibrations = initialize(devices)
        except KeyboardInterrupt:
            print("\n초기화를 중단했습니다.")
            return 1

        print("\n초기화 결과:")

        for side in SIDES:
            calibration = calibrations[side]
            print(f"  {side}: {calibration.path}")
            print(np.round(calibration.device_to_base, 3))

        fig = plt.figure(figsize=(9, 8))
        ax = fig.add_subplot(projection="3d")
        setup_axes(ax)
        info = fig.text(0.01, 0.99, "", va="top", family="monospace", fontsize=8)

        bodies = {
            side: SphereBody(ax, side, SIDE_COLORS[side])
            for side in SIDES
        }
        poses = {}

        def reset():
            for side in SIDES:
                poses[side] = (START_POSITIONS[side].copy(), np.eye(3))

        def on_key(event):
            if event.key == "r":
                reset()

        reset()
        fig.canvas.mpl_connect("key_press_event", on_key)
        last_update = [time.perf_counter()]

        def update(_):
            frame_time = time.perf_counter()
            dt = min(frame_time - last_update[0], MAX_DT_S)
            last_update[0] = frame_time
            lines = []

            for side in SIDES:
                calibration = calibrations[side]
                state = read_latest(devices[calibration.path])

                # report를 처리한 시각이 t이므로 읽은 뒤의 시각과 비교한다.
                age = time.perf_counter() - state["t"]

                # 새 report가 끊긴 축 입력은 움직임에 쓰지 않는다.
                fresh = args.input_timeout <= 0 or age <= args.input_timeout
                translation, angular = to_base(
                    [state[axis] for axis in AXES] if fresh else np.zeros(6),
                    calibration.device_to_base,
                )
                position, rotation = step(
                    *poses[side],
                    translation,
                    angular,
                    dt,
                    args.translation_speed,
                    args.rotation_speed,
                )
                poses[side] = (position, rotation)
                bodies[side].draw(position, rotation)

                pressed = [i for i, value in enumerate(state["buttons"]) if value]
                lines.append(
                    f"{side:5s} {calibration.path}  base: "
                    + " ".join(
                        f"{a}={v:+.2f}"
                        for a, v in zip(BASE_AXES, (*translation, *angular))
                    )
                    + f"  buttons={pressed}  input_age={age:.3f}s"
                    + ("" if fresh else " (stale)")
                    + f"  pos={np.round(position, 3)}"
                )

            info.set_text("\n".join(lines))
            return []

        print("\n창에서 마우스를 움직여 구를 확인하세요. r: 시작 위치로, 창 닫기: 종료")
        _animation = FuncAnimation(fig, update, interval=33, cache_frame_data=False)
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
