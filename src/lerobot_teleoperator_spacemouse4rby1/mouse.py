"""SpaceMouse 읽기, 왼팔/오른팔 초기화, 버튼 판정, 입력 처리(데드존·필터·감도·모드). 로봇 명령은 없다.

입력 twist는 base 기준 (x, y, z, rx, ry, rz)이고 각 값은 -1~1이다.
앞 3개는 이동 방향, 뒤 3개는 회전축 방향이다.
"""

import time
from dataclasses import dataclass

import numpy as np
import pyspacemouse

AXES = ("x", "y", "z", "roll", "pitch", "yaw")
SIDES = ("right", "left")
SIDE_NAMES = {"right": "오른팔", "left": "왼팔"}

# 팔마다 버튼 0으로 바꾸는 락
LOCK_MODES = ("normal", "position_lock", "rotation_lock")
# 팔마다 버튼 1로 바꾸는 이동 기준 좌표계. tool은 그리퍼(ee) 좌표계다.
FRAMES = ("base", "tool")
# 키보드 m, t로 바꾸는 양팔 협조 모드. tracking: 따르는 팔이 주도 팔 입력을 그대로,
# mirroring: 로봇 XZ 평면에 대칭으로 따른다.
ARM_MODES = ("normal", "tracking", "mirroring")


def open_spacemouse(stack, path):
    """HID_Z_UP(+X 오른쪽, +Y 앞쪽, +Z 위쪽) 기준으로 장치를 연다."""
    return stack.enter_context(
        pyspacemouse.open_by_path(
            path,
            nonblocking=True,
            axis_convention=pyspacemouse.AxisConvention.HID_Z_UP,
        )
    )


def read_latest(device):
    """대기 중인 입력 report를 모두 처리하고 마지막 상태를 복사해 반환한다."""
    last_t = None

    # 대기 중인 입력 report를 최대 64개 처리한다.
    for _ in range(64):
        state = device.read()

        if state.t == last_t:
            break

        last_t = state.t

    # 드라이버 내부 state가 변경되어도 반환값은 유지되도록 복사한다.
    result = {axis: float(getattr(state, axis)) for axis in AXES}
    result["buttons"] = tuple(state.buttons)
    result["t"] = float(state.t)

    return result


# 초기화: 각 팔마다 사용자가 마우스를 로봇 기준 +X 방향으로 밀면, 민 방향에 가장 가까운
# 마우스 축(앞·뒤·왼쪽·오른쪽)을 로봇 +X로 정한다. Z축은 항상 마우스를 들어올리는
# 방향(+)이므로 +Y는 오른손 좌표계로 따라 정해진다.

# 수평 이동 입력이 이 크기를 넘은 채로 HOLD_S 동안 유지되면 방향으로 인정한다.
PUSH_THRESHOLD = 0.3
HOLD_S = 0.5
RELEASE_THRESHOLD = 0.08

# 민 방향이 가장 가까운 마우스 축에서 이만큼 넘게 벗어나면(대각선에 가까우면) 어느 축인지 애매하므로 다시 측정한다.
MAX_AXIS_ERROR_DEG = 30.0
# 마우스 축 (HID_Z_UP: +X 오른쪽, +Y 앞쪽). 마우스 앞에 앉은 사람 기준 이름
DEVICE_AXES = (
    ((0.0, 1.0), "앞(+Y)"),
    ((0.0, -1.0), "뒤(-Y)"),
    ((1.0, 0.0), "오른쪽(+X)"),
    ((-1.0, 0.0), "왼쪽(-X)"),
)


@dataclass(frozen=True)
class MouseCalibration:
    path: str
    # base 벡터 = device_to_base @ 장치 벡터 (HID_Z_UP 기준)
    device_to_base: np.ndarray
    axis_error_deg: float


def x_axis_to_matrix(x_dir):
    """+X로 민 수평 방향을 가장 가까운 마우스 축에 맞춰 장치 -> base 회전 행렬을 만든다.

    (행렬, 고른 마우스 축 이름, 민 방향과 그 축 사이 각도[deg])를 반환한다.
    """
    x_dir = np.asarray(x_dir, dtype=float) / np.linalg.norm(x_dir)
    axis, name = max(DEVICE_AXES, key=lambda item: np.dot(item[0], x_dir))
    error_deg = float(np.degrees(np.arccos(np.clip(np.dot(axis, x_dir), -1.0, 1.0))))

    if error_deg > MAX_AXIS_ERROR_DEG:
        raise ValueError(
            f"민 방향이 마우스 {name} 축에서 {error_deg:.1f}도 벗어나 어느 축인지 애매합니다."
        )

    base_x = np.array([axis[0], axis[1], 0.0])
    base_z = np.array([0.0, 0.0, 1.0])
    base_y = np.cross(base_z, base_x)

    # 각 행은 장치 좌표로 표현한 base 축이다.
    return np.array([base_x, base_y, base_z]), name, error_deg


def initialize(devices, sides=SIDES):
    """devices(경로 -> 열린 장치)를 sides에 매핑하고 축을 맞춘다. 터미널 안내에 따라 진행한다.

    팔마다: 남은 마우스 중 하나를 +X로 민 채 HOLD_S 유지 → 그 마우스를 그 팔로 정하고
    민 방향으로 축 행렬을 만든다 → 손을 뗄 때까지 기다린다. 대각선에 가까우면 다시 잰다.
    """
    if len(devices) < len(sides):
        raise ValueError(
            f"마우스가 {len(sides)}개 필요하지만 {len(devices)}개만 열렸습니다."
        )

    result = {}

    for side in sides:
        name = SIDE_NAMES[side]
        assigned = {calibration.path for calibration in result.values()}
        remaining = [path for path in devices if path not in assigned]

        while True:
            print(f"\n[{name}] {name} 마우스를 로봇 기준 +X 방향(로봇 앞쪽)으로 밀고 유지하세요.", flush=True)
            held_path = None
            held_since = None
            samples = []
            warned = False

            # 한 마우스만 수평으로 PUSH_THRESHOLD 넘게, 위아래보다 크게 밀린 채 HOLD_S 유지될 때까지
            while True:
                pushes = {}

                for path in remaining:
                    state = read_latest(devices[path])
                    vector = np.array([state["x"], state["y"]])
                    magnitude = np.linalg.norm(vector)

                    if magnitude > PUSH_THRESHOLD and magnitude > abs(state["z"]):
                        pushes[path] = vector

                now = time.monotonic()

                if len(pushes) == 1:
                    path, vector = next(iter(pushes.items()))

                    if path != held_path:
                        held_path, held_since, samples = path, now, []

                    samples.append(vector)

                    if now - held_since >= HOLD_S:
                        break
                else:
                    if len(pushes) > 1 and not warned:
                        print("  두 마우스가 함께 움직였습니다. 한 마우스만 움직여주세요.")
                        warned = True

                    held_path = None

                time.sleep(0.01)

            print(f"  {name} 마우스: {held_path}")
            print("  확인했습니다. 마우스에서 손을 떼주세요.", flush=True)

            while not all(
                max(abs(state[axis]) for axis in ("x", "y", "z")) < RELEASE_THRESHOLD
                for state in (read_latest(devices[path]) for path in remaining)
            ):
                time.sleep(0.01)

            try:
                matrix, axis_name, error_deg = x_axis_to_matrix(np.mean(samples, axis=0))
            except ValueError as exc:
                print(f"  {exc} 마우스 축 방향으로 똑바로 밀어 다시 측정합니다.")
                continue

            print(
                f"  {name} 축 설정 완료: 마우스 {axis_name} = 로봇 앞(+X), "
                f"로봇 왼쪽(+Y)은 위에서 봤을 때 그 축에서 반시계 90도 (민 방향과 {error_deg:.1f}도 차이)"
            )
            result[side] = MouseCalibration(held_path, matrix, error_deg)
            break

    return result


# 버튼: 마우스 한 대의 버튼 0, 1 입력을 단독 누름과 동시 누름 이벤트로 바꾼다.

BUTTON0 = "button0"
BUTTON1 = "button1"
COMBO = "combo"


class ButtonTracker:
    """버튼 0, 1을 window_s 안에 함께 누르면 COMBO, 아니면 단독 누름으로 본다.

    단독 누름은 그 버튼을 뗄 때, 또는 누른 채 window_s가 지날 때 확정한다.
    그래서 단독 누름 이벤트는 최대 window_s만큼 늦게 나온다.
    이벤트를 낸 뒤 버튼을 누르고 있으면 모두 뗄 때까지 새 입력을 받지 않는다.
    """

    def __init__(self, window_s):
        self.window_s = window_s
        self._previous = (False, False)
        self._pending = None  # (먼저 누른 버튼 번호, 누른 시각)
        self._wait_release = False

    def update(self, buttons, now):
        pressed = (bool(buttons[0]), bool(buttons[1]))
        rising = [i for i in (0, 1) if pressed[i] and not self._previous[i]]
        self._previous = pressed
        events = []

        if self._wait_release:
            self._wait_release = any(pressed)
            return events

        if self._pending is None:
            if len(rising) == 2:
                events.append(COMBO)
                self._wait_release = True
            elif rising:
                self._pending = (rising[0], now)

            return events

        button, pressed_at = self._pending
        other = 1 - button

        if other in rising and now - pressed_at <= self.window_s:
            events.append(COMBO)
            self._pending = None
            self._wait_release = True
        elif not pressed[button] or now - pressed_at > self.window_s:
            events.append(BUTTON0 if button == 0 else BUTTON1)
            self._pending = None
            self._wait_release = any(pressed)

        return events


def next_mode(modes, current):
    return modes[(modes.index(current) + 1) % len(modes)]


def apply_deadzone(values, deadzone):
    values = np.clip(np.asarray(values, dtype=float), -1.0, 1.0)

    # deadzone 바깥 구간을 다시 0~1 범위로 조정한다.
    return (
        np.sign(values)
        * np.maximum(np.abs(values) - deadzone, 0.0)
        / (1.0 - deadzone)
    )


def keep_dominant_axes(values, ratio):
    """6축 중 가장 큰 입력의 ratio 이하인 축은 0으로 만든다. ratio가 0이면 그대로 둔다.

    캡을 밀 때 같이 들어오는 기울기처럼, 의도하지 않은 작은 축 입력을 없앤다.
    """
    values = np.array(values, dtype=float)
    largest = np.max(np.abs(values))

    if ratio > 0.0 and largest > 0.0:
        values[np.abs(values) <= ratio * largest] = 0.0

    return values


def apply_curve(values, temperature):
    """0~1 입력을 지수 곡선으로 바꾼다. 0과 ±1은 그대로라 최대 속도는 같고, 작게 밀수록 더 천천히 움직인다.

    temperature가 클수록 곡선이 강하다 (10이면 0.5 → 0.23). 0이면 그대로(직선)다.
    """
    values = np.asarray(values, dtype=float)

    if temperature <= 0.0:
        return values

    return np.sign(values) * np.expm1(np.log1p(temperature) * np.abs(values)) / temperature


def curve_magnitude(vector, temperature):
    """벡터 길이에만 곡선을 적용하고 방향은 그대로 둔다.

    축마다 곡선을 적용하면 작은 축이 더 많이 줄어 대각선 입력이 큰 축 쪽으로 휜다.
    """
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)

    if length == 0.0:
        return vector

    return vector * (apply_curve(length, temperature) / length)


def to_base(axes6, device_to_base, deadzone, axis_ratio=0.0, translation_curve=0.0, rotation_curve=0.0):
    """마우스 6축 입력을 초기화 결과로 보정해 base 기준 twist로 바꾼다.

    axis_ratio 필터는 곡선 전에 실제로 민 값으로 판단하고, 마우스 자체 축에 적용한다.
    base로 바꾼 뒤에 적용하면 마우스를 비스듬히 놓았을 때 대각선 이동이 잘린다.
    회전은 보통 이동보다 작게 밀리므로 곡선 세기를 따로 둔다.
    """
    filtered = keep_dominant_axes(apply_deadzone(axes6, deadzone), axis_ratio)
    translation = filtered[:3]
    angular = filtered[3:]

    # 대각선 입력에서도 크기가 1을 넘지 않게 한 뒤, 이동·회전 각각의 크기에 곡선을 적용한다.
    translation = curve_magnitude(translation / max(1.0, np.linalg.norm(translation)), translation_curve)
    angular = curve_magnitude(angular / max(1.0, np.linalg.norm(angular)), rotation_curve)

    return np.concatenate([device_to_base @ translation, device_to_base @ angular])


def mirror(twist):
    """로봇 XZ 평면 대칭. 이동은 y만, 회전축(축 벡터)은 rx, rz가 뒤집힌다."""
    return np.asarray(twist, dtype=float) * np.array([1, -1, 1, -1, 1, -1])


def apply_lock(twist, lock):
    twist = np.array(twist, dtype=float)

    if lock == "position_lock":
        twist[:3] = 0.0
    elif lock == "rotation_lock":
        twist[3:] = 0.0

    return twist


def tool_to_base(twist, rotation):
    """그리퍼 좌표계 기준 twist를 base 기준으로 바꾼다. rotation: base에서 본 그리퍼 자세."""
    twist = np.asarray(twist, dtype=float)
    return np.concatenate([rotation @ twist[:3], rotation @ twist[3:]])


def apply_arm_mode(twists, arm_mode, leader):
    """twists: side -> base 기준 twist. 따르는 팔의 입력을 주도 팔 입력으로 바꾼 새 dict를 반환한다."""
    result = {side: np.array(twist, dtype=float) for side, twist in twists.items()}
    follower = SIDES[1 - SIDES.index(leader)]

    if arm_mode == "tracking":
        result[follower] = result[leader].copy()
    elif arm_mode == "mirroring":
        result[follower] = mirror(result[leader])

    return result
