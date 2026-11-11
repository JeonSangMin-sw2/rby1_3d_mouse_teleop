"""두 SpaceMouse로 RB-Y1 양팔 입력을 만드는 LeRobot teleoperator. 로봇 명령은 없다.

get_action()은 팔마다 base 기준 twist(-1~1)와 그리퍼 닫힘(0/1)을 반환한다.
send_feedback()으로 받은 목표 그리퍼 자세는 그리퍼 좌표계 기준 이동에 쓴다.
버튼·키보드로 바꾸는 모드와 그리퍼 토글은 여기서 처리하고 events로 알린다.
"""

import time
from contextlib import ExitStack

import numpy as np
import pyspacemouse
from lerobot.teleoperators.teleoperator import Teleoperator
from scipy.spatial.transform import Rotation

from . import settings
from .config_spacemouse4rby1 import SpaceMouse4Rby1Config
from .events import EventBus, GripperChanged, ModeChanged
from .mouse import (
    ARM_MODES,
    AXES,
    BUTTON0,
    BUTTON1,
    COMBO,
    FRAMES,
    LOCK_MODES,
    SIDES,
    ButtonTracker,
    apply_arm_mode,
    apply_lock,
    initialize,
    next_mode,
    open_spacemouse,
    read_latest,
    to_base,
    tool_to_base,
)

TWIST_KEYS = ("x", "y", "z", "rx", "ry", "rz")
ROTVEC_KEYS = ("rx", "ry", "rz")
ARM_MODE_KEYS = {"m": "mirroring", "t": "tracking"}


class SpaceMouse4Rby1(Teleoperator):
    config_class = SpaceMouse4Rby1Config
    name = "spacemouse4rby1"

    def __init__(self, config, events=None):
        super().__init__(config)
        self.config = config
        self.events = events if events is not None else EventBus()

        if settings.MAIN_ARM not in SIDES:
            raise ValueError(f"main_arm은 right 또는 left여야 합니다: {settings.MAIN_ARM!r}")

        for key in ("translation_curve_temperature", "rotation_curve_temperature"):
            if not getattr(settings, key.upper()) >= 0.0:
                raise ValueError(f"{key}는 0 이상이어야 합니다: {getattr(settings, key.upper())!r}")

        self._stack = None
        self._devices = {}
        self._calibrations = None
        self._buttons = {side: ButtonTracker(settings.COMBO_WINDOW_S) for side in SIDES}
        self._tool_rotations = None

        self.locks = {side: LOCK_MODES[0] for side in SIDES}
        self.frames = {side: FRAMES[0] for side in SIDES}
        self.arm_mode = "normal"
        self.leader = settings.MAIN_ARM
        self.gripper_closed = {side: False for side in SIDES}
        self.wrist_only = {side: False for side in SIDES}
        # 실행 기록용: 마지막으로 읽은 마우스 6축 원래 값과 그 값이 오래되지 않았는지
        self.raw_axes = {side: np.zeros(6) for side in SIDES}
        self.raw_fresh = {side: False for side in SIDES}

    @property
    def action_features(self):
        features = {
            f"{side}_ee.{key}": float for side in SIDES for key in TWIST_KEYS
        }
        # 1이면 그리퍼 좌표계 z축 회전만 하는 중이다 (컨트롤러가 손목 위주로 돌린다).
        features.update({f"{side}_ee.wrist_only": float for side in SIDES})
        features.update({f"{side}_gripper.closed": float for side in SIDES})
        # ARM_MODES 순서 번호 (0 일반, 1 트래킹, 2 미러링). 따르는 팔은 main_arm이 아닌 팔이다.
        features["follow.mode"] = float
        return features

    @property
    def feedback_features(self):
        # 목표 그리퍼(ee) 자세의 base 기준 회전 벡터
        return {f"{side}_ee.{key}": float for side in SIDES for key in ROTVEC_KEYS}

    @property
    def is_connected(self):
        return self._stack is not None

    @property
    def is_calibrated(self):
        return self._calibrations is not None

    def connect(self, calibrate=True):
        if self.is_connected:
            raise RuntimeError("Teleoperator is already connected.")

        detected = pyspacemouse.get_connected_devices_by_path()

        if len(detected) < len(SIDES):
            raise ConnectionError(
                f"SpaceMouse가 {len(SIDES)}개 필요하지만 {len(detected)}개만 감지되었습니다."
            )

        stack = ExitStack()

        try:
            for path in sorted(detected):
                self._devices[path] = open_spacemouse(stack, path)
        except BaseException:
            stack.close()
            self._devices = {}
            raise

        self._stack = stack

        if calibrate:
            self.calibrate()

    def calibrate(self):
        """터미널 안내에 따라 마우스를 왼팔/오른팔에 매핑하고 축을 맞춘다."""
        self._calibrations = initialize(self._devices)

    def configure(self):
        pass

    def get_action(self):
        if not self.is_calibrated:
            raise RuntimeError("Call connect() and calibrate() first.")

        twists = {}

        for side in SIDES:
            state = read_latest(self._devices[self._calibrations[side].path])
            now = time.perf_counter()

            # report를 처리한 시각이 t이므로 읽은 뒤의 시각과 비교한다.
            fresh = now - state["t"] <= settings.INPUT_TIMEOUT_S
            self.raw_axes[side] = np.array([state[axis] for axis in AXES])
            self.raw_fresh[side] = fresh
            axes = self.raw_axes[side] if fresh else np.zeros(6)
            # 락이 걸린 축은 30% 필터 전에 지운다. 그러지 않으면 회전락 중에도 기울기가 가장 큰 축으로
            # 잡혀 작은 이동 입력을 걸러 낸 뒤, 락이 회전까지 지워 아무것도 안 남는다.
            axes = apply_lock(axes, self.locks[side])
            twists[side] = to_base(
                axes,
                self._calibrations[side].device_to_base,
                settings.DEADZONE,
                settings.AXIS_FILTER_RATIO,
                settings.TRANSLATION_CURVE_TEMPERATURE,
                settings.ROTATION_CURVE_TEMPERATURE,
            )

            for event in self._buttons[side].update(state["buttons"], now):
                self._handle_button(side, event)

        twists = self.apply_modes(twists)
        action = {}

        for side in SIDES:
            action.update({
                f"{side}_ee.{key}": float(value)
                for key, value in zip(TWIST_KEYS, twists[side])
            })
            action[f"{side}_ee.wrist_only"] = float(self.wrist_only[side])
            action[f"{side}_gripper.closed"] = float(self.gripper_closed[side])

        action["follow.mode"] = float(ARM_MODES.index(self.arm_mode))
        return action

    def apply_modes(self, twists):
        """팔마다 락과 기준 좌표계를 적용한 뒤, 협조 모드면 따르는 팔 입력을 주도 팔 입력으로 바꾼다.

        협조 모드에서 따르는 팔은 주도 팔의 락·좌표계가 적용된 base 기준 입력을 받는다.
        """
        result = {}

        for side in SIDES:
            twist = apply_lock(twists[side], self.locks[side])
            # 그리퍼 좌표계에서 z축 회전만 들어오면 컨트롤러가 손목(arm_6) 위주로 돌린다.
            self.wrist_only[side] = (
                self.frames[side] == "tool" and twist[5] != 0.0 and not np.any(twist[:5])
            )

            if self.frames[side] == "tool":
                if self._tool_rotations is None:
                    raise RuntimeError("그리퍼 좌표계를 쓰려면 send_feedback()으로 그리퍼 자세를 먼저 받아야 합니다.")

                twist = tool_to_base(twist, self._tool_rotations[side])

            result[side] = twist

        # 따르는 팔은 주도 팔의 base 기준 입력을 받으므로 자기 그리퍼 z축 회전이 아니다.
        if self.arm_mode != "normal":
            self.wrist_only[SIDES[1 - SIDES.index(self.leader)]] = False

        return apply_arm_mode(result, self.arm_mode, self.leader)

    def _handle_button(self, side, event):
        if event == COMBO:
            self.gripper_closed[side] = not self.gripper_closed[side]
            self.events.emit(GripperChanged(side, self.gripper_closed[side]))
        elif event == BUTTON0:
            self.locks[side] = next_mode(LOCK_MODES, self.locks[side])
            self.emit_modes()
        elif event == BUTTON1:
            self.frames[side] = next_mode(FRAMES, self.frames[side])
            self.emit_modes()

    def handle_key(self, key):
        """m: 미러링, t: 트래킹. 지금 모드와 같은 키를 다시 누르면 일반으로 돌아간다."""
        mode = ARM_MODE_KEYS.get(key.lower())

        if mode is None:
            return

        self.arm_mode = "normal" if self.arm_mode == mode else mode
        self.emit_modes()

    def emit_modes(self):
        self.events.emit(ModeChanged(dict(self.locks), dict(self.frames), self.arm_mode, self.leader))

    def emit_state(self):
        """현재 모드와 그리퍼 상태를 모두 알린다. (시작할 때 UI 초기화용)"""
        self.emit_modes()

        for side in SIDES:
            self.events.emit(GripperChanged(side, self.gripper_closed[side]))

    def send_feedback(self, feedback):
        self._tool_rotations = {
            side: Rotation.from_rotvec(
                [feedback[f"{side}_ee.{key}"] for key in ROTVEC_KEYS]
            ).as_matrix()
            for side in SIDES
        }

    def disconnect(self):
        try:
            if self._stack is not None:
                self._stack.close()
        finally:
            self._stack = None
            self._devices = {}
            self._calibrations = None
