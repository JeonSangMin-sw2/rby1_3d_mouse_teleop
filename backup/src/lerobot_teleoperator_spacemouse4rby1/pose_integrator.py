"""Hardware-independent dual-arm pose and gripper target integration."""

import time

import numpy as np
from scipy.spatial.transform import Rotation


# devices.py에서 사용하는 HID_Z_UP 기준:
# +X: 오른쪽, +Y: 앞쪽, +Z: 위쪽
#
# 기본 변환:
# 마우스 앞쪽 -> robot base +X
# 마우스 왼쪽 -> robot base +Y
# 마우스 위쪽 -> robot base +Z
DEFAULT_DEVICE_TO_BASE = np.array(
    [
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
)

SIDES = ("left", "right")


def _check_rotation(matrix):
    if (
        matrix.shape != (3, 3)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6)
    ):
        raise ValueError("Expected a proper 3x3 rotation matrix.")


class DualPoseIntegrator:
    def __init__(
        self,
        config,
        *,
        device_to_base=None,
        max_dt_s=0.05,
        workspace_bounds=None,
    ):
        self.config = config
        self.max_dt_s = float(max_dt_s)

        self.tcp_offset = np.asarray(
            config.tcp_offset_m,
            dtype=float,
        )

        if (
            self.tcp_offset.shape != (3,)
            or not np.isfinite(self.tcp_offset).all()
        ):
            raise ValueError("tcp_offset_m must contain 3 finite values.")

        if not np.isfinite(self.max_dt_s) or self.max_dt_s <= 0:
            raise ValueError("max_dt_s must be finite and positive.")

        self._maps = {}
        self._bounds = {}

        for side in SIDES:
            matrix = (
                DEFAULT_DEVICE_TO_BASE
                if device_to_base is None
                else device_to_base[side]
            )
            matrix = np.array(matrix, dtype=float, copy=True)
            _check_rotation(matrix)
            self._maps[side] = matrix

            if workspace_bounds is not None and side in workspace_bounds:
                bounds = np.array(
                    workspace_bounds[side],
                    dtype=float,
                    copy=True,
                )

                if (
                    bounds.shape != (2, 3)
                    or not np.isfinite(bounds).all()
                    or np.any(bounds[0] > bounds[1])
                ):
                    raise ValueError(f"Invalid {side} workspace bounds.")

                self._bounds[side] = bounds

        self._poses = {}
        self._grippers = {}

    def reset(self, poses, grippers=None):
        """현재 로봇 pose와 그리퍼 위치를 초기 목표로 설정한다."""
        targets = {}

        for side in SIDES:
            pose = np.array(poses[side], dtype=float, copy=True)

            if (
                pose.shape != (4, 4)
                or not np.isfinite(pose).all()
                or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-6)
            ):
                raise ValueError(f"Invalid {side} SE(3) pose.")

            _check_rotation(pose[:3, :3])

            if side in self._bounds:
                lower, upper = self._bounds[side]

                tcp_position = (
                    pose[:3, 3]
                    + pose[:3, :3] @ self.tcp_offset
                )

                if (
                    np.any(tcp_position < lower)
                    or np.any(tcp_position > upper)
                ):
                    raise ValueError(
                        f"Initial {side} TCP is outside workspace."
                    )

            targets[side] = pose

        grip_targets = {}

        if self.config.use_gripper:
            if grippers is None:
                raise ValueError(
                    "Actual initial gripper positions are required."
                )

            for side in SIDES:
                value = float(grippers[side])

                if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                    raise ValueError(f"Invalid {side} gripper position.")

                grip_targets[side] = value

        self._poses = targets
        self._grippers = grip_targets

    def _filter(self, values):
        values = np.asarray(values, dtype=float)

        if not np.isfinite(values).all():
            raise ValueError("SpaceMouse axes must be finite.")

        values = np.clip(values, -1.0, 1.0)
        deadzone = self.config.deadzone

        # deadzone 바깥 구간을 다시 0~1 범위로 조정한다.
        return (
            np.sign(values)
            * np.maximum(np.abs(values) - deadzone, 0.0)
            / (1.0 - deadzone)
        )

    def _read_input(self, raw, now):
        if raw is None or not raw.get("valid", True):
            return np.zeros(3), np.zeros(3), ()

        timestamp = float(raw["t"])

        if not np.isfinite(timestamp) or timestamp < 0:
            return np.zeros(3), np.zeros(3), ()

        buttons = tuple(raw["buttons"])
        age = now - timestamp

        if age < 0 or age > self.config.input_timeout_s:
            # 버튼은 누른 상태에서 새 report가 없을 수 있어 유지한다.
            # 오래된 축 입력만 0으로 만든다.
            return np.zeros(3), np.zeros(3), buttons

        translation = self._filter(
            [raw[k] for k in ("x", "y", "z")]
        )
        angular = self._filter(
            [raw[k] for k in ("roll", "pitch", "yaw")]
        )

        return translation, angular, buttons

    def update(self, inputs, dt, *, now=None):
        if not self._poses:
            raise RuntimeError(
                "Call reset() with actual poses before update()."
            )

        dt = float(dt)
        now = time.perf_counter() if now is None else float(now)

        if (
            not np.isfinite(dt)
            or dt < 0
            or not np.isfinite(now)
        ):
            raise ValueError(
                "dt must be non-negative; dt and now must be finite."
            )

        # 긴 일시 정지 뒤 누적 시간을 한 번에 적용하지 않는다.
        dt = min(dt, self.max_dt_s)

        targets, grippers = self.get_targets()

        for side in SIDES:
            translation, angular, buttons = self._read_input(
                inputs[side],
                now,
            )

            # 대각선 입력에서도 전체 속도가 설정값을 넘지 않게 한다.
            translation /= max(1.0, np.linalg.norm(translation))
            angular /= max(1.0, np.linalg.norm(angular))

            translation = self._maps[side] @ translation
            angular = self._maps[side] @ angular

            # 외부 목표는 arm_6 기준으로 유지하고,
            # 적분할 때만 TCP 자세로 변환한다.
            pose = targets[side]

            pose[:3, 3] += (
                pose[:3, :3] @ self.tcp_offset
            )

            pose[:3, 3] += (
                translation
                * self.config.translation_speed_m_s
                * dt
            )

            delta_rotation = Rotation.from_rotvec(
                angular * self.config.rotation_speed_rad_s * dt
            ).as_matrix()

            # base 기준 회전이므로 왼쪽에서 곱한다.
            pose[:3, :3] = delta_rotation @ pose[:3, :3]

            if side in self._bounds:
                lower, upper = self._bounds[side]
                pose[:3, 3] = np.clip(
                    pose[:3, 3],
                    lower,
                    upper,
                )

            # 적분된 TCP 목표를 다시 arm_6 목표로 변환한다.
            pose[:3, 3] -= (
                pose[:3, :3] @ self.tcp_offset
            )

            if self.config.use_gripper:
                open_index = self.config.gripper_open_button
                close_index = self.config.gripper_close_button

                opening = (
                    open_index < len(buttons)
                    and bool(buttons[open_index])
                )
                closing = (
                    close_index < len(buttons)
                    and bool(buttons[close_index])
                )

                # 두 버튼을 동시에 누르면 변화 없음.
                velocity = int(opening) - int(closing)

                grippers[side] = float(
                    np.clip(
                        grippers[side]
                        + velocity * self.config.gripper_speed * dt,
                        0.0,
                        1.0,
                    )
                )

        # 양팔 계산이 모두 성공한 뒤 내부 목표를 갱신한다.
        self._poses = targets
        self._grippers = grippers

        return self.get_targets()

    def get_targets(self):
        if not self._poses:
            raise RuntimeError("Call reset() before get_targets().")

        return (
            {
                side: pose.copy()
                for side, pose in self._poses.items()
            },
            self._grippers.copy(),
        )