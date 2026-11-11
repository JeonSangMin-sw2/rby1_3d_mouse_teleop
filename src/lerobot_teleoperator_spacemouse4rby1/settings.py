"""텔레옵 설정. 값은 같은 폴더의 settings.yaml에서 읽어 대문자 전역 변수로 둔다."""

from pathlib import Path

import numpy as np
import yaml

SETTINGS_PATH = Path(__file__).with_name("settings.yaml")

# yaml에서 바꾸지 않는 고정값
CONTROL_PERIOD_S = 0.01  # 제어 주기 100 Hz
INPUT_TIMEOUT_S = 0.1  # 이보다 오래된 마우스 축 입력은 0으로 본다.
COMBO_WINDOW_S = 0.4  # 버튼 0, 1을 이 시간 안에 함께 누르면 그리퍼 토글
LOG_DIR = Path(__file__).resolve().parents[2] / "logs"  # 실행 기록 폴더 (프로젝트 폴더의 logs/)

KEYS = (
    "robot_address",
    "impedance_enabled",
    "unlimited_mode",
    "ready_pose_deg",
    "ready_minimum_time_s",
    "linear_velocity",
    "angular_velocity",
    "linear_acceleration",
    "angular_acceleration",
    "acceleration_limit_scaling",
    "align_velocity",
    "tcp_offset_m",
    "arm_stiffness",
    "arm_torque_limit",
    "arm_damping_ratio",
    "elbow_limit_rad",
    "max_target_lead_m",
    "max_target_lead_rad",
    "collision_distance_m",
    "collision_release_margin_m",
    "main_arm",
    "deadzone",
    "axis_filter_ratio",
    "translation_curve_temperature",
    "rotation_curve_temperature",
    "log_enabled",
    "use_gripper",
    "max_gripper_open",
)


def load(path=SETTINGS_PATH):
    with open(path, encoding="utf-8") as file:
        values = yaml.safe_load(file)

    missing = set(KEYS) - set(values)
    unknown = set(values) - set(KEYS)

    if missing or unknown:
        raise ValueError(
            f"{path} 설정 키가 맞지 않습니다. 빠진 키: {sorted(missing)}, 모르는 키: {sorted(unknown)}"
        )

    return {key.upper(): values[key] for key in KEYS}


globals().update(load())


def ready_pose_rad():
    return {name: np.deg2rad(values) for name, values in READY_POSE_DEG.items()}  # noqa: F821
