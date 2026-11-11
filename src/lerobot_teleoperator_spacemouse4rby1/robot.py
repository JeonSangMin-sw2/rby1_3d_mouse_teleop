"""RB-Y1 제어: 레디자세 이동 후 양팔 Cartesian 명령을 스트림으로 보낸다.

send_action()은 SpaceMouse4Rby1.get_action()의 twist를 적분해 팔마다 EE 목표를 만든다.
트래킹·미러링에서 따르는 팔은 입력 대신 메인 팔이 실제로 움직인 만큼만 움직이고,
메인 팔은 따르는 팔이 따라온 만큼보다 선행 한도 넘게 앞서지 못한다.
링크 사이 거리가 COLLISION_DISTANCE_M보다 가까우면 경고하고, 그 링크에 걸린 팔은
상대 링크에서 멀어지는 이동만 허용한다(회전은 막는다).
"""

import itertools
import re
import time
from contextlib import suppress

import numpy as np
import rby1_sdk as rby
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation

from . import settings
from .events import CollisionChanged, LogMessage, SingularityChanged, StatusUpdated
from .mouse import ARM_MODES, mirror

SIDES = ("right", "left")
SIDE_NAMES = {"right": "오른팔", "left": "왼팔"}
EE_LINKS = {"right": "ee_right", "left": "ee_left"}
TORSO_LINK = "link_torso_5"  # 모니터 그림의 기준 좌표계
# 그리퍼 좌표계에서 z축 회전만 할 때(행동의 *_ee.wrist_only)는 base 기준 목표만 주면 7관절이 회전을
# 나눠 맡아 어깨가 같이 돈다. arm_6를 돌리면 그리퍼가 자기 z축으로 같은 각도만큼 돌므로 arm_6에 보조 관절 목표를 함께 준다.
# 손목만 돌릴 때 arm_6가 한도에서 이만큼 안쪽에서 멈춘다. 한도에 닿은 뒤에도 회전 목표가 들어가면
# 다른 관절로 회전을 이어 가 팔 전체가 출렁였다 (시뮬레이터: 어깨 24°, 팔꿈치 26°).
WRIST_LIMIT_MARGIN_RAD = np.radians(5.0)
SUPPORTED_MODELS = ("a", "m")
STREAM_HOLD_TIME_S = 30.0
MAX_DT_S = 0.05
STATUS_PERIOD_S = 0.1
COLLISION_QUERY_COUNT = 5
ALIGN_TOLERANCE_M = 1e-4  # 정렬 목표가 이만큼 가까워지면 끝낸다.
ALIGN_AXES = {"x": "앞뒤(x)", "y": "좌우(y)", "z": "높이(z)"}
SINGULARITY_RELEASE_RATIO = 1.2  # 특이점 근접은 최소 특이값이 기준의 이 배수를 넘어야 풀린다 (경계에서 반복 방지)
SINGULARITY_PROBE_M = 0.002  # 특이점 쪽 방향을 볼 때 TCP를 움직여 보는 크기 (이동)
SINGULARITY_PROBE_RAD = 0.01  # 같은 용도 (회전)
JOINT_LIMIT_NEAR_RAD = np.radians(5.0)  # 한도 안내: 한도까지 이만큼 안쪽인 관절을 "한도 근처"로 본다
STALL_S = 0.3  # 목표가 선행 한도까지 앞선 채 이만큼 지나면 관절 한도 때문인지 본다
STALL_RECHECK_S = 0.5
FOLLOW_HOLD_LOG_M = 0.002  # 따르는 팔 때문에 메인 팔 목표가 이만큼 넘게 줄면 로그를 남긴다 (한 번 밀 때 한 번)


def detect_model(address):
    """로봇 정보에서 모델(a/m)을 읽는다."""
    probe = rby.create_robot_a(address)

    try:
        if not probe.connect(max_retries=1, timeout_ms=3000):
            raise ConnectionError(f"로봇에 연결하지 못했습니다: {address}")

        name = probe.get_robot_info().robot_model_name.strip().lower()
    finally:
        with suppress(Exception):
            probe.disconnect()

    model = name[:1]

    if model not in SUPPORTED_MODELS:
        raise ValueError(f"지원하지 않는 로봇 모델입니다: {name!r} (A, M만 지원)")

    return model


def lead_limits():
    """목표가 실제 자세보다 앞설 수 있는 (거리 m, 각도 rad). 임피던스는 힘을 낼 만큼 앞서야 해서 따로 둔다."""
    if settings.IMPEDANCE_ENABLED:
        return settings.IMPEDANCE_MAX_TARGET_LEAD_M, settings.IMPEDANCE_MAX_TARGET_LEAD_RAD

    return settings.MAX_TARGET_LEAD_M, settings.MAX_TARGET_LEAD_RAD


def _rotation_lead(target, actual):
    """실제 자세에서 목표 자세까지의 base 기준 회전 벡터."""
    return Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()


def drop_opposing_lead(target, actual, twist):
    """목표가 실제 자세보다 입력 반대쪽으로 앞서 있으면 그 성분을 버린다.

    팔이 끝에 닿아 목표만 앞서 쌓인 뒤 반대로 밀면, 쌓인 만큼 되돌리느라
    한동안 안 움직이는 것을 막는다. 입력과 수직인 성분은 그대로 둔다.
    """
    twist = np.asarray(twist, dtype=float)
    result = np.array(target, dtype=float, copy=True)

    lead = result[:3, 3] - actual[:3, 3]
    velocity = twist[:3]

    if np.dot(velocity, lead) < 0.0:
        lead -= np.dot(lead, velocity) / np.dot(velocity, velocity) * velocity
        result[:3, 3] = actual[:3, 3] + lead

    rotation = _rotation_lead(result, actual)
    angular = twist[3:]

    if np.dot(angular, rotation) < 0.0:
        rotation -= np.dot(rotation, angular) / np.dot(angular, angular) * angular
        result[:3, :3] = Rotation.from_rotvec(rotation).as_matrix() @ actual[:3, :3]

    return result


def clamp_lead(target, actual, max_position, max_rotation):
    """목표가 실제 자세보다 max_position [m], max_rotation [rad]보다 앞서지 않게 줄인다."""
    result = np.array(target, dtype=float, copy=True)

    lead = result[:3, 3] - actual[:3, 3]
    distance = np.linalg.norm(lead)

    if distance > max_position:
        result[:3, 3] = actual[:3, 3] + lead * (max_position / distance)

    rotation = _rotation_lead(result, actual)
    angle = np.linalg.norm(rotation)

    if angle > max_rotation:
        result[:3, :3] = (
            Rotation.from_rotvec(rotation * (max_rotation / angle)).as_matrix()
            @ actual[:3, :3]
        )

    return result


def integrate(pose, twist, dt, linear_speed, angular_speed):
    """4x4 pose를 base 기준 twist로 dt만큼 움직인 새 pose를 반환한다."""
    twist = np.asarray(twist, dtype=float)
    result = np.array(pose, dtype=float, copy=True)
    result[:3, 3] += twist[:3] * linear_speed * dt

    # base 기준 회전이므로 왼쪽에서 곱한다.
    result[:3, :3] = (
        Rotation.from_rotvec(twist[3:] * angular_speed * dt).as_matrix()
        @ result[:3, :3]
    )

    return result


def arm_of_link(link):
    if "right" in link or link.startswith("ee_finger_r"):
        return "right"

    if "left" in link or link.startswith("ee_finger_l"):
        return "left"

    return None


def tcp_jacobian(body_jacobian, ee_pose, tcp_offset):
    """SDK body 자코비안([ee 각속도; ee 원점 속도], ee 좌표계)을 TCP 점, base 방향의 [선속도; 각속도]로 바꾼다.

    twist와 같은 기준이라 특이값·방향을 입력과 바로 비교할 수 있다.
    """
    rotation = ee_pose[:3, :3]
    angular = rotation @ body_jacobian[:3]
    offset = rotation @ np.asarray(tcp_offset, dtype=float)
    # TCP 속도 = ee 원점 속도 + 각속도 × (ee 원점 → TCP)
    linear = rotation @ body_jacobian[3:] + np.cross(angular.T, offset).T
    return np.vstack([linear, angular])


def remove_toward(velocity, direction):
    """velocity [선속도, 각속도]에서 direction(단위 벡터) 쪽으로 가는 성분만 지운다. 반대쪽은 그대로 둔다."""
    velocity = np.asarray(velocity, dtype=float)
    along = float(np.dot(velocity, direction))
    return velocity - along * direction if along > 0.0 else velocity


def blocking_joints(jacobian, velocity, q, lower, upper, margin):
    """velocity로 가려면 한도까지 margin 안쪽인 관절이 한도 쪽으로 가야 해서 못 가는지 본다.

    못 가면 그렇게 막는 가장 작은 관절 묶음(팔 관절 번호)을, 갈 수 있으면 빈 목록을 반환한다.
    한도 근처 관절은 한도 반대쪽으로만 움직일 수 있다고 두고 J·dq = velocity를 만족하는 dq가 있는지 푼다.
    """
    # 관절 속도 범위(±1)에 들어오도록 크기를 작게 맞춘다. 방향만 본다.
    velocity = np.asarray(velocity, dtype=float)
    velocity = velocity / np.linalg.norm(velocity) * 1e-3
    near = {}

    for joint in range(len(q)):
        if upper[joint] - q[joint] < margin:
            near[joint] = (-1.0, 0.0)
        elif q[joint] - lower[joint] < margin:
            near[joint] = (0.0, 1.0)

    def feasible(limited):
        bounds = [near[j] if j in limited else (-1.0, 1.0) for j in range(len(q))]
        result = linprog(np.zeros(len(q)), A_eq=jacobian, b_eq=velocity, bounds=bounds, method="highs")
        return result.status == 0

    if not near or feasible(set(near)):
        return []

    for size in range(1, len(near) + 1):
        for subset in itertools.combinations(sorted(near), size):
            if not feasible(set(subset)):
                return list(subset)

    return sorted(near)


def collision_filter(side, twist, pairs):
    """가까운 링크 쌍(pairs)에 걸린 팔의 twist에서 허용되는 부분만 남긴다.

    상대 링크에서 멀어지는 이동만 남기고 회전은 0으로 만든다.
    같은 팔 안의 두 링크가 가까우면 방향을 정할 수 없으므로 모두 막는다.
    """
    twist = np.array(twist, dtype=float)

    for pair in pairs:
        arm1, arm2 = arm_of_link(pair.link1), arm_of_link(pair.link2)

        if side not in (arm1, arm2):
            continue

        twist[3:] = 0.0

        if arm1 == arm2:
            twist[:3] = 0.0
            continue

        mine, other = (
            (pair.position1, pair.position2) if arm1 == side
            else (pair.position2, pair.position1)
        )
        away = np.asarray(mine, dtype=float) - np.asarray(other, dtype=float)

        if np.dot(twist[:3], away) <= 0.0:
            twist[:3] = 0.0

    return twist


class RobotStopped(RuntimeError):
    """로봇이 특이점·관절 한도 등으로 스스로 제어를 끝냈다. reinitialize()로 다시 시작할 수 있다."""


class Rby1Controller:
    def __init__(self, events, address=settings.ROBOT_ADDRESS):
        for key in ("max_target_lead_m", "max_target_lead_rad", "impedance_max_target_lead_m", "impedance_max_target_lead_rad"):
            if not getattr(settings, key.upper()) > 0.0:
                raise ValueError(f"{key}는 0보다 커야 합니다: {getattr(settings, key.upper())!r}")

        if not settings.SINGULARITY_SIGMA_MIN >= 0.0:
            raise ValueError(f"singularity_sigma_min은 0 이상이어야 합니다: {settings.SINGULARITY_SIGMA_MIN!r}")

        if not settings.COMMAND_MINIMUM_TIME_S > settings.CONTROL_PERIOD_S:
            raise ValueError(
                f"command_minimum_time_s는 제어 주기({settings.CONTROL_PERIOD_S}s)보다 커야 합니다: "
                f"{settings.COMMAND_MINIMUM_TIME_S!r}"
            )

        # 명령의 속도 한도가 linear_velocity라 그보다 빠른 정렬 속도는 낼 수 없다.
        if not 0.0 < settings.ALIGN_VELOCITY <= settings.LINEAR_VELOCITY:
            raise ValueError(
                f"align_velocity는 0보다 크고 linear_velocity({settings.LINEAR_VELOCITY}) 이하여야 합니다: "
                f"{settings.ALIGN_VELOCITY!r}"
            )

        self.events = events
        self.address = address

        self._robot = None
        self._model = None
        self._dynamics = None
        self._fk_state = None
        self._stream = None
        self._gripper = None

        # 그리퍼 몸통(ee) 좌표계에서 본 역기구학 기준점(TCP). 목표·실제 자세는 모두 TCP 자세로 다룬다.
        self._tcp = np.eye(4)
        self._tcp[:3, 3] = settings.TCP_OFFSET_M
        self._tcp_inverse = np.linalg.inv(self._tcp)
        self._torso = np.eye(4)  # _read_poses()가 채우는 base 기준 link_torso_5 자세
        self._q = None
        self._wrist_only = {side: False for side in SIDES}
        self._moving = {side: False for side in SIDES}  # 직전 주기에 그 팔 입력이 있었는지
        self._wrist_capped = {side: False for side in SIDES}  # 손목 한도로 회전을 막고 있는지 (로그 한 번만)
        self._wrist_limits = None  # connect()가 채우는 팔별 arm_6 (하한, 상한) [rad]
        self._arm_limits = None  # connect()가 채우는 팔별 7관절 (하한 배열, 상한 배열) [rad]
        # 특이점에서 빠져나오는 방향을 볼 때 쓰는 FK 상태. 이번 주기 FK(_fk_state)를 건드리지 않게 따로 둔다.
        self._probe_state = None
        self._jacobians = {}  # 이번 주기 팔별 TCP 자코비안 (6×7, [선속도; 각속도], base 방향)
        self._sigma = {side: np.inf for side in SIDES}  # 이번 주기 TCP 자코비안의 최소 특이값
        self._singular = {side: False for side in SIDES}
        self._toward = {side: None for side in SIDES}  # 특이점 근접일 때 특이점 쪽으로 가는 [선속도, 각속도] 단위 방향
        self._stall_since = {side: None for side in SIDES}
        self._stall_checked = {side: 0.0 for side in SIDES}
        self._stall_reported = {side: False for side in SIDES}
        self._actual = None  # 이번 주기에 읽은 실제 TCP 자세
        self._align_goal = {side: None for side in SIDES}  # 정렬 중인 팔의 (축, 목표값 [m], 시작할 때의 목표 자세)
        # 팔로우 기준: 팔로우를 시작할 때의 (메인 팔 실제 자세, 따르는 팔 목표 자세). None이면 다음 주기에 다시 잡는다.
        self._follow_anchor = None
        self._follower_holding = False  # 따르는 팔이 못 따라와 메인 팔을 붙잡고 있다고 로그를 남겼는지
        self.last_cycle = None

        self._targets = None
        self._reset_reference = True
        self._last_update = None
        self._last_status = 0.0
        self._blocked = False
        self._gripper_closed = None

    @property
    def is_connected(self):
        return self._stream is not None

    def connect(self):
        model = detect_model(self.address)
        self._robot = rby.create_robot(self.address, model)

        if not self._robot.connect():
            raise ConnectionError(f"로봇에 연결하지 못했습니다: {self.address}")

        self._model = self._robot.model()
        self._dynamics = self._robot.get_dynamics()
        self._fk_state = self._dynamics.make_state(
            ["base", *EE_LINKS.values(), TORSO_LINK],
            self._model.robot_joint_names,
        )
        lower = self._dynamics.get_limit_q_lower(self._fk_state)
        upper = self._dynamics.get_limit_q_upper(self._fk_state)
        self._wrist_limits = {
            side: (lower[self._arm_idx(side)[6]], upper[self._arm_idx(side)[6]]) for side in SIDES
        }
        self._arm_limits = {
            side: (np.asarray(lower)[self._arm_idx(side)], np.asarray(upper)[self._arm_idx(side)]) for side in SIDES
        }
        self._probe_state = self._dynamics.make_state(
            ["base", *EE_LINKS.values(), TORSO_LINK],
            self._model.robot_joint_names,
        )

        self._power_on()

        if settings.USE_GRIPPER:
            from .gripper import Gripper

            self._ensure_tool_flange_12v()
            self._gripper = Gripper()
            self._gripper.connect()

        self.return_to_ready()

    def _tool_flange_voltage(self, side):
        return int(getattr(self._robot.get_state(), f"tool_flange_{side}").output_voltage)

    def _ensure_tool_flange_12v(self):
        """그리퍼 전원(툴 플랜지)을 12V로 맞춘다. 12V면 그대로, 꺼져 있으면 켜고, 다른 전압이면 끈 뒤 켠다.

        다른 패키지가 다른 전압으로 켜 둔 채 넘어오면 그리퍼 homing이 제대로 안 되던 문제가 있었다.
        """
        changed = False

        for side in SIDES:
            name = SIDE_NAMES[side]
            voltage = self._tool_flange_voltage(side)

            if voltage == 12:
                self.events.emit(LogMessage(f"{name} 툴 플랜지: 이미 12V, 그대로 진행합니다."))
                continue

            if voltage != 0:
                self.events.emit(LogMessage(f"{name} 툴 플랜지: {voltage}V로 켜져 있어 끈 뒤 12V로 다시 켭니다."))

                if not self._robot.set_tool_flange_output_voltage(side, 0):
                    raise RuntimeError(f"{name} 툴 플랜지 전원을 끄지 못했습니다.")

                time.sleep(0.5)
            else:
                self.events.emit(LogMessage(f"{name} 툴 플랜지: 꺼져 있어 12V로 켭니다."))

            if not self._robot.set_tool_flange_output_voltage(side, 12):
                raise RuntimeError(f"{name} 툴 플랜지를 12V로 켜지 못했습니다.")

            deadline = time.monotonic() + 2.0

            while self._tool_flange_voltage(side) != 12:
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        f"{name} 툴 플랜지가 2초 안에 12V가 되지 않았습니다 (지금 {self._tool_flange_voltage(side)}V)."
                    )

                time.sleep(0.1)

            changed = True

        # 전원을 바꿨으면 그리퍼 모터가 켜질 때까지 잠시 기다린다. (공식 lerobot-robot-rby1과 같은 0.5초)
        if changed:
            time.sleep(0.5)

    def return_to_ready(self):
        """스트림을 멈추고 레디자세로 이동한 뒤, 그 자세를 목표로 스트림을 다시 연다."""
        if self._stream is not None and not self._stream.is_done():
            self._stream.cancel()

        # 취소된 제어는 로봇 쪽에서 약 0.1초 동안 멈추는 중으로 남는다. 그 사이 보낸 명령은
        # 우선순위가 같아 거부되므로 제어가 끝날 때까지 기다린다. (공식 lerobot-robot-rby1과 같은 방식)
        if not self._robot.wait_for_control_ready(2000):
            raise RuntimeError("로봇 제어가 2초 안에 비지 않아 레디자세로 갈 수 없습니다.")

        self._stream = None
        self.move_to_ready()

        self._targets = self._read_poses()[0]
        self._wrist_only = {side: False for side in SIDES}
        self._moving = {side: False for side in SIDES}
        self._align_goal = {side: None for side in SIDES}
        self._follow_anchor = None
        self._reset_reference = True
        self._last_update = None
        self._stream = self._robot.create_command_stream(priority=1)

    def align_axis(self, main_side, axis):
        """따르는 팔 TCP의 base 축(x, y, z) 하나를 메인 팔에 맞추기 시작한다. 나머지 축과 회전은 그대로 둔다.

        x, z는 메인 팔과 같은 값으로, y는 몸 가운데(base y=0) 기준 좌우 대칭(-y)으로 맞춘다.
        """
        index = "xyz".index(axis)
        follower = SIDES[1 - SIDES.index(main_side)]
        poses = self._read_poses()[0]
        goal = poses[main_side][index, 3] * (-1.0 if axis == "y" else 1.0)
        self._align_goal = {side: None for side in SIDES}
        self._align_goal[follower] = (axis, goal, self._targets[follower].copy())
        self.events.emit(LogMessage(
            f"{SIDE_NAMES[follower]} {ALIGN_AXES[axis]}를 {SIDE_NAMES[main_side]}에 "
            f"{'좌우 대칭으로 ' if axis == 'y' else ''}맞춥니다 ({(goal - poses[follower][index, 3]) * 100:+.1f} cm). "
            "그 팔 마우스를 움직이면 취소됩니다."
        ))

    def _align_step(self, side, actual, near, dt):
        """정렬 중인 팔 목표: 정렬을 시작할 때의 목표 자세에서 맞추는 축 값만 목표값 쪽으로 align_velocity로 옮긴다.

        목표 전체에 선행 한도를 걸면 팔이 따라오며 생긴 다른 축 편차가 목표에 굳어 그 축이 틀어지므로
        (시뮬레이터: y 정렬 중 x·z 34 mm), 선행 한도는 맞추는 축에만 건다.
        """
        axis, goal, start = self._align_goal[side]
        index = "xyz".index(axis)
        value = self._targets[side][index, 3]
        self._wrist_only[side] = False
        self._moving[side] = False

        if abs(goal - value) >= ALIGN_TOLERANCE_M:
            lead_m = lead_limits()[0]
            direction = np.zeros(6)
            direction[index] = np.sign(goal - value)

            if not np.any(collision_filter(side, direction, near)):
                self._end_align(side, f"{SIDE_NAMES[side]} 충돌 경고로 정렬을 멈춥니다.")
                return

            # 특이점 쪽 성분을 지우고 나서 맞추는 축 방향이 절반도 안 남으면 그 축으로는 특이점에 들어가는 중이다.
            if abs(self._guard_singularity(side, direction)[index]) < 0.5:
                self._end_align(side, f"{SIDE_NAMES[side]} 특이점 근처라 정렬을 멈춥니다.")
                return

            reach = settings.ALIGN_VELOCITY * dt
            value += np.clip(goal - value, -reach, reach)
            value = np.clip(
                value,
                actual[side][index, 3] - lead_m,
                actual[side][index, 3] + lead_m,
            )

        self._targets[side] = start.copy()
        self._targets[side][index, 3] = value

        if abs(goal - value) < ALIGN_TOLERANCE_M:
            self._targets[side][index, 3] = goal
            self._end_align(side, f"{SIDE_NAMES[side]} {ALIGN_AXES[axis]} 정렬 완료.")


    def _end_align(self, side, message):
        self._align_goal[side] = None
        # 목표를 지금 그대로 두고 멈춘다. (손 뗄 때처럼 실제 자세로 바꾸지 않는다)
        self._moving[side] = False
        self.events.emit(LogMessage(message))

    def reinitialize(self):
        """제어가 멈춘 뒤 fault를 지우고 control manager를 unlimited로 다시 켠 다음 레디자세로 돌아간다.

        관절이 한도 밖에서 멈췄으면 일반 모드로는 켜지지 않는다. unlimited는 프로그램이 끝날 때까지 유지한다.
        """
        if self._stream is not None and not self._stream.is_done():
            self._stream.cancel()

        self._stream = None
        self.events.emit(LogMessage("재초기화: fault를 지우고 control manager를 unlimited로 다시 켭니다."))
        self._power_on(unlimited=True)
        self.return_to_ready()
        self.events.emit(LogMessage("재초기화 완료. 지금부터 unlimited 모드로 제어합니다."))

    def _power_on(self, unlimited=settings.UNLIMITED_MODE):
        robot = self._robot
        robot.reset_fault_control_manager()

        if not robot.is_power_on(".*") and not robot.power_on(".*"):
            raise RuntimeError("전원을 켜지 못했습니다.")

        if not robot.is_servo_on(".*") and not robot.servo_on(".*"):
            raise RuntimeError("서보를 켜지 못했습니다.")

        robot.reset_fault_control_manager()

        if not robot.enable_control_manager(unlimited_mode_enabled=unlimited):
            raise RuntimeError("control manager를 켜지 못했습니다.")

    def move_to_ready(self):
        """레디자세로 관절 위치 이동하고 끝날 때까지 기다린다."""
        pose = settings.ready_pose_rad()

        def joint(position):
            return (
                rby.JointPositionCommandBuilder()
                .set_position(position)
                .set_minimum_time(settings.READY_MINIMUM_TIME_S)
            )

        command = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_torso_command(joint(pose["torso"]))
                .set_right_arm_command(joint(pose["right_arm"]))
                .set_left_arm_command(joint(pose["left_arm"]))
            )
        )
        feedback = self._robot.send_command(command).get()

        if feedback.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            raise RuntimeError(f"레디자세 이동이 끝나지 않았습니다: {feedback.finish_code}")

    def _read_poses(self):
        """현재 TCP 자세와 가까운 링크 쌍 목록을 반환한다."""
        q = np.asarray(self._robot.get_state().position, dtype=float)
        self._q = q
        self._fk_state.set_q(q)
        self._dynamics.compute_forward_kinematics(self._fk_state)

        poses = {
            side: np.array(self._dynamics.compute_transformation(self._fk_state, 0, index)) @ self._tcp
            for index, side in enumerate(SIDES, start=1)
        }
        self._torso = np.array(self._dynamics.compute_transformation(self._fk_state, 0, len(SIDES) + 1))
        self._jacobians = {side: self._arm_jacobian(self._fk_state, side) for side in SIDES}
        pairs = self._dynamics.detect_collisions_or_nearest_links(
            self._fk_state, COLLISION_QUERY_COUNT
        )
        return poses, pairs

    def send_action(self, action):
        if not self.is_connected:
            raise RuntimeError("Call connect() first.")

        now = time.perf_counter()
        dt = 0.0 if self._last_update is None else min(now - self._last_update, MAX_DT_S)
        self._last_update = now

        actual, pairs = self._read_poses()
        self._actual = actual
        threshold = settings.COLLISION_DISTANCE_M + (
            settings.COLLISION_RELEASE_MARGIN_M if self._blocked else 0.0
        )
        near = [pair for pair in pairs if pair.distance < threshold]
        min_distance = min((pair.distance for pair in pairs), default=float("inf"))
        blocked = bool(near)

        if blocked != self._blocked:
            self._blocked = blocked
            pair = near[0] if near else pairs[0]
            self.events.emit(
                CollisionChanged(blocked, pair.distance, pair.link1, pair.link2)
            )

            # 막히는 순간, 걸린 팔은 앞서 있던 목표를 버리고 지금 자세에 멈춘다. 팔로우 기준도 다시 잡는다.
            if blocked:
                self._follow_anchor = None

                for side in SIDES:
                    if any(side in (arm_of_link(p.link1), arm_of_link(p.link2)) for p in near):
                        self._targets[side] = actual[side]

        self._update_singularity()
        linear_speed = settings.LINEAR_VELOCITY
        angular_speed = settings.ANGULAR_VELOCITY
        scale = np.array([linear_speed] * 3 + [angular_speed] * 3)  # twist(-1~1) → [m/s, rad/s]
        inputs = {}
        arm_mode = ARM_MODES[int(action["follow.mode"])]
        leader = settings.MAIN_ARM
        follower = SIDES[1 - SIDES.index(leader)]

        # 팔로우 중에는 따르는 팔을 먼저 계산해, 메인 팔이 이번 주기 따르는 팔 목표를 보고 붙잡히게 한다.
        for side in (follower, leader) if arm_mode != "normal" else SIDES:
            twist = np.array([action[f"{side}_ee.{key}"] for key in ("x", "y", "z", "rx", "ry", "rz")])
            aligning = self._align_goal[side] is not None

            if aligning and np.any(twist != 0.0):
                self._end_align(side, f"{SIDE_NAMES[side]} 마우스 입력이 들어와 정렬을 취소합니다.")
                aligning = False
            elif aligning:
                inputs[side] = twist

                if side == follower:
                    self._follow_anchor = None

                self._align_step(side, actual, near, dt)
                continue

            if side == follower and arm_mode != "normal":
                # 입력을 그대로 쓰면 메인 팔이 특이점·도달 한계·충돌 가드로 덜 가도 이 팔은 계속 가서 점점 어긋난다.
                inputs[side] = twist
                self._follow_leader(side, actual, near, leader, arm_mode == "mirroring")
                continue

            if side == follower:
                self._follow_anchor = None

            twist = collision_filter(side, twist, near)
            twist = self._guard_singularity(side, twist * scale) / scale
            inputs[side] = twist
            moving = bool(np.any(twist != 0.0))
            self._wrist_only[side] = moving and action[f"{side}_ee.wrist_only"] > 0.5

            if moving:
                self._drop_lead_toward_singularity(side, actual[side])
                target = drop_opposing_lead(self._targets[side], actual[side], twist)
                target = integrate(target, twist, dt, linear_speed, angular_speed)
                self._targets[side] = clamp_lead(
                    target,
                    actual[side],
                    *lead_limits(),
                )

                if side == leader and arm_mode != "normal" and self._follow_anchor is not None:
                    self._hold_by_follower(side, follower, arm_mode == "mirroring")

                if self._wrist_only[side]:
                    self._cap_wrist_target(side)

                self._check_stall(side, actual[side], twist * scale)
            elif self._moving[side]:
                # 손을 뗀 순간 목표를 실제 자세로 맞춰, 앞서 있던 목표까지 더 가지 않고 바로 멈춘다.
                # 그 뒤로는 이 자세를 계속 목표로 둔다(매 주기 실제 자세로 바꾸면 처지는 만큼 따라 내려간다).
                self._targets[side] = actual[side].copy()

            self._moving[side] = moving

            if not moving:
                self._stall_since[side] = None
                self._stall_reported[side] = False

            if side == leader and not moving:
                self._follower_holding = False

        # 로봇이 특이점·관절 한도 등으로 스스로 제어를 끝내면 스트림이 끝난다.
        if self._stream.is_done():
            raise RobotStopped(self._stop_reason())

        try:
            self._stream.send_command(self._build_command())
        except RuntimeError as exc:
            raise RobotStopped(self._stop_reason()) from exc

        self._reset_reference = False
        self._send_gripper(action)

        # 실행 기록용: 이번 주기에 보낸 값과 읽은 값
        self.last_cycle = {
            "inputs": inputs,
            "targets": {side: self._targets[side].copy() for side in SIDES},
            "actuals": actual,
            "arm_q": {
                "right": self._q[self._model.right_arm_idx],
                "left": self._q[self._model.left_arm_idx],
            },
            "min_distance": min_distance,
            "blocked": blocked,
            "sigma": dict(self._sigma),
        }

        if now - self._last_status >= STATUS_PERIOD_S:
            self._last_status = now
            self.events.emit(StatusUpdated(
                min_distance,
                {side: self._targets[side].copy() for side in SIDES},
                actual,
                inputs,
                self._tcp.copy(),
                self._torso.copy(),
            ))

    def _follow_leader(self, side, actual, near, leader, mirroring):
        """따르는 팔 목표 = 기준 자세 + 메인 팔이 기준에서 실제로 움직인 만큼 (미러링이면 대칭으로).

        매 주기 이동량을 더하면 선행 한도에 잘린 몫이 사라져 어긋남이 쌓이므로(시뮬레이터: 3초에 33 mm),
        기준에서의 변화로 매번 다시 계산한다.
        """
        self._wrist_only[side] = False
        self._moving[side] = False

        if self._follow_anchor is None:
            self._follow_anchor = (actual[leader].copy(), self._targets[side].copy())

        leader_start, follower_start = self._follow_anchor
        motion = np.concatenate([
            actual[leader][:3, 3] - leader_start[:3, 3],
            Rotation.from_matrix(actual[leader][:3, :3] @ leader_start[:3, :3].T).as_rotvec(),
        ])
        desired = integrate(follower_start, mirror(motion) if mirroring else motion, 1.0, 1.0, 1.0)

        # 충돌 가드: 지금 목표에서 원하는 목표로 가는 방향이 막히면 막힌 만큼만 옮긴다. 기준은 그대로 두어
        # 메인 팔이 이 팔 목표에 붙잡히게 한다(_hold_by_follower).
        step = np.concatenate([
            desired[:3, 3] - self._targets[side][:3, 3],
            _rotation_lead(desired, self._targets[side]),
        ])
        allowed = self._guard_singularity(side, collision_filter(side, step, near))
        self._drop_lead_toward_singularity(side, actual[side])

        if np.array_equal(allowed, step):
            target = desired
        else:
            target = integrate(self._targets[side], allowed, 1.0, 1.0, 1.0)

        self._targets[side] = clamp_lead(
            target, actual[side], *lead_limits()
        )

    def _arm_jacobian(self, state, side):
        """state의 FK로 그 팔의 TCP 자코비안(6×7)을 만든다. compute_forward_kinematics 뒤에 부른다."""
        index = SIDES.index(side) + 1
        ee_pose = np.array(self._dynamics.compute_transformation(state, 0, index))
        body = np.asarray(self._dynamics.compute_body_jacobian(state, 0, index))[:, self._arm_idx(side)]
        return tcp_jacobian(body, ee_pose, settings.TCP_OFFSET_M)

    def _update_singularity(self):
        """팔마다 TCP 자코비안의 최소 특이값을 보고, 기준 아래면 특이점 쪽 방향을 구해 둔다.

        로봇은 특이점에 닿으면 스스로 제어를 끝낸다(Singularity). 그 전에 그쪽 입력만 막아 멈추지 않게 한다.
        """
        for side in SIDES:
            sigma = np.linalg.svd(self._jacobians[side], compute_uv=False)[-1]
            self._sigma[side] = sigma
            threshold = settings.SINGULARITY_SIGMA_MIN * (
                SINGULARITY_RELEASE_RATIO if self._singular[side] else 1.0
            )
            near = sigma < threshold
            self._toward[side] = self._singular_toward(side) if near else None

            if near != self._singular[side]:
                self._singular[side] = near
                self.events.emit(SingularityChanged(side, near, float(sigma)))

    def _singular_toward(self, side):
        """TCP를 움직이면 최소 특이값이 줄어드는 방향([선속도, 각속도] 단위 벡터). 축마다 조금 움직여 본 기울기로 구한다.

        최소 특이값 방향(특이 벡터) 성분만 막으면 나머지 성분으로 계속 특이점에 들어가 멈췄다
        (시뮬레이터: 위 +z, 회전 rx). 그래서 최소 특이값을 실제로 깎는 방향을 막는다.
        """
        inverse = np.linalg.pinv(self._jacobians[side])
        gradient = np.zeros(6)

        for axis in range(6):
            step = SINGULARITY_PROBE_M if axis < 3 else SINGULARITY_PROBE_RAD
            q = self._q.copy()
            q[self._arm_idx(side)] += inverse[:, axis] * step
            self._probe_state.set_q(q)
            self._dynamics.compute_forward_kinematics(self._probe_state)
            sigma = np.linalg.svd(self._arm_jacobian(self._probe_state, side), compute_uv=False)[-1]
            gradient[axis] = (sigma - self._sigma[side]) / step

        norm = np.linalg.norm(gradient)
        return -gradient / norm if norm > 0.0 else None

    def _guard_singularity(self, side, velocity):
        """특이점 근접이면 velocity [선속도, 각속도]에서 특이점 쪽 성분만 지운다."""
        toward = self._toward[side]
        return np.asarray(velocity, dtype=float) if toward is None else remove_toward(velocity, toward)

    def _drop_lead_toward_singularity(self, side, actual):
        """특이점 근접이면 앞서 있던 목표를 모두 버리고 실제 자세에서 다시 시작한다.

        도달 한계에서 목표가 앞서 있으면 TCP는 거의 그대로인데 로봇 IK가 관절만 돌려 0.2초 만에 특이점에 들어갔다
        (시뮬레이터: 위 +z, 최소 특이값 0.107 → 0.033). 특이점 쪽 성분만 버리면 막지 못했다.
        """
        if self._toward[side] is not None:
            self._targets[side] = actual.copy()

    def _check_stall(self, side, actual, velocity):
        """목표가 선행 한도까지 앞선 채(팔이 못 따라옴) STALL_S가 지나면 관절 한도 때문인지 보고 로그를 남긴다.

        특이점이 아니라 관절 한도로 막히는 경우(예: 레디자세 손목 arm_6가 한도 근처)는 자코비안 랭크로 알 수 없어 따로 본다.
        """
        now = time.perf_counter()
        target = self._targets[side]
        lead_m, lead_rad = lead_limits()
        saturated = (
            np.linalg.norm(target[:3, 3] - actual[:3, 3]) > 0.9 * lead_m
            or np.linalg.norm(_rotation_lead(target, actual)) > 0.9 * lead_rad
        )

        if not saturated:
            self._stall_since[side] = None
            return

        if self._stall_since[side] is None:
            self._stall_since[side] = now

        if (
            self._stall_reported[side]
            or now - self._stall_since[side] < STALL_S
            or now - self._stall_checked[side] < STALL_RECHECK_S
        ):
            return

        self._stall_checked[side] = now
        q = self._q[self._arm_idx(side)]
        lower, upper = self._arm_limits[side]
        joints = blocking_joints(self._jacobians[side], velocity, q, lower, upper, JOINT_LIMIT_NEAR_RAD)

        if joints:
            self._stall_reported[side] = True
            detail = ", ".join(
                f"arm_{j}({np.degrees(q[j]):.0f}°, 한도 "
                f"{np.degrees(upper[j] if upper[j] - q[j] < q[j] - lower[j] else lower[j]):.0f}°)"
                for j in joints
            )
            self.events.emit(LogMessage(f"{SIDE_NAMES[side]} {detail} 한도 때문에 그 방향으로 더 못 갑니다."))

    def _hold_by_follower(self, leader, follower, mirroring):
        """메인 팔 목표가 따르는 팔이 따라온 만큼(따르는 팔 목표를 메인 팔 쪽으로 되돌린 자세)보다 선행 한도 넘게 앞서지 않게 한다.

        따르는 팔이 관절 한도·충돌 가드로 못 가면 그 목표가 멈추므로 메인 팔도 멈춘다. 정상적으로 따라올 때는
        되돌린 자세가 메인 팔 실제 자세와 같아 원래 선행 한도와 같다.
        """
        leader_start, follower_start = self._follow_anchor
        follower_target = self._targets[follower]
        motion = np.concatenate([
            follower_target[:3, 3] - follower_start[:3, 3],
            Rotation.from_matrix(follower_target[:3, :3] @ follower_start[:3, :3].T).as_rotvec(),
        ])
        reached = integrate(leader_start, mirror(motion) if mirroring else motion, 1.0, 1.0, 1.0)
        held = clamp_lead(self._targets[leader], reached, *lead_limits())

        if (
            np.linalg.norm(held[:3, 3] - self._targets[leader][:3, 3]) > FOLLOW_HOLD_LOG_M
            and not self._follower_holding
        ):
            self._follower_holding = True
            self.events.emit(LogMessage(
                f"{SIDE_NAMES[follower]}이 따라오지 못해 {SIDE_NAMES[leader]}도 더 가지 않습니다."
            ))

        self._targets[leader] = held

    def get_feedback(self):
        """teleoperator.send_feedback()에 줄 목표 그리퍼 자세(base 기준 회전 벡터)."""
        feedback = {}

        for side in SIDES:
            rotvec = Rotation.from_matrix(self._targets[side][:3, :3]).as_rotvec()
            feedback.update({f"{side}_ee.{key}": float(v) for key, v in zip(("rx", "ry", "rz"), rotvec)})

        return feedback

    def _stop_reason(self):
        """control manager 상태와 마지막 제어 오류 메시지로 중단 원인을 만든다."""
        state = self._robot.get_control_manager_state().state
        message = "로그에서 원인을 찾지 못함"

        for log in reversed(self._robot.get_last_log(50)):
            found = re.search(r'"message":"([^"]+)"', str(log))

            if found:
                message = found.group(1)
                break

        return f"로봇이 제어를 중단했습니다: {message} ({state.name})"

    def _arm_idx(self, side):
        return self._model.right_arm_idx if side == "right" else self._model.left_arm_idx

    def _wrist_remaining(self, side):
        """실제 그리퍼에서 목표 그리퍼까지 남은 그리퍼 z축 회전각 [rad]. 손목만 회전 중일 때만 의미가 있다."""
        return Rotation.from_matrix(self._actual[side][:3, :3].T @ self._targets[side][:3, :3]).as_rotvec()[2]

    def _wrist_joint_target(self, side):
        """그리퍼 z축 회전만 할 때 arm_6 보조 목표: 지금 arm_6 각도 + 남은 z축 회전각."""
        return f"{side}_arm_6", float(self._q[self._arm_idx(side)[6]] + self._wrist_remaining(side))

    def _cap_wrist_target(self, side):
        """손목만 회전 중일 때 arm_6 목표가 한도 여유 안으로 들어가지 않게 목표 회전을 줄인다.

        이미 여유 안에 있으면 밖으로 밀어내지 않고 더 들어가지만 않게 한다.
        """
        q6 = self._q[self._arm_idx(side)[6]]
        lower, upper = self._wrist_limits[side]
        low = min(lower + WRIST_LIMIT_MARGIN_RAD, q6)
        high = max(upper - WRIST_LIMIT_MARGIN_RAD, q6)
        wanted = q6 + self._wrist_remaining(side)
        # 범위 밖인지로 판정한다. clip 뒤 q6를 빼서 remaining과 ==로 비교하면 반올림 오차 때문에
        # 한도와 먼 곳에서도 85%가 막힌 것으로 나와 로그가 잘못 떴다.
        capped = not low <= wanted <= high

        if capped:
            allowed = np.clip(wanted, low, high) - q6
            self._targets[side][:3, :3] = (
                self._actual[side][:3, :3] @ Rotation.from_rotvec([0.0, 0.0, allowed]).as_matrix()
            )

        if capped and not self._wrist_capped[side]:
            self.events.emit(LogMessage(f"{SIDE_NAMES[side]} 손목(arm_6)이 한도 근처라 그 방향 회전을 멈춥니다."))

        self._wrist_capped[side] = capped

    def _build_command(self):
        body = rby.BodyComponentBasedCommandBuilder()
        minimum_time = settings.COMMAND_MINIMUM_TIME_S

        for side in SIDES:
            header = rby.CommandHeaderBuilder().set_control_hold_time(STREAM_HOLD_TIME_S)

            if settings.IMPEDANCE_ENABLED:
                builder = (
                    rby.CartesianImpedanceControlCommandBuilder()
                    .set_command_header(header)
                    .set_minimum_time(minimum_time)
                    .set_joint_stiffness(settings.ARM_STIFFNESS)
                    .set_joint_torque_limit(settings.ARM_TORQUE_LIMIT)
                    .set_joint_damping_ratio(settings.ARM_DAMPING_RATIO)
                    .set_stop_joint_position_tracking_error(0)
                    .set_stop_orientation_tracking_error(0)
                    .set_reset_reference(self._reset_reference)
                    # 자세 유지(set_nullspace_joint_target)는 넣지 않는다. 레디자세 기준이면 손을 떼도 팔이
                    # 레디자세 쪽으로 자세를 바꾸려 하고, 세기를 10으로 올리면 어깨(arm_0)가 64~80도 휘둘렸으며
                    # 비중(cost_weight 0.1~10)은 효과가 없었다 (시뮬레이터, 2026-11-11).
                    .add_joint_limit(f"{side}_arm_3", *settings.ELBOW_LIMIT_RAD)
                    .add_target(
                        "base",
                        EE_LINKS[side],
                        self._targets[side] @ self._tcp_inverse,
                        settings.LINEAR_VELOCITY,
                        settings.ANGULAR_VELOCITY,
                        settings.LINEAR_ACCELERATION,
                        settings.ANGULAR_ACCELERATION,
                    )
                )
            else:
                builder = (
                    rby.CartesianCommandBuilder()
                    .set_command_header(header)
                    .set_minimum_time(minimum_time)
                    .add_target(
                        "base",
                        EE_LINKS[side],
                        self._targets[side] @ self._tcp_inverse,
                        settings.LINEAR_VELOCITY,
                        settings.ANGULAR_VELOCITY,
                        settings.ACCELERATION_LIMIT_SCALING,
                    )
                )
                # 자세 유지용으로 add_joint_position_target을 관절 0~5에 걸어 봤지만 관절이 그 자리에 묶여
                # 그리퍼가 전혀 움직이지 않았다 (시뮬레이터, 2026-11-11). 위치 제어에는 자세 유지를 걸지 않는다.

            if self._wrist_only[side]:
                builder.add_joint_position_target(*self._wrist_joint_target(side))

            if side == "right":
                body.set_right_arm_command(builder)
            else:
                body.set_left_arm_command(builder)

        return rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(body)
        )

    def _send_gripper(self, action):
        closed = {side: action[f"{side}_gripper.closed"] > 0.5 for side in SIDES}

        if self._gripper is not None and closed != self._gripper_closed:
            self._gripper.set_closed(closed)

        self._gripper_closed = closed

    def disconnect(self):
        try:
            if self._stream is not None and not self._stream.is_done():
                self._stream.cancel()
        finally:
            self._stream = None

            if self._gripper is not None:
                self._gripper.disconnect()
                self._gripper = None

            if self._robot is not None:
                self._robot.disconnect()
                self._robot = None
