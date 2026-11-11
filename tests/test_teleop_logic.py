"""버튼 조합, 모드, 미러링, 충돌 필터 로직을 확인한다. 장치와 로봇은 쓰지 않는다.

python tests/test_teleop_logic.py 로 실행한다. (pytest로도 실행된다)
"""

from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from lerobot_teleoperator_spacemouse4rby1 import SpaceMouse4Rby1, SpaceMouse4Rby1Config
from lerobot_teleoperator_spacemouse4rby1.events import EventBus, LogMessage, ModeChanged
from lerobot_teleoperator_spacemouse4rby1.mouse import (
    BUTTON0,
    BUTTON1,
    COMBO,
    FRAMES,
    LOCK_MODES,
    ButtonTracker,
    apply_arm_mode,
    apply_curve,
    apply_lock,
    keep_dominant_axes,
    mirror,
    next_mode,
    to_base,
    tool_to_base,
    x_axis_to_matrix,
)
from lerobot_teleoperator_spacemouse4rby1.robot import (
    Rby1Controller,
    arm_of_link,
    clamp_lead,
    collision_filter,
    drop_opposing_lead,
    integrate,
)


def feed(tracker, steps):
    """steps: (시각, (버튼0, 버튼1)) 목록. 나온 이벤트를 모두 반환한다."""
    events = []

    for now, buttons in steps:
        events += tracker.update(buttons, now)

    return events


def test_combo_within_window():
    steps = [(0.0, (1, 0)), (0.3, (1, 1)), (0.4, (0, 0))]
    assert feed(ButtonTracker(0.5), steps) == [COMBO]


def test_combo_same_report():
    assert feed(ButtonTracker(0.5), [(0.0, (1, 1)), (0.1, (0, 0))]) == [COMBO]


def test_late_second_button_is_not_combo():
    # 0번을 누른 채 0.5초가 지나면 단독 누름으로 확정하고, 이후 1번은 무시한다.
    steps = [(0.0, (1, 0)), (0.6, (1, 0)), (0.7, (1, 1)), (0.8, (0, 0))]
    assert feed(ButtonTracker(0.5), steps) == [BUTTON0]


def test_single_click_on_release():
    assert feed(ButtonTracker(0.5), [(0.0, (1, 0)), (0.1, (0, 0))]) == [BUTTON0]
    assert feed(ButtonTracker(0.5), [(0.0, (0, 1)), (0.1, (0, 0))]) == [BUTTON1]


def test_no_retrigger_while_held():
    steps = [(0.0, (1, 1)), (0.2, (1, 0)), (0.3, (1, 1)), (0.4, (0, 0)), (0.5, (1, 0)), (0.6, (0, 0))]
    assert feed(ButtonTracker(0.5), steps) == [COMBO, BUTTON0]


def test_mode_cycle():
    assert next_mode(LOCK_MODES, "normal") == "position_lock"
    assert next_mode(LOCK_MODES, "rotation_lock") == "normal"
    assert next_mode(FRAMES, "base") == "tool"
    assert next_mode(FRAMES, "tool") == "base"


def test_locks():
    twist = np.arange(1, 7.0)
    assert np.allclose(apply_lock(twist, "position_lock"), [0, 0, 0, 4, 5, 6])
    assert np.allclose(apply_lock(twist, "rotation_lock"), [1, 2, 3, 0, 0, 0])
    assert np.allclose(apply_lock(twist, "normal"), twist)


def test_tracking_and_mirroring_follow_leader():
    right = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    left = np.full(6, 9.0)
    twists = {"right": right, "left": left}
    assert np.allclose(apply_arm_mode(twists, "tracking", "right")["left"], right)
    assert np.allclose(apply_arm_mode(twists, "mirroring", "right")["left"], [0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    # 주도 팔을 왼팔로 바꾸면 오른팔이 따른다.
    assert np.allclose(apply_arm_mode(twists, "tracking", "left")["right"], left)
    assert np.allclose(apply_arm_mode(twists, "normal", "right")["left"], left)


def test_tool_frame_moves_along_gripper_axes():
    # 그리퍼가 base z축으로 90도 돌아 있으면 그리퍼 x 방향 = base y 방향
    rotation = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    assert np.allclose(tool_to_base([1, 0, 0, 0, 0, 1], rotation), [0, 1, 0, 0, 0, 1])
    assert np.allclose(tool_to_base([0, 0, 0, 1, 0, 0], rotation), [0, 0, 0, 0, 1, 0])


def make_teleop():
    events = EventBus()
    modes = []
    events.subscribe(lambda e: modes.append(e) if isinstance(e, ModeChanged) else None)
    return SpaceMouse4Rby1(SpaceMouse4Rby1Config(id="test"), events), modes


def test_buttons_change_own_arm_only():
    teleop, modes = make_teleop()
    teleop._handle_button("left", BUTTON0)
    teleop._handle_button("right", BUTTON1)
    assert teleop.locks == {"right": "normal", "left": "position_lock"}
    assert teleop.frames == {"right": "tool", "left": "base"}
    assert modes[-1].locks["left"] == "position_lock" and modes[-1].frames["right"] == "tool"
    teleop._handle_button("right", COMBO)
    assert teleop.gripper_closed == {"right": True, "left": False}


def test_keys_toggle_arm_mode():
    teleop, modes = make_teleop()
    teleop.handle_key("m")
    assert teleop.arm_mode == "mirroring"
    teleop.handle_key("t")
    assert teleop.arm_mode == "tracking"
    teleop.handle_key("T")
    assert teleop.arm_mode == "normal"
    teleop.handle_key("x")
    assert teleop.arm_mode == "normal" and len(modes) == 3


def test_follower_gets_leader_input_after_frame_and_lock():
    teleop, _ = make_teleop()
    rotation = Rotation.from_euler("z", 90, degrees=True)
    teleop.send_feedback({
        f"{side}_ee.{key}": value
        for side in ("right", "left")
        for key, value in zip(("rx", "ry", "rz"), rotation.as_rotvec())
    })
    teleop.frames["right"] = "tool"
    teleop.locks["right"] = "rotation_lock"
    teleop.arm_mode = "tracking"
    result = teleop.apply_modes({"right": np.array([1.0, 0, 0, 0, 0, 1]), "left": np.full(6, 9.0)})
    # 오른팔: 그리퍼 x → base y, 회전락으로 회전 0. 왼팔은 그 base 입력을 그대로 받는다.
    assert np.allclose(result["right"], [0, 1, 0, 0, 0, 0])
    assert np.allclose(result["left"], [0, 1, 0, 0, 0, 0])


def test_lock_applies_before_axis_filter():
    # 회전락 중 밀면서 기울기(0.3)가 크게 들어와도 이동(y 0.08, z 0.2)은 남아야 한다.
    raw = np.array([0.0, 0.08, 0.2, 0.3, 0.3, 0.0])
    locked_first = to_base(apply_lock(raw, "rotation_lock"), np.eye(3), 0.0, 0.3, 0.0)
    assert np.allclose(locked_first[3:], 0) and locked_first[2] > 0 and locked_first[1] > 0
    # 예전 순서(필터 뒤 락)면 기울기 기준 30% 이하인 y 이동이 지워진다.
    filtered_first = apply_lock(to_base(raw, np.eye(3), 0.0, 0.3, 0.0), "rotation_lock")
    assert filtered_first[1] == 0 and np.allclose(filtered_first[3:], 0)


def test_tool_frame_without_feedback_fails():
    teleop, _ = make_teleop()
    teleop.frames["left"] = "tool"
    try:
        teleop.apply_modes({"right": np.zeros(6), "left": np.ones(6)})
    except RuntimeError:
        return
    raise AssertionError("피드백 없이 그리퍼 좌표계를 썼습니다.")


def test_mirror_matches_reflected_rotation():
    # XZ 평면 반사 M에 대해 M R(w) M == R(mirror(w))
    w = np.array([0.3, -0.7, 0.5])
    reflect = np.diag([1.0, -1.0, 1.0])
    expected = reflect @ Rotation.from_rotvec(w).as_matrix() @ reflect
    mirrored = Rotation.from_rotvec(mirror(np.r_[0, 0, 0, w])[3:]).as_matrix()
    assert np.allclose(expected, mirrored)


def test_integrate_in_base_frame():
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("y", 90, degrees=True).as_matrix()
    moved = integrate(pose, [1, 0, 0, 0, 0, 1], 0.5, 0.1, np.pi / 2)
    assert np.allclose(moved[:3, 3], [0.05, 0, 0])
    expected = Rotation.from_euler("z", 45, degrees=True).as_matrix() @ pose[:3, :3]
    assert np.allclose(moved[:3, :3], expected)


def test_input_curve():
    # 끝과 0은 그대로, 중간은 작아지고 부호는 유지된다.
    values = np.array([1.0, -1.0, 0.0, 0.5, -0.2])
    curved = apply_curve(values, 10.0)
    assert np.allclose(curved[:3], [1.0, -1.0, 0.0])
    assert np.isclose(curved[3], 0.2317, atol=1e-4) and np.isclose(curved[4], -0.0615, atol=1e-4)
    # 0이면 직선
    assert np.allclose(apply_curve(values, 0.0), values)
    # 곡선은 단조 증가
    grid = np.linspace(0, 1, 101)
    assert np.all(np.diff(apply_curve(grid, 10.0)) > 0)


def test_curve_keeps_diagonal_direction():
    # 이동 (0.8, 0.5)를 밀면 방향은 그대로(약 32도)이고 크기만 곡선을 따른다.
    twist = to_base([0.8, 0.5, 0, 0, 0, 0], np.eye(3), deadzone=0.0, axis_ratio=0.3, translation_curve=10.0)
    length = np.hypot(0.8, 0.5)
    assert np.isclose(np.degrees(np.arctan2(twist[1], twist[0])), np.degrees(np.arctan2(0.5, 0.8)))
    assert np.isclose(np.linalg.norm(twist[:3]), (11 ** length - 1) / 10)
    # 끝까지 밀면 크기 1, 회전도 따로 같은 방식
    assert np.isclose(np.linalg.norm(to_base([1, 0, 0, 0, 0, 0], np.eye(3), 0.0, 0.0, 10.0)[:3]), 1.0)
    rotation = to_base([0, 0, 0, 0.3, 0.0, 0.4], np.eye(3), 0.0, 0.0, 10.0, 10.0)[3:]
    assert np.allclose(rotation / np.linalg.norm(rotation), [0.6, 0, 0.8])


def test_rotation_uses_own_curve():
    # 이동과 회전을 같은 크기(0.5)로 밀어도 회전은 자기 곡선(4)을, 이동은 자기 곡선(10)을 따른다.
    twist = to_base([0.5, 0, 0, 0, 0, 0.5], np.eye(3), 0.0, 0.0, translation_curve=10.0, rotation_curve=4.0)
    assert np.isclose(twist[0], (11 ** 0.5 - 1) / 10)
    assert np.isclose(twist[5], (5 ** 0.5 - 1) / 4)
    assert twist[5] > twist[0]


def test_axis_filter_drops_minor_axes():
    # 밀면서 같이 들어온 기울기(0.2, 0.25)는 최대(0.9)의 30% 이하라 0이 된다.
    values = [0.9, 0.3, 0.0, 0.2, -0.25, 0.1]
    assert np.allclose(keep_dominant_axes(values, 0.3), [0.9, 0.3, 0, 0, 0, 0])
    # 30%를 조금 넘는 대각선 입력은 남긴다.
    assert np.allclose(keep_dominant_axes([0.8, 0.5, 0, 0, 0, 0], 0.3), [0.8, 0.5, 0, 0, 0, 0])
    # 회전이 가장 크면 작은 이동이 잘린다.
    assert np.allclose(keep_dominant_axes([0.1, 0, 0, 0, 0, -0.7], 0.3), [0, 0, 0, 0, 0, -0.7])
    # ratio 0이면 그대로, 입력이 없으면 그대로
    assert np.allclose(keep_dominant_axes(values, 0.0), values)
    assert np.allclose(keep_dominant_axes(np.zeros(6), 0.3), 0)


def pose_at(position, rotvec=(0, 0, 0)):
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    pose[:3, 3] = position
    return pose


def test_reverse_input_drops_wound_up_lead():
    # 목표가 +x로 5cm, +y로 1cm 앞서 쌓인 상태에서 -x로 밀면 x 선행분만 버린다.
    actual = pose_at([0, 0, 0])
    target = pose_at([0.05, 0.01, 0])
    result = drop_opposing_lead(target, actual, [-1, 0, 0, 0, 0, 0])
    assert np.allclose(result[:3, 3], [0, 0.01, 0])
    # 같은 방향이나 수직 입력이면 그대로 둔다.
    assert np.allclose(drop_opposing_lead(target, actual, [1, 0, 0, 0, 0, 0]), target)
    assert np.allclose(drop_opposing_lead(target, actual, [0, 0, 1, 0, 0, 0]), target)


def test_reverse_rotation_drops_wound_up_lead():
    actual = pose_at([0, 0, 0], [0, 0, 0.2])
    target = pose_at([0, 0, 0], [0, 0, 0.5])
    result = drop_opposing_lead(target, actual, [0, 0, 0, 0, 0, -1])
    assert np.allclose(result[:3, :3], actual[:3, :3])


def test_clamp_lead_keeps_direction():
    actual = pose_at([0, 0, 0], [0, 0, 0])
    target = pose_at([0.06, 0.08, 0], [0.6, 0, 0])
    result = clamp_lead(target, actual, 0.05, 0.3)
    assert np.allclose(result[:3, 3], [0.03, 0.04, 0])
    assert np.allclose(Rotation.from_matrix(result[:3, :3]).as_rotvec(), [0.3, 0, 0])
    # 한도 안이면 그대로
    near = pose_at([0.01, 0, 0])
    assert np.allclose(clamp_lead(near, actual, 0.05, 0.3), near)


def unit(deg):
    return np.array([np.cos(np.radians(deg)), np.sin(np.radians(deg))])


def test_calibration_snaps_to_nearest_mouse_axis():
    # 마우스 앞(+Y)에서 20도 비스듬히 밀어도 앞(+Y) 축으로 맞춘다.
    matrix, name, error = x_axis_to_matrix(unit(110))
    assert name == "앞(+Y)" and np.isclose(error, 20.0)
    assert np.allclose(matrix @ [0, 1, 0], [1, 0, 0])
    # +Y는 따로 재지 않고 위에서 봤을 때 반시계 90도: 마우스 왼쪽(-X) = 로봇 왼쪽(+Y)
    assert np.allclose(matrix @ [-1, 0, 0], [0, 1, 0])
    assert np.allclose(matrix @ [0, 0, 1], [0, 0, 1])
    assert np.isclose(np.linalg.det(matrix), 1.0)


def test_calibration_other_axes():
    # 로봇을 마주 보고 마우스를 내 쪽(뒤, -Y)으로 당겨 로봇 앞을 정한 경우
    matrix, name, _ = x_axis_to_matrix(unit(-90))
    assert name == "뒤(-Y)"
    assert np.allclose(matrix @ [0, -1, 0], [1, 0, 0])
    assert np.allclose(matrix @ [1, 0, 0], [0, 1, 0])  # 내 오른쪽 = 로봇 왼쪽
    assert x_axis_to_matrix(unit(5))[1] == "오른쪽(+X)"
    assert x_axis_to_matrix(unit(185))[1] == "왼쪽(-X)"


def test_calibration_rejects_diagonal():
    try:
        x_axis_to_matrix(unit(45 + 10))
    except ValueError:
        return
    raise AssertionError("대각선에 가까운 입력을 받아들였습니다.")


def pair(link1, link2, p1, p2):
    return SimpleNamespace(link1=link1, link2=link2, position1=p1, position2=p2, distance=0.01)


def wrist_controller(q6_deg, remaining_deg):
    """로봇 없이 손목 한도 판정만 보는 컨트롤러. 오른팔 arm_6 = q6, 목표는 실제보다 z축으로 remaining 더 돈 자세."""
    events = EventBus()
    logs = []
    events.subscribe(lambda e: logs.append(e.text) if isinstance(e, LogMessage) else None)
    robot = Rby1Controller(events)
    robot._model = SimpleNamespace(right_arm_idx=list(range(7)), left_arm_idx=list(range(7, 14)))
    robot._q = np.zeros(14)
    robot._q[6] = np.radians(q6_deg)
    robot._wrist_limits = {"right": np.radians((-155.0, 155.0))}
    actual = np.eye(4)
    actual[:3, :3] = Rotation.from_euler("xyz", [170, 10, 40], degrees=True).as_matrix()
    target = actual.copy()
    target[:3, :3] = actual[:3, :3] @ Rotation.from_rotvec([0, 0, np.radians(remaining_deg)]).as_matrix()
    robot._actual = {"right": actual}
    robot._targets = {"right": target}
    return robot, logs


def test_wrist_cap_only_near_limit():
    # 한도와 먼 곳(반올림 오차가 나는 값 포함)에서는 막지 않고 로그도 없다.
    for q6, remaining in ((0.0, 2.0), (16.35, -1.7), (-120.0, 3.3), (149.0, 0.9)):
        robot, logs = wrist_controller(q6, remaining)
        before = robot._targets["right"].copy()
        robot._cap_wrist_target("right")
        assert not robot._wrist_capped["right"] and logs == [], (q6, remaining)
        assert np.array_equal(robot._targets["right"], before)
    # 여유(155-5=150도)를 넘어가는 회전은 150도까지만 남기고 로그를 한 번 남긴다.
    robot, logs = wrist_controller(148.0, 4.0)
    robot._cap_wrist_target("right")
    assert robot._wrist_capped["right"] and len(logs) == 1
    assert np.isclose(np.degrees(robot._wrist_remaining("right")), 2.0)
    # 이미 여유 안(152도)이면 밖으로 밀어내지 않고, 한도 쪽으로만 막는다.
    robot, logs = wrist_controller(152.0, -1.0)
    robot._cap_wrist_target("right")
    assert not robot._wrist_capped["right"]
    robot, logs = wrist_controller(152.0, 1.0)
    robot._cap_wrist_target("right")
    assert robot._wrist_capped["right"] and np.isclose(robot._wrist_remaining("right"), 0.0, atol=1e-9)


def make_pose(position, euler_deg):
    pose = np.eye(4)
    pose[:3, 3] = position
    pose[:3, :3] = Rotation.from_euler("xyz", euler_deg, degrees=True).as_matrix()
    return pose


def test_follow_copies_leader_motion_without_drift():
    # 미러링: 오른팔이 움직인 만큼 왼팔 목표가 대칭으로 움직인다. 왼팔이 못 따라가는 동안 목표는 실제보다
    # 1 cm까지만 앞서지만, 잘린 몫은 버려지지 않아 왼팔이 따라오면 오른팔과 정확히 대칭이 된다.
    robot = Rby1Controller(EventBus())
    leader_start = make_pose([0.4, -0.2, 0.9], [180, 0, 10])
    follower_start = make_pose([0.4, 0.2, 0.9], [180, 0, -10])
    robot._targets = {"right": leader_start.copy(), "left": follower_start.copy()}
    leader = leader_start.copy()
    # 팔로우를 켠 첫 주기에 두 팔의 기준 자세를 잡는다.
    robot._follow_leader("left", {"right": leader, "left": follower_start}, [], "right", mirroring=True)

    for _ in range(10):
        leader = integrate(leader, [0.003, 0.002, 0, 0.01, 0, 0], 1.0, 1.0, 1.0)
        robot._follow_leader("left", {"right": leader, "left": follower_start}, [], "right", mirroring=True)

    lead = np.linalg.norm(robot._targets["left"][:3, 3] - follower_start[:3, 3])
    assert lead <= 0.01 + 1e-9

    for _ in range(10):
        robot._follow_leader("left", {"right": leader, "left": robot._targets["left"].copy()}, [], "right", mirroring=True)

    reflect = np.diag([1.0, -1.0, 1.0])
    expected_position = follower_start[:3, 3] + reflect @ (leader[:3, 3] - leader_start[:3, 3])
    expected_rotation = reflect @ (leader[:3, :3] @ leader_start[:3, :3].T) @ reflect @ follower_start[:3, :3]
    assert np.allclose(robot._targets["left"][:3, 3], expected_position)
    assert np.allclose(robot._targets["left"][:3, :3], expected_rotation)


def test_follow_stops_when_leader_stops():
    # 트래킹: 오른팔 입력이 계속 들어와도 오른팔이 실제로 안 움직이면 왼팔 목표도 그대로다.
    robot = Rby1Controller(EventBus())
    follower_start = make_pose([0.4, 0.2, 0.9], [180, 0, 0])
    leader = make_pose([0.4, -0.2, 0.9], [180, 0, 0])
    robot._targets = {"right": leader.copy(), "left": follower_start.copy()}

    for _ in range(50):
        robot._follow_leader("left", {"right": leader, "left": robot._targets["left"].copy()}, [], "right", mirroring=False)

    assert np.allclose(robot._targets["left"], follower_start)


def test_leader_held_when_follower_cannot_follow():
    # 미러링: 왼팔이 y로 2 cm(대칭이면 오른팔 -2 cm)까지만 따라왔으면 오른팔 목표는 그보다 1 cm 넘게 못 앞선다.
    events = EventBus()
    logs = []
    events.subscribe(lambda e: logs.append(e.text) if isinstance(e, LogMessage) else None)
    robot = Rby1Controller(events)
    leader_start = make_pose([0.4, -0.2, 0.9], [180, 0, 0])
    follower_start = make_pose([0.4, 0.2, 0.9], [180, 0, 0])
    robot._follow_anchor = (leader_start.copy(), follower_start.copy())
    robot._targets = {
        "right": make_pose([0.4, -0.25, 0.9], [180, 0, 0]),
        "left": make_pose([0.4, 0.22, 0.9], [180, 0, 0]),
    }
    robot._hold_by_follower("right", "left", mirroring=True)
    assert np.allclose(robot._targets["right"][:3, 3], [0.4, -0.23, 0.9])
    assert len(logs) == 1
    # 왼팔이 오른팔만큼 따라와 있으면 오른팔 목표는 그대로다.
    robot._targets["left"] = make_pose([0.4, 0.23, 0.9], [180, 0, 0])
    before = robot._targets["right"].copy()
    robot._hold_by_follower("right", "left", mirroring=True)
    assert np.allclose(robot._targets["right"], before) and len(logs) == 1


def test_action_carries_follow_mode():
    teleop, _ = make_teleop()
    assert "follow.mode" in teleop.action_features


def test_wrist_only_only_in_tool_frame():
    teleop, _ = make_teleop()
    teleop.send_feedback({f"{side}_ee.{key}": 0.0 for side in ("right", "left") for key in ("rx", "ry", "rz")})
    z_only = {"right": np.array([0, 0, 0, 0, 0, 0.5]), "left": np.zeros(6)}

    teleop.apply_modes(z_only)
    assert teleop.wrist_only == {"right": False, "left": False}  # 베이스 좌표계에서는 아님

    teleop.frames["right"] = "tool"
    teleop.apply_modes(z_only)
    assert teleop.wrist_only["right"]

    teleop.apply_modes({"right": np.array([0.1, 0, 0, 0, 0, 0.5]), "left": np.zeros(6)})
    assert not teleop.wrist_only["right"]  # 이동이 섞이면 아님
    teleop.apply_modes({"right": np.array([0, 0, 0, 0.2, 0, 0.5]), "left": np.zeros(6)})
    assert not teleop.wrist_only["right"]  # 다른 축 회전이 섞이면 아님

    # 트래킹에서 따르는 팔은 주도 팔의 base 입력을 받으므로 손목만 회전이 아니다.
    teleop.frames["left"] = "tool"
    teleop.arm_mode = "tracking"
    teleop.apply_modes({"right": np.array([0, 0, 0, 0, 0, 0.5]), "left": np.array([0, 0, 0, 0, 0, 0.5])})
    assert teleop.wrist_only == {"right": True, "left": False}


class FakeFlangeRobot:
    """툴 플랜지 전압만 흉내 낸다. 시뮬레이터에는 툴 플랜지가 없다."""

    def __init__(self, voltage):
        self.voltage = {"right": voltage, "left": voltage}
        self.calls = []

    def get_state(self):
        return SimpleNamespace(**{
            f"tool_flange_{side}": SimpleNamespace(output_voltage=v) for side, v in self.voltage.items()
        })

    def set_tool_flange_output_voltage(self, side, voltage):
        self.calls.append((side, voltage))
        self.voltage[side] = voltage
        return True


def test_tool_flange_12v_rules():
    import lerobot_teleoperator_spacemouse4rby1.robot as rc

    original_time = rc.time
    rc.time = SimpleNamespace(sleep=lambda _s: None, monotonic=original_time.monotonic)

    try:
        for start, expected in (
            (12, []),
            (0, [("right", 12), ("left", 12)]),
            (24, [("right", 0), ("right", 12), ("left", 0), ("left", 12)]),
        ):
            controller = rc.Rby1Controller(EventBus())
            controller._robot = FakeFlangeRobot(start)
            controller._ensure_tool_flange_12v()
            assert controller._robot.calls == expected, (start, controller._robot.calls)
            assert controller._robot.voltage == {"right": 12, "left": 12}
    finally:
        rc.time = original_time


def test_arm_of_link():
    assert arm_of_link("link_right_arm_5") == "right"
    assert arm_of_link("ee_finger_l1") == "left"
    assert arm_of_link("link_torso_5") is None


def test_collision_filter_allows_only_moving_away():
    # 오른팔 링크가 몸통(+y쪽)에 가까움: -y 이동은 허용, +y 이동과 회전은 막는다.
    near = [pair("link_torso_5", "link_right_arm_3", [0, 0, 1], [0, -0.1, 1])]
    away = collision_filter("right", [0, -1, 0, 0.5, 0, 0], near)
    toward = collision_filter("right", [0, 1, 0, 0, 0, 0], near)
    assert np.allclose(away, [0, -1, 0, 0, 0, 0])
    assert np.allclose(toward, 0)
    # 걸리지 않은 왼팔은 그대로
    assert np.allclose(collision_filter("left", [0, 1, 0, 1, 0, 0], near), [0, 1, 0, 1, 0, 0])


def test_collision_filter_blocks_same_arm_pair():
    near = [pair("link_right_arm_2", "link_right_arm_6", [0, 0, 0], [0, 0, 0.1])]
    assert np.allclose(collision_filter("right", [1, 1, 1, 1, 1, 1], near), 0)


def test_run_log_writes_one_row_per_cycle():
    import csv
    import tempfile

    from lerobot_teleoperator_spacemouse4rby1.events import StageChanged
    from lerobot_teleoperator_spacemouse4rby1.main import HEADER, RunLog

    teleop, _ = make_teleop()
    teleop.raw_axes["right"] = np.array([0.5, 0, 0, 0, 0, 0.1])
    teleop.raw_fresh["right"] = True
    pose = pose_at([0.4, -0.3, 0.9], [0, 0, 0.2])
    robot = SimpleNamespace(last_cycle={
        "inputs": {"right": np.array([1.0, 0, 0, 0, 0, 0]), "left": np.zeros(6)},
        "targets": {"right": pose, "left": pose},
        "actuals": {"right": pose, "left": pose},
        "arm_q": {"right": np.radians(np.arange(7)), "left": np.zeros(7)},
        "min_distance": 0.09,
        "blocked": False,
    })

    with tempfile.TemporaryDirectory() as directory:
        log = RunLog(directory)
        log.on_event(StageChanged("제어 중"))
        log.write(teleop, robot)
        log.write(teleop, robot)
        log.close()

        with open(log.path, encoding="utf-8") as file:
            rows = list(csv.DictReader(file))

    assert len(rows) == 2 and list(rows[0]) == HEADER
    assert rows[0]["right_raw_x"] == "0.5000" and rows[0]["right_raw_fresh"] == "1"
    assert rows[0]["right_target_rz"] == "0.20000" and rows[0]["right_q6_deg"] == "6.00"
    # 이벤트는 그 다음 한 줄에만 붙는다.
    assert "제어 중" in rows[0]["events"] and rows[1]["events"] == ""


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]

    for test in tests:
        test()
        print(f"OK   {test.__name__}")

    print(f"{len(tests)}개 통과")
