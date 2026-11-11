"""시뮬레이터에서 Rby1Controller를 마우스 대신 합성 입력으로 확인한다.

시뮬레이터 주소(127.0.0.1:50051)에서만 실행된다. 로봇이 레디자세로 움직인다.

    python tests/sim_controller_check.py collision   # 몸통·왼팔 쪽으로 밀어 충돌 차단과 빠져나오기 확인
    python tests/sim_controller_check.py limits      # 앞·바깥·위·회전 한계까지 밀어 특이점 fault 여부 확인
    python tests/sim_controller_check.py response    # 밀기 시작·반대로 밀기의 반응 지연과 따라오는 지연
    python tests/sim_controller_check.py reach       # 몸 안쪽(+y)으로 밀었을 때 이동 거리와 어깨 roll(arm_1)
    python tests/sim_controller_check.py tcp         # 회전만 줬을 때 TCP는 제자리에 있고 몸통이 도는지
    python tests/sim_controller_check.py toolz       # 그리퍼 좌표계 z축 회전에서 관절별로 얼마나 움직이는지
    python tests/sim_controller_check.py wristswitch # 손목만 회전 ↔ 이동을 오갈 때 튀는지
    python tests/sim_controller_check.py release     # 밀다가 놓았을 때 놓은 뒤로 더 움직이는 거리
    python tests/sim_controller_check.py ready       # 움직인 뒤 레디자세로 돌아가고 다시 조작되는지 (r 키)
    python tests/sim_controller_check.py align       # 왼팔 x·y(대칭)·z를 오른팔에 맞추는지, 왼팔 마우스 입력으로 취소되는지 (x, y, z 키)
    python tests/sim_controller_check.py follow      # 팔로우에서 오른팔이 못 가면 왼팔도 멈추는지 (m, t 키)
    python tests/sim_controller_check.py reinit      # 팔을 끝까지 뻗어 제어가 멈추게 한 뒤 재초기화로 다시 조작되는지 (i 키)

--variant로 설정 하나만 바꿔 같은 입력으로 비교한다.
"""

import argparse
import time

import numpy as np
from scipy.spatial.transform import Rotation

from lerobot_teleoperator_spacemouse4rby1 import settings
from lerobot_teleoperator_spacemouse4rby1.events import CollisionChanged, EventBus, LogMessage
from lerobot_teleoperator_spacemouse4rby1.mouse import mirror, tool_to_base
from lerobot_teleoperator_spacemouse4rby1.robot import Rby1Controller, RobotStopped

KEYS = ("x", "y", "z", "rx", "ry", "rz")


def action(right=(0,) * 6, wrist_only=False, left=(0,) * 6, follow=0):
    """팔별 입력으로 만든 행동. wrist_only는 텔레옵의 '그리퍼 좌표계 z축 회전만' 표시다(오른팔).

    follow는 팔로우 모드 번호(0 일반, 1 트래킹, 2 미러링)이고 메인 팔은 settings.MAIN_ARM이다.
    """
    result = {f"right_ee.{k}": float(v) for k, v in zip(KEYS, right)}
    result.update({f"left_ee.{k}": float(v) for k, v in zip(KEYS, left)})
    result.update({"right_ee.wrist_only": float(wrist_only), "left_ee.wrist_only": 0.0})
    result.update({"right_gripper.closed": 0.0, "left_gripper.closed": 0.0})
    result["follow.mode"] = float(follow)
    return result


def run(robot, twist, seconds):
    """twist를 seconds 동안 보내고 오른팔 EE 위치, 최소 거리, 팔꿈치 최대 각도를 반환한다."""
    elbow = robot._model.right_arm_idx[3]
    elbow_max = -np.inf
    end = time.perf_counter() + seconds

    while time.perf_counter() < end:
        robot.send_action(action(twist))
        elbow_max = max(elbow_max, robot._robot.get_state().position[elbow])
        time.sleep(settings.CONTROL_PERIOD_S)

    poses, pairs = robot._read_poses()
    return poses["right"][:3, 3], pairs[0], np.rad2deg(elbow_max)


def time_to_move(robot, twist, seconds, distance=0.005):
    """twist를 보내며 오른팔이 twist 이동 방향으로 distance만큼 갈 때까지 걸린 시간과 목표-실제 지연을 잰다.

    방향 없이 거리만 보면, 반대로 밀 때 관성으로 앞으로 더 밀려난 거리까지 움직인 것으로 센다.
    """
    direction = np.asarray(twist[:3], dtype=float)
    direction /= np.linalg.norm(direction)
    start = robot._read_poses()[0]["right"][:3, 3].copy()
    began = time.perf_counter()
    reached = None
    lags = []
    durations = []

    while time.perf_counter() - began < seconds:
        sent = time.perf_counter()
        robot.send_action(action(twist))
        durations.append(time.perf_counter() - sent)
        actual = robot._read_poses()[0]["right"][:3, 3]
        lags.append(np.linalg.norm(robot._targets["right"][:3, 3] - actual))

        if reached is None and np.dot(actual - start, direction) > distance:
            reached = time.perf_counter() - began

        time.sleep(settings.CONTROL_PERIOD_S)

    return reached, float(np.mean(lags[-20:])), durations


def check_collision(robot, collisions):
    start, _, _ = run(robot, (0,) * 6, 0.5)
    position, near, _ = run(robot, (0, 1, 0, 0, 0, 0), 12.0)
    print(f"왼쪽(+y)으로 12초: 이동 {np.round(position - start, 3)}, 최소 거리 {near.distance:.4f} ({near.link1}-{near.link2})")

    held, near, _ = run(robot, (0, 1, 0, 0, 0, 0), 2.0)
    print(f"차단 중 계속 +y 2초: 이동 {np.round(held - position, 4)}, 최소 거리 {near.distance:.4f}")

    away, near, _ = run(robot, (0, -1, 0, 0, 0, 0), 2.0)
    print(f"반대(-y) 2초: 이동 {np.round(away - held, 4)}, 최소 거리 {near.distance:.4f}")
    print("충돌 이벤트:", [(e.blocked, round(e.distance, 4)) for e in collisions])


def check_limits(robot):
    phases = [
        ("앞 +x", (1, 0, 0, 0, 0, 0), 8),
        ("바깥 -y", (0, -1, 0, 0, 0, 0), 6),
        ("위 +z", (0, 0, 1, 0, 0, 0), 6),
        ("회전 rx", (0, 0, 0, 1, 0, 0), 4),
        ("회전 ry", (0, 0, 0, 0, 1, 0), 4),
        ("회전 rz", (0, 0, 0, 0, 0, 1), 4),
        ("앞+위+회전", (0.7, 0, 0.7, 0.5, 0.5, 0), 6),
    ]
    worst = -np.inf

    for name, twist, seconds in phases:
        _, _, elbow_max = run(robot, twist, seconds)
        worst = max(worst, elbow_max)
        print(f"{name:10s}: 팔꿈치 최대 {elbow_max:6.1f}°")

    limit = np.rad2deg(settings.ELBOW_LIMIT_RAD[1])
    print(f"fault 없음. 팔꿈치 최대 {worst:.1f}° (한도 {limit:.1f}°)")


def check_response(robot):
    run(robot, (0,) * 6, 0.5)
    ideal = 0.005 / settings.LINEAR_VELOCITY
    started, lag, forward = time_to_move(robot, (1, 0, 0, 0, 0, 0), 1.5)
    reversed_, _, backward = time_to_move(robot, (-1, 0, 0, 0, 0, 0), 2.0)
    durations = np.array(forward + backward) * 1000
    print(
        f"5 mm 움직이기까지: 시작 {started} s, 반대로 밀기 {reversed_} s "
        f"(지연 없는 이상값 {ideal:.2f} s), 목표-실제 지연 {lag * 1000:.1f} mm"
    )
    print(
        f"send_action 한 번: 중앙값 {np.median(durations):.1f} ms, 최대 {durations.max():.1f} ms "
        f"(제어 주기 {settings.CONTROL_PERIOD_S * 1000:.0f} ms, {len(durations)}회)"
    )


def check_reach(robot):
    shoulder = robot._model.right_arm_idx[1]
    start, _, _ = run(robot, (0,) * 6, 0.5)
    position, near, _ = run(robot, (0, 1, 0, 0, 0, 0), 8.0)
    roll = np.rad2deg(robot._robot.get_state().position[shoulder])
    print(f"안쪽(+y) 8초: y 이동 {position[1] - start[1]:.3f} m, arm_1 {roll:.1f}° (한도 +1°), 최소 거리 {near.distance:.4f}")


def check_tcp(robot):
    """회전 입력만 주면 TCP(기준점)는 거의 그대로이고 그리퍼 몸통 원점만 돌아야 한다."""
    run(robot, (0,) * 6, 0.5)
    tcp_start = robot._read_poses()[0]["right"]
    body_start = tcp_start @ robot._tcp_inverse
    run(robot, (0, 0, 0, 0, 0, 1), 2.0)
    run(robot, (0,) * 6, 1.0)
    tcp_end = robot._read_poses()[0]["right"]
    body_end = tcp_end @ robot._tcp_inverse
    angle = np.degrees(np.arccos(np.clip((np.trace(tcp_start[:3, :3].T @ tcp_end[:3, :3]) - 1) / 2, -1, 1)))
    print(
        f"z축 회전 2초: 회전 {angle:.1f}°, TCP 이동 {np.linalg.norm(tcp_end[:3, 3] - tcp_start[:3, 3]) * 1000:.1f} mm, "
        f"몸통 원점 이동 {np.linalg.norm(body_end[:3, 3] - body_start[:3, 3]) * 1000:.1f} mm "
        f"(TCP 오프셋 {np.round(settings.TCP_OFFSET_M, 3)} m)"
    )


def check_tool_z(robot):
    """그리퍼 좌표계 z축 회전만 +, - 방향으로 2초씩 줬을 때 관절별로 얼마나 움직이는지.

    이상적으로는 arm_6만 움직이고, 손목 한도에 닿으면 그 방향으로는 멈춘다.
    """
    idx = robot._model.right_arm_idx

    for direction in (1, -1):
        run(robot, (0,) * 6, 0.5)
        start = np.rad2deg(np.asarray(robot._robot.get_state().position)[idx])
        peak = np.zeros(7)
        end_time = time.perf_counter() + 2.0

        while time.perf_counter() < end_time:
            # 텔레옵과 같이 목표 그리퍼 방향으로 그리퍼 z축 회전을 base 기준으로 바꾼다.
            rotation = robot._targets["right"][:3, :3]
            robot.send_action(action(tool_to_base([0, 0, 0, 0, 0, direction], rotation), wrist_only=True))
            now = np.rad2deg(np.asarray(robot._robot.get_state().position)[idx])
            peak = np.maximum(peak, np.abs(now - start))
            time.sleep(settings.CONTROL_PERIOD_S)

        run(robot, (0,) * 6, 1.0)
        end = np.rad2deg(np.asarray(robot._robot.get_state().position)[idx])
        print(f"[{'+' if direction > 0 else '-'} 방향] 시작 arm_6 {start[6]:.1f}°")
        print("  관절별 최대 변화 [deg] (arm_0 ~ arm_6):", np.round(peak, 1))
        print("  끝난 뒤 변화    [deg] (arm_0 ~ arm_6):", np.round(end - start, 1))


def check_wrist_switch(robot):
    """손목만 회전(arm_6 보조 목표) ↔ 이동을 오갈 때 한 주기 사이에 튀는지 본다."""
    idx = robot._model.right_arm_idx
    run(robot, (0,) * 6, 0.5)
    phases = [("손목 회전", True), ("앞으로 이동", False), ("손목 회전", True), ("멈춤", None)]

    for name, wrist in phases:
        last_q = np.asarray(robot._robot.get_state().position)[idx]
        last_p = robot._read_poses()[0]["right"][:3, 3].copy()
        joint_step = 0.0
        tcp_step = 0.0
        end_time = time.perf_counter() + 1.0

        while time.perf_counter() < end_time:
            # 레디자세 손목이 + 방향 한도 근처(150°)라 - 방향으로 돌린다.
            if wrist is True:
                twist = tool_to_base([0, 0, 0, 0, 0, -1], robot._targets["right"][:3, :3])
            elif wrist is False:
                twist = (1, 0, 0, 0, 0, 0)
            else:
                twist = (0,) * 6

            robot.send_action(action(twist, wrist_only=wrist is True))
            q = np.asarray(robot._robot.get_state().position)[idx]
            p = robot._read_poses()[0]["right"][:3, 3]
            joint_step = max(joint_step, np.degrees(np.abs(q - last_q)).max())
            tcp_step = max(tcp_step, np.linalg.norm(p - last_p) * 1000)
            last_q, last_p = q, p.copy()
            time.sleep(settings.CONTROL_PERIOD_S)

        print(f"{name:8s}: 한 주기 최대 관절 변화 {joint_step:.2f}°, 최대 TCP 이동 {tcp_step:.2f} mm, 손목만 회전 상태 {robot._wrist_only['right']}")


def check_release(robot):
    """마우스를 밀다가 놓았을 때 놓은 뒤로 더 움직인 거리. 손을 떼면 바로 멈춰야 한다."""
    run(robot, (0,) * 6, 0.5)

    for name, twist in (("앞 +x", (1, 0, 0, 0, 0, 0)), ("위 +z", (0, 0, 1, 0, 0, 0))):
        run(robot, twist, 1.0)
        released = robot._read_poses()[0]["right"][:3, 3].copy()
        lead = np.linalg.norm(robot._targets["right"][:3, 3] - released) * 1000
        settled, _, _ = run(robot, (0,) * 6, 1.0)
        print(f"{name}: 놓는 순간 목표가 앞서 있던 거리 {lead:.1f} mm → 놓은 뒤 더 움직인 거리 {np.linalg.norm(settled - released) * 1000:.1f} mm")


def check_ready(robot):
    ready = robot._read_poses()[0]["right"][:3, 3].copy()
    moved, _, _ = run(robot, (1, 0, 1, 0, 0, 0), 2.0)
    robot.return_to_ready()
    back = robot._read_poses()[0]["right"][:3, 3].copy()
    again, _, _ = run(robot, (0, 0, 1, 0, 0, 0), 1.0)
    print(
        f"움직인 거리 {np.linalg.norm(moved - ready) * 1000:.1f} mm → 레디자세 복귀 후 차이 {np.linalg.norm(back - ready) * 1000:.1f} mm "
        f"→ 다시 위(+z)로 1초 {(again[2] - back[2]) * 1000:+.1f} mm"
    )


def check_align(robot, logs):
    """오른팔을 옮긴 뒤 x·y·z 키(align_axis)로 왼팔을 맞춘다(y는 좌우 대칭). 끝나고 왼팔 마우스 입력으로 취소도 확인한다."""
    run(robot, (0.6, -0.4, 0.7, 0, 0, 0), 1.5)

    for axis in "xyz":
        index = "xyz".index(axis)
        before = robot._read_poses()[0]
        robot.align_axis("right", axis)
        start = time.perf_counter()

        while robot._align_goal["left"] is not None and time.perf_counter() - start < 8.0:
            robot.send_action(action())
            time.sleep(settings.CONTROL_PERIOD_S)

        took = time.perf_counter() - start
        run(robot, (0,) * 6, 0.5)
        after = robot._read_poses()[0]
        sign = -1.0 if axis == "y" else 1.0
        others = [i for i in range(3) if i != index]
        angle = np.degrees(np.linalg.norm(
            Rotation.from_matrix(before["left"][:3, :3].T @ after["left"][:3, :3]).as_rotvec()
        ))
        print(
            f"{axis}: 맞추기 전 차이 {(sign * before['right'][index, 3] - before['left'][index, 3]) * 1000:+.1f} mm → "
            f"{took:.1f}초 뒤 {(sign * after['right'][index, 3] - after['left'][index, 3]) * 1000:+.1f} mm, "
            f"왼팔 다른 축 변화 {np.linalg.norm(after['left'][others, 3] - before['left'][others, 3]) * 1000:.1f} mm, "
            f"회전 변화 {angle:.2f}°"
        )

    run(robot, (0, 0, -1, 0, 0, 0), 1.0)
    robot.align_axis("right", "z")
    run(robot, (0,) * 6, 0.3)
    robot.send_action(action(left=(1, 0, 0, 0, 0, 0)))
    print(f"취소 확인: 왼팔 입력 뒤 정렬 중 = {robot._align_goal['left'] is not None}")
    run(robot, (0,) * 6, 0.5)

    for text in logs:
        print(f"  로그: {text}")


def check_follow(robot):
    """팔로우에서 메인 팔(오른팔)이 못 가는 상황(도달 한계·충돌 가드)에 왼팔도 같이 멈추는지.

    텔레옵처럼 왼팔 입력에는 오른팔 입력(미러링이면 대칭)을 넣는다. 왼팔 움직임이 오른팔 실제 움직임(미러링이면 대칭)에서
    얼마나 어긋나는지를 움직이는 동안(최대)과 손을 떼고 1.5초 뒤(최종)로 본다. 최대에는 따르는 팔의 반응 지연이 들어간다.
    """
    cases = (
        ("트래킹, 앞 +x 5초", 1, (1, 0, 0, 0, 0, 0), 5.0),
        ("미러링, 몸 안쪽 +y 6초 (충돌 가드에 막힘)", 2, (0, 1, 0, 0, 0, 0), 6.0),
        ("미러링, 위 +z·회전 ry 3초", 2, (0, 0, 0.7, 0, 0.7, 0), 3.0),
    )

    def mismatch(start, mode):
        poses = robot._actual
        flip = np.ones(3) if mode == 1 else np.array([1.0, -1.0, 1.0])
        leader = poses["right"][:3, 3] - start["right"][:3, 3]
        follower = poses["left"][:3, 3] - start["left"][:3, 3]
        leader_turn = Rotation.from_matrix(poses["right"][:3, :3] @ start["right"][:3, :3].T).as_rotvec()
        follower_turn = Rotation.from_matrix(poses["left"][:3, :3] @ start["left"][:3, :3].T).as_rotvec()
        expected_turn = leader_turn if mode == 1 else mirror(np.r_[0, 0, 0, leader_turn])[3:]
        return (
            np.linalg.norm(leader) * 1000,
            np.linalg.norm(follower - flip * leader) * 1000,
            np.degrees(np.linalg.norm(follower_turn - expected_turn)),
        )

    for name, mode, twist, seconds in cases:
        robot.return_to_ready()
        start = robot._read_poses()[0]
        follower_twist = twist if mode == 1 else mirror(twist)
        worst_mm = worst_deg = 0.0
        stopped = None
        begin = time.perf_counter()

        try:
            while time.perf_counter() - begin < seconds + 1.5:
                pushing = time.perf_counter() - begin < seconds
                robot.send_action(action(
                    twist if pushing else (0,) * 6, left=follower_twist if pushing else (0,) * 6, follow=mode
                ))
                moved, mm, deg = mismatch(start, mode)
                worst_mm, worst_deg = max(worst_mm, mm), max(worst_deg, deg)
                time.sleep(settings.CONTROL_PERIOD_S)
        except RobotStopped as exc:
            stopped = exc

        commanded = np.linalg.norm(twist[:3]) * settings.LINEAR_VELOCITY * seconds * 1000
        print(
            f"{name}: 오른팔 이동 {moved:.0f} mm (입력대로면 {commanded:.0f} mm), "
            f"왼팔 어긋남 최대 {worst_mm:.1f} mm / {worst_deg:.2f}°, 최종 {mm:.1f} mm / {deg:.2f}°"
        )

        if stopped is not None:
            print(f"  멈춤: {stopped} → 재초기화")
            robot.reinitialize()


def check_reinit(robot):
    """앞·위로 끝까지 밀어 로봇이 제어를 멈추게 한 뒤, reinitialize()로 레디자세에 돌아와 다시 움직이는지 본다."""
    ready = robot._read_poses()[0]["right"][:3, 3].copy()

    try:
        run(robot, (1, 0, 0.5, 0, 0, 0), 15.0)
        print("15초 동안 제어가 멈추지 않았습니다. (재초기화는 확인하지 못함)")
        return
    except RobotStopped as exc:
        print(f"멈춤: {exc}")

    robot.reinitialize()
    state = robot._robot.get_control_manager_state()
    back = robot._read_poses()[0]["right"][:3, 3].copy()
    again, _, _ = run(robot, (0, 0, 1, 0, 0, 0), 1.0)
    print(
        f"재초기화 뒤 control manager {state.state.name}, unlimited {state.unlimited_mode_enabled}, "
        f"레디자세와 차이 {np.linalg.norm(back - ready) * 1000:.1f} mm → 다시 위(+z)로 1초 {(again[2] - back[2]) * 1000:+.1f} mm"
    )


def apply_variant(variant):
    if variant == "accel":
        settings.LINEAR_ACCELERATION, settings.ANGULAR_ACCELERATION = 150.0, 80 * np.pi
    elif variant == "stiff":
        settings.ARM_STIFFNESS = [400.0] * 7
        settings.ARM_TORQUE_LIMIT = [60.0] * 7
    elif variant == "position":
        settings.IMPEDANCE_ENABLED = False
    elif variant == "noelbow":
        settings.ELBOW_LIMIT_RAD = [-2.6, 2.6]
    elif variant == "lead5":
        settings.MAX_TARGET_LEAD_M = 0.05
    elif variant == "ready5":
        pose = {name: list(values) for name, values in settings.READY_POSE_DEG.items()}
        pose["right_arm"][1], pose["left_arm"][1] = -5.0, 5.0
        settings.READY_POSE_DEG = pose


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "scenario",
        choices=("collision", "limits", "response", "reach", "tcp", "ready", "toolz", "wristswitch", "release", "align", "follow", "reinit"),
    )
    parser.add_argument(
        "--variant",
        choices=("base", "accel", "stiff", "position", "noelbow", "ready5", "lead5"),
        default="base",
        help="accel: 가속도 공식값, stiff: 강성 400, position: 위치 제어, noelbow: 팔꿈치 한도 끔, "
        "ready5: 레디자세 어깨 roll을 ∓5°로, lead5: 목표 선행 한도 5 cm",
    )
    parser.add_argument(
        "--impedance",
        choices=("on", "off"),
        help="yaml의 impedance_enabled 대신 임피던스 제어를 켜거나 끈다. (noelbow, stiff, accel은 임피던스에서만 의미 있음)",
    )
    args = parser.parse_args()

    if args.impedance is not None:
        settings.IMPEDANCE_ENABLED = args.impedance == "on"

    if settings.ROBOT_ADDRESS not in ("127.0.0.1:50051", "localhost:50051"):
        raise SystemExit(f"시뮬레이터 주소에서만 실행합니다. 현재 robot_address={settings.ROBOT_ADDRESS}")

    apply_variant(args.variant)

    events = EventBus()
    collisions = []
    events.subscribe(lambda e: collisions.append(e) if isinstance(e, CollisionChanged) else None)
    logs = []
    events.subscribe(lambda e: logs.append(e.text) if isinstance(e, LogMessage) else None)

    robot = Rby1Controller(events)
    robot.connect()

    control = "임피던스" if settings.IMPEDANCE_ENABLED else "위치 제어"
    print(f"[{args.scenario} / {args.variant} / {control}, 직선 속도 {settings.LINEAR_VELOCITY} m/s]")

    try:
        {
            "collision": lambda: check_collision(robot, collisions),
            "limits": lambda: check_limits(robot),
            "response": lambda: check_response(robot),
            "reach": lambda: check_reach(robot),
            "tcp": lambda: check_tcp(robot),
            "toolz": lambda: check_tool_z(robot),
            "wristswitch": lambda: check_wrist_switch(robot),
            "release": lambda: check_release(robot),
            "ready": lambda: check_ready(robot),
            "align": lambda: check_align(robot, logs),
            "follow": lambda: check_follow(robot),
            "reinit": lambda: check_reinit(robot),
        }[args.scenario]()
    except RuntimeError as exc:
        print(f"중단: {exc}")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
