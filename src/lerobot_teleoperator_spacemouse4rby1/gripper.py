"""RB-Y1 그리퍼 제어(Gripper)와 모니터 창에 그릴 그리퍼 메시(GripperModel).

Gripper는 SDK 35번 예제와 같은 방식으로 /dev/rby1_gripper를 직접 쓴다.
UPC에서만 열리고, connect()의 homing에서 손가락이 양 끝까지 움직인다.
GripperModel은 model/ 폴더의 OBJ를 읽어 그리퍼 자세대로 삼각형을 만든다.
"""

import threading
import time
from pathlib import Path

import numpy as np
import rby1_sdk as rby
from rby1_sdk import upc

from . import settings

GRIPPER_IDS = {"right": 0, "left": 1}
BAUD_RATE = 2_000_000
HOMING_TORQUE = 0.3
HOLD_TORQUE = 5.0
LOOP_PERIOD_S = 0.1


class Gripper:
    def __init__(self):
        self._bus = None
        self._ids = list(GRIPPER_IDS.values())
        self._min_q = np.full(len(self._ids), np.inf)
        self._max_q = np.full(len(self._ids), -np.inf)
        self._target_q = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def connect(self):
        if not 0 <= settings.MAX_GRIPPER_OPEN <= 1000:
            raise ValueError(f"max_gripper_open은 0~1000이어야 합니다: {settings.MAX_GRIPPER_OPEN!r}")

        self._bus = rby.DynamixelBus(upc.GripperDeviceName)

        if not self._bus.open_port():
            raise RuntimeError(f"그리퍼 포트를 열지 못했습니다: {upc.GripperDeviceName}")

        upc.initialize_device(upc.GripperDeviceName)

        if not self._bus.set_baud_rate(BAUD_RATE):
            raise RuntimeError("그리퍼 baud rate를 설정하지 못했습니다.")

        self._bus.set_torque_constant([1.0] * len(self._ids))

        for side, dev_id in GRIPPER_IDS.items():
            if not self._bus.ping(dev_id):
                raise RuntimeError(f"{side} 그리퍼(Dynamixel ID {dev_id})가 응답하지 않습니다.")

        self._bus.group_sync_write_torque_enable([(i, 1) for i in self._ids])
        self._home()

        self._set_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self._bus.group_sync_write_send_torque([(i, HOLD_TORQUE) for i in self._ids])

        # 처음에는 열린 상태를 목표로 둔다.
        self.set_closed({side: False for side in GRIPPER_IDS})

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _set_mode(self, mode):
        self._bus.group_sync_write_torque_enable([(i, 0) for i in self._ids])
        self._bus.group_sync_write_operating_mode([(i, mode) for i in self._ids])
        self._bus.group_sync_write_torque_enable([(i, 1) for i in self._ids])

    def _home(self):
        """양 방향으로 밀어 엔코더가 30번 연속 멈추면 그 끝을 이동 범위로 삼는다."""
        self._set_mode(rby.DynamixelBus.CurrentControlMode)
        q = np.zeros(len(self._ids))

        for direction in (1.0, -1.0):
            previous = None
            still = 0

            while still < 30:
                self._bus.group_sync_write_send_torque(
                    [(i, HOMING_TORQUE * direction) for i in self._ids]
                )
                read = self._bus.group_fast_sync_read_encoder(self._ids)

                if read is not None:
                    for dev_id, encoder in read:
                        q[self._ids.index(dev_id)] = encoder

                self._min_q = np.minimum(self._min_q, q)
                self._max_q = np.maximum(self._max_q, q)
                still = still + 1 if previous is not None and np.array_equal(previous, q) else 0
                previous = q.copy()
                time.sleep(0.1)

        self._bus.group_sync_write_send_torque([(i, 0.0) for i in self._ids])

        if not np.all(self._max_q > self._min_q):
            raise RuntimeError("그리퍼 homing에서 이동 범위를 얻지 못했습니다.")

    def set_closed(self, closed):
        """closed: side -> bool. True면 닫는다."""
        # 35번 예제(GRIPPER_DIRECTION=False)와 같이 닫힘이 min 엔코더, 열림이 max 엔코더다.
        # 열 때는 max_gripper_open(0~1000)만큼만 벌린다.
        wide = settings.MAX_GRIPPER_OPEN / 1000.0
        opening = np.array([0.0 if closed[side] else wide for side in GRIPPER_IDS])

        with self._lock:
            self._target_q = opening * (self._max_q - self._min_q) + self._min_q

    def _loop(self):
        while self._running:
            with self._lock:
                target = self._target_q

            self._bus.group_sync_write_send_position(
                [(dev_id, q) for dev_id, q in zip(self._ids, target.tolist())]
            )
            time.sleep(LOOP_PERIOD_S)

    def disconnect(self):
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self._bus is not None:
            try:
                self._bus.group_sync_write_torque_enable([(i, 0) for i in self._ids])
            finally:
                self._bus.close_port()
                self._bus = None


MODEL_DIR = Path(__file__).with_name("model")
# URDF gripper_finger_r1/r2 관절 위치 (그리퍼 몸통 좌표계 기준)
FINGER_MOUNT_X = 0.003
FINGER_MOUNT_Z = -0.073
MAX_OPENING_M = 0.05
# 그리는 속도를 위해 이 크기 격자로 꼭짓점을 합쳐 삼각형 수를 줄인다.
SIMPLIFY_CELL_M = 0.004


def load_obj(path):
    """OBJ에서 꼭짓점과 삼각형(다각형은 부채꼴로 나눔)을 읽는다."""
    vertices = []
    faces = []

    with open(path, encoding="utf-8") as file:
        for line in file:
            if line.startswith("v "):
                vertices.append([float(value) for value in line.split()[1:4]])
            elif line.startswith("f "):
                indices = [int(item.split("/")[0]) - 1 for item in line.split()[1:]]
                faces.extend([indices[0], indices[i], indices[i + 1]] for i in range(1, len(indices) - 1))

    return np.array(vertices), np.array(faces)


def simplify(vertices, faces, cell):
    """격자 칸마다 꼭짓점을 하나로 합치고, 찌그러지거나 겹친 삼각형을 버린다."""
    keys, cluster = np.unique(np.floor(vertices / cell).astype(int), axis=0, return_inverse=True)
    cluster = cluster.ravel()
    merged = np.zeros((len(keys), 3))
    np.add.at(merged, cluster, vertices)
    merged /= np.bincount(cluster)[:, None]

    faces = cluster[faces]
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    faces = np.unique(np.sort(faces, axis=1), axis=0)
    return merged, faces


def _transform(pose, vertices):
    return vertices @ pose[:3, :3].T + pose[:3, 3]


class GripperModel:
    def __init__(self, model_dir=MODEL_DIR):
        body = simplify(*load_obj(model_dir / "EE_BODY.obj"), SIMPLIFY_CELL_M)
        finger_parts = [load_obj(model_dir / name) for name in ("EE_FINGER_0.obj", "EE_FINGER_1.obj")]
        offset = len(finger_parts[0][0])
        finger = simplify(
            np.vstack([finger_parts[0][0], finger_parts[1][0]]),
            np.vstack([finger_parts[0][1], finger_parts[1][1] + offset]),
            SIMPLIFY_CELL_M,
        )
        self.body = body[0][body[1]]  # (삼각형 수, 3, 3)
        self.finger = finger[0][finger[1]]

    def triangles(self, body_pose, opening):
        """body_pose: base 기준 그리퍼 몸통(ee) 자세 4x4, opening: 손가락 하나가 벌어진 거리 [m].

        (몸통 삼각형, 손가락 삼각형)을 base 좌표로 반환한다. 각 배열은 (삼각형 수, 3, 3)이다.
        """
        first = np.eye(4)
        first[:3, 3] = [FINGER_MOUNT_X + opening, 0.0, FINGER_MOUNT_Z]
        second = np.diag([-1.0, -1.0, 1.0, 1.0])  # z축 180도
        second[:3, 3] = [-FINGER_MOUNT_X - opening, 0.0, FINGER_MOUNT_Z]

        body = _transform(body_pose, self.body.reshape(-1, 3)).reshape(self.body.shape)
        fingers = np.concatenate([
            _transform(body_pose @ mount, self.finger.reshape(-1, 3)).reshape(self.finger.shape)
            for mount in (first, second)
        ])
        return body, fingers
