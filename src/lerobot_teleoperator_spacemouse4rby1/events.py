"""제어 코드와 UI 사이의 이벤트. UI는 이 이벤트만 구독한다."""

import threading
from dataclasses import dataclass


@dataclass(frozen=True)
class StageChanged:
    """진행 단계 안내 (예: 마우스 초기화, 레디자세 이동, 제어 중)."""

    text: str


@dataclass(frozen=True)
class ModeChanged:
    locks: dict  # side -> normal / position_lock / rotation_lock
    frames: dict  # side -> base / tool
    arm_mode: str  # normal / tracking / mirroring
    leader: str  # tracking·mirroring에서 주도하는 팔


@dataclass(frozen=True)
class GripperChanged:
    side: str
    closed: bool


@dataclass(frozen=True)
class CollisionChanged:
    """충돌 경고 상태가 바뀔 때 보낸다."""

    blocked: bool
    distance: float
    link1: str
    link2: str


@dataclass(frozen=True)
class SingularityChanged:
    """팔의 특이점 근접 상태가 바뀔 때 보낸다. sigma는 TCP 자코비안의 최소 특이값이다."""

    side: str
    near: bool
    sigma: float


@dataclass(frozen=True)
class StatusUpdated:
    """제어 주기마다 보내는 상태 요약."""

    min_distance: float
    targets: dict  # side -> base 기준 목표 TCP 자세 4x4
    actuals: dict  # side -> base 기준 실제 TCP 자세 4x4
    inputs: dict  # side -> 모드 적용 후 base 기준 입력 (x, y, z, rx, ry, rz)
    tcp: object  # 그리퍼 몸통(ee) 좌표계에서 본 TCP 자세 4x4. 몸통 자세 = TCP 자세 @ inv(tcp)
    torso: object  # base 기준 link_torso_5 자세 4x4 (모니터 그림의 기준 좌표계)


@dataclass(frozen=True)
class Stopped:
    reason: str


@dataclass(frozen=True)
class LogMessage:
    """실행 로그 한 줄 (툴 플랜지 전원, 손목 한도, 실행 기록 경로 등)."""

    text: str


class EventBus:
    """subscribe한 콜백에 이벤트를 전달한다. 콜백은 emit한 스레드에서 실행된다."""

    def __init__(self):
        self._lock = threading.Lock()
        self._subscribers = []

    def subscribe(self, callback):
        with self._lock:
            self._subscribers.append(callback)

    def unsubscribe(self, callback):
        with self._lock:
            self._subscribers.remove(callback)

    def emit(self, event):
        with self._lock:
            subscribers = list(self._subscribers)

        for callback in subscribers:
            callback(event)
