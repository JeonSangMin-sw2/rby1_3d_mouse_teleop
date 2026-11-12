"""모드와 상태를 보여주는 PySide6 모니터. events.py의 이벤트만 받아 표시한다.

조작·팔 패널(오른팔 위, 왼팔 아래) | 그리퍼 그림(위, 정면을 위아래로) | 실행 로그 3열로 놓고 창 크기는 고정한다.
다른 UI로 바꿀 때는 EventBus.subscribe()로 같은 이벤트를 받고, 키 입력은 on_key로 넘기면 된다.
"""

import signal
import time

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

from .events import (
    CollisionChanged,
    GripperChanged,
    LogMessage,
    ModeChanged,
    StageChanged,
    StatusUpdated,
    Stopped,
)
from . import settings
from .gripper import MAX_OPENING_M, GripperModel

# 그림에서 그리퍼를 열었을 때 손가락 하나가 벌어지는 거리 (max_gripper_open 0~1000 반영)
OPEN_M = MAX_OPENING_M * settings.MAX_GRIPPER_OPEN / 1000.0

MODE_NAMES = {
    "normal": "락 없음",
    "position_lock": "포지션락",
    "rotation_lock": "회전락",
    "base": "베이스",
    "tool": "그리퍼",
}
ARM_MODE_NAMES = {"normal": "일반", "tracking": "트래킹", "mirroring": "미러링"}
# 팔 패널 상태 블럭: 줄마다 ((상태 키, 블럭 글자), ...)와 지금 상태 블럭의 바탕색. 꺼진 블럭은 BLOCK_OFF.
STATE_BLOCKS = {
    "lock": ((("normal", "락 없음"), ("position_lock", "포지션락"), ("rotation_lock", "회전락")), "#00c853"),
    "frame": ((("base", "베이스"), ("tool", "그리퍼")), "#1e88e5"),
}
BLOCK_OFF = "background: #e0e0e0; color: #9e9e9e;"
BLOCK_STYLE = "border-radius: 4px; padding: 2px 6px; font-weight: bold;"
SIDE_NAMES = {"right": "오른팔", "left": "왼팔"}
SIDE_COLORS = {"right": QtGui.QColor("#ff7f0e"), "left": QtGui.QColor("#9467bd")}
AXIS_COLORS = (QtGui.QColor("#d62728"), QtGui.QColor("#2ca02c"), QtGui.QColor("#1f77b4"))
# (분류, ((키, 버튼 글자, 켜짐/꺼짐을 보여주는 팔로우 모드), ...)). 버튼을 누르면 그 키를 누른 것과 같다.
KEY_GROUPS = (
    ("기본", (("r", "레디자세 (r)", None), ("i", "재초기화 (i)", None))),
    ("팔로우", (("m", "미러링 (m)", "mirroring"), ("t", "트래킹 (t)", "tracking"))),
    ("정렬", (("x", "x 정렬 (x)", None), ("y", "y 대칭 (y)", None), ("z", "z 정렬 (z)", None))),
)
# 창 크기 고정 [px]. 글자 길이에 따라 열 너비가 바뀌지 않게 한다.
CONTROLS_WIDTH = 430
SCENE_WIDTH = 420
LOG_WIDTH = 360
WINDOW_SIZE = (1270, 720)
MOUSE_SHORTCUTS = (
    ("마우스 왼쪽 버튼", "그 팔 락 바꾸기"),
    ("마우스 오른쪽 버튼", "그 팔 기준 좌표계 바꾸기"),
    ("두 버튼 동시에", "그 팔 그리퍼 열기 / 닫기"),
)

TCP_AXIS_LENGTH_M = 0.06
TORSO_AXIS_LENGTH_M = 0.12
GRID_M = 0.1
# 그림은 모두 link_torso_5 좌표계에서 그린다.
# (제목, 좌표 → 화면 (가로, 세로) 투영 행렬, 보는 사람 쪽 방향, 가로 범위, 세로 범위 [m])
# 두 그림 모두 로봇을 마주 보고 선 사람 기준이다.
# 위: 화면 위 = 로봇 뒤(-x), 화면 오른쪽 = 로봇 왼쪽(+y)
# 정면: 화면 오른쪽 = 로봇 왼쪽(+y), 화면 위 = 위(+z)
VIEWS = (
    (
        "위, 마주 봄 (↑ 로봇 뒤, → 로봇 왼쪽)",
        np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]),
        np.array([0.0, 0.0, 1.0]),
        (-0.55, 0.55),
        (-0.7, 0.1),
    ),
    (
        "정면, 마주 봄 (→ 로봇 왼쪽, ↑ 위)",
        np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
        np.array([1.0, 0.0, 0.0]),
        (-0.55, 0.55),
        (-0.5, 0.2),
    ),
)
# 위 그림을 z축으로 180° 돌린 것(로봇 뒤에서 본 시점). 화면 위 = 로봇 앞(+x), 화면 오른쪽 = 로봇 오른쪽(-y)
TOP_VIEW_ROTATED = (
    "위, 로봇 뒤에서 봄 (↑ 로봇 앞, → 로봇 오른쪽)",
    -VIEWS[0][1],
    VIEWS[0][2],
    (-0.55, 0.55),
    (-0.1, 0.7),
)
LOG_MAX_LINES = 500  # 실행 로그 창에 남기는 최근 줄 수
BODY_GRAY = 95
FINGER_GRAY = 175


class _Bridge(QtCore.QObject):
    """제어 스레드에서 받은 이벤트를 Qt 메인 스레드로 넘긴다."""

    received = QtCore.Signal(object)


class SceneView(QtWidgets.QWidget):
    """link_torso_5 좌표계 하나에 두 그리퍼를 위·정면에서 그린 그림.

    가운데 굵은 축은 link_torso_5, 그리퍼 모델은 실제 자세, 그리퍼 끝의 굵은 축은 실제 TCP,
    팔 색 원과 점선 축은 목표 TCP다. 옅은 격자는 10 cm 간격이다.
    """

    def __init__(self, model):
        super().__init__()
        self.model = model
        self.setMinimumHeight(320)
        self.actuals = None
        self.targets = None
        self.tcp = np.eye(4)
        self.openings = {side: OPEN_M for side in SIDE_NAMES}
        self.top_rotated = False

    def set_top_rotated(self, rotated):
        self.top_rotated = rotated
        self.update()

    def set_state(self, actuals, targets, tcp, torso):
        # base 기준 자세를 link_torso_5 기준으로 바꿔 둔다.
        to_torso = np.linalg.inv(torso)
        self.actuals = {side: to_torso @ pose for side, pose in actuals.items()}
        self.targets = {side: to_torso @ pose for side, pose in targets.items()}
        self.tcp = tcp
        self.update()

    def set_closed(self, side, closed):
        self.openings[side] = 0.0 if closed else OPEN_M
        self.update()

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        views = (TOP_VIEW_ROTATED if self.top_rotated else VIEWS[0], *VIEWS[1:])
        height = self.height() / len(views)

        for index, view in enumerate(views):
            self._draw_view(painter, QtCore.QRectF(0, index * height, self.width(), height), *view)

    def _draw_view(self, painter, rect, title, projection, toward_viewer, u_range, v_range):
        area = rect.adjusted(4, 20, -4, -4)
        painter.setPen(self.palette().color(QtGui.QPalette.WindowText))
        painter.drawText(rect.adjusted(6, 2, 0, 0), QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop, title)
        painter.setPen(self.palette().color(QtGui.QPalette.Mid))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawRect(area)

        scale = min(area.width() / (u_range[1] - u_range[0]), area.height() / (v_range[1] - v_range[0]))
        center = (np.mean(u_range), np.mean(v_range))
        origin_x = area.center().x() - center[0] * scale
        origin_y = area.center().y() + center[1] * scale

        def to_screen(point):
            u, v = projection @ point
            return QtCore.QPointF(origin_x + u * scale, origin_y - v * scale)

        painter.save()
        painter.setClipRect(area)
        self._draw_grid(painter, area, center, scale, origin_x, origin_y)
        self._draw_axes(painter, np.eye(4), to_screen, TORSO_AXIS_LENGTH_M, QtCore.Qt.SolidLine, 3)
        painter.setPen(self.palette().color(QtGui.QPalette.WindowText))
        painter.drawText(to_screen(np.zeros(3)) + QtCore.QPointF(6, 16), "torso_5")

        if self.actuals is not None:
            self._draw_models(painter, projection, toward_viewer, scale, origin_x, origin_y)

            for side in SIDE_NAMES:
                target, actual = self.targets[side], self.actuals[side]
                self._draw_axes(painter, target, to_screen, TCP_AXIS_LENGTH_M, QtCore.Qt.DashLine, 1)
                painter.setPen(QtGui.QPen(SIDE_COLORS[side], 2))
                painter.setBrush(QtCore.Qt.NoBrush)
                painter.drawEllipse(to_screen(target[:3, 3]), 6, 6)
                self._draw_axes(painter, actual, to_screen, TCP_AXIS_LENGTH_M, QtCore.Qt.SolidLine, 3)
                painter.setPen(SIDE_COLORS[side])
                painter.drawText(to_screen(actual[:3, 3]) + QtCore.QPointF(10, -10), SIDE_NAMES[side])

        painter.restore()

    def _draw_grid(self, painter, area, center, scale, origin_x, origin_y):
        painter.setPen(QtGui.QPen(self.palette().color(QtGui.QPalette.Midlight), 1))
        half_u = area.width() / scale / 2
        half_v = area.height() / scale / 2

        for u in np.arange(np.ceil((center[0] - half_u) / GRID_M), np.floor((center[0] + half_u) / GRID_M) + 1) * GRID_M:
            painter.drawLine(QtCore.QLineF(origin_x + u * scale, area.top(), origin_x + u * scale, area.bottom()))

        for v in np.arange(np.ceil((center[1] - half_v) / GRID_M), np.floor((center[1] + half_v) / GRID_M) + 1) * GRID_M:
            painter.drawLine(QtCore.QLineF(area.left(), origin_y - v * scale, area.right(), origin_y - v * scale))

    def _draw_models(self, painter, projection, toward_viewer, scale, origin_x, origin_y):
        """두 그리퍼의 삼각형을 한꺼번에 먼 것부터 그려, 서로 가리는 것도 맞게 보이게 한다."""
        triangles = []
        grays = []
        tcp_inverse = np.linalg.inv(self.tcp)

        for side in SIDE_NAMES:
            body, fingers = self.model.triangles(self.actuals[side] @ tcp_inverse, self.openings[side])
            triangles += [body, fingers]
            grays += [np.full(len(body), BODY_GRAY), np.full(len(fingers), FINGER_GRAY)]

        triangles = np.concatenate(triangles)
        gray = np.concatenate(grays)

        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        lengths[lengths == 0.0] = 1.0
        # 단순화한 메시는 면 방향(감는 순서)이 섞여 있어 뒷면을 버리면 구멍이 난다. 모두 그리고 밝기만 절댓값으로 낸다.
        shade = 0.35 + 0.65 * np.abs(normals @ toward_viewer) / lengths
        levels = np.clip(gray * shade, 0, 255).astype(int)

        order = np.argsort(triangles.mean(axis=1) @ toward_viewer)
        screen = triangles[order] @ projection.T * [scale, -scale] + [origin_x, origin_y]
        painter.setPen(QtCore.Qt.NoPen)
        Point = QtCore.QPointF

        for points, level in zip(screen.tolist(), levels[order].tolist()):
            painter.setBrush(QtGui.QColor(level, level, level))
            painter.drawPolygon([Point(*points[0]), Point(*points[1]), Point(*points[2])])

    def _draw_axes(self, painter, pose, to_screen, length, style, width):
        origin = to_screen(pose[:3, 3])

        for axis, color in zip(pose[:3, :3].T, AXIS_COLORS):
            painter.setPen(QtGui.QPen(color, width, style))
            painter.drawLine(origin, to_screen(pose[:3, 3] + axis * length))


class StateBlocks(QtWidgets.QWidget):
    """고를 수 있는 상태를 글자 크기만 한 블럭으로 나란히 놓고, 지금 상태인 블럭만 color로 칠한다."""

    def __init__(self, states, color):
        super().__init__()
        self.on = f"background: {color}; color: white;"
        self.blocks = {}
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        for key, text in states:
            block = QtWidgets.QLabel(text)
            block.setStyleSheet(BLOCK_STYLE + BLOCK_OFF)
            layout.addWidget(block)
            self.blocks[key] = block

        layout.addStretch(1)

    def set_current(self, current):
        for key, block in self.blocks.items():
            block.setStyleSheet(BLOCK_STYLE + (self.on if key == current else BLOCK_OFF))


class ArmPanel(QtWidgets.QGroupBox):
    def __init__(self, side):
        super().__init__(f"{SIDE_NAMES[side]} (메인)" if side == settings.MAIN_ARM else SIDE_NAMES[side])
        self.blocks = {name: StateBlocks(states, color) for name, (states, color) in STATE_BLOCKS.items()}
        self.labels = {name: QtWidgets.QLabel("-") for name in ("gripper", "target", "actual", "input")}
        titles = {
            "lock": "락 (왼쪽 버튼)",
            "frame": "기준 좌표계 (오른쪽 버튼)",
            "gripper": "그리퍼 (두 버튼)",
            "target": "목표 TCP 위치 (base)",
            "actual": "실제 TCP 위치 (base)",
            "input": "입력",
        }
        fixed = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        form = QtWidgets.QFormLayout(self)

        for name, blocks in self.blocks.items():
            form.addRow(titles[name], blocks)

        for name, label in self.labels.items():
            if name != "gripper":
                label.setFont(fixed)

            form.addRow(titles[name], label)


class MonitorWindow(QtWidgets.QWidget):
    def __init__(self, on_key=None):
        super().__init__()
        self.setWindowTitle("RB-Y1 SpaceMouse Teleop")
        self.on_key = on_key

        # 진행 단계(StageChanged)를 보여 주고, 충돌 경고 중에는 경고를 대신 보여 준다.
        self._stage = "-"
        self._collision = None
        self.status_label = QtWidgets.QLabel("-")
        self.status_label.setWordWrap(True)
        # 두 줄 높이로 고정해 긴 중단 사유가 들어와도 아래 칸이 밀리지 않게 한다.
        self.status_label.setFixedHeight(2 * self.status_label.fontMetrics().lineSpacing() + 4)
        status = QtWidgets.QFormLayout()
        status.addRow("<b>상태</b>", self.status_label)

        shortcuts = QtWidgets.QGroupBox("조작 (키보드는 이 창에서 바로, 또는 터미널에 입력 후 Enter)")
        shortcut_layout = QtWidgets.QVBoxLayout(shortcuts)
        buttons = QtWidgets.QGridLayout()
        self.mode_buttons = {}

        for row, (group, keys) in enumerate(KEY_GROUPS):
            buttons.addWidget(QtWidgets.QLabel(f"<b>{group}</b>"), row, 0)

            for column, (key, text, mode) in enumerate(keys, start=1):
                button = QtWidgets.QPushButton(text)
                # 버튼이 포커스를 가져가면 스페이스바로 눌리고 창의 키 입력이 버튼으로 간다.
                button.setFocusPolicy(QtCore.Qt.NoFocus)
                button.clicked.connect(lambda _checked=False, key=key: self._press(key))
                buttons.addWidget(button, row, column)

                if mode is not None:
                    button.setCheckable(True)
                    self.mode_buttons[mode] = button

        shortcut_layout.addLayout(buttons)
        mouse_form = QtWidgets.QFormLayout()

        for key, description in MOUSE_SHORTCUTS:
            mouse_form.addRow(f"<b>{key}</b>", QtWidgets.QLabel(description))

        shortcut_layout.addLayout(mouse_form)

        self.panels = {side: ArmPanel(side) for side in SIDE_NAMES}
        controls_widget = QtWidgets.QWidget()
        controls_widget.setFixedWidth(CONTROLS_WIDTH)
        controls = QtWidgets.QVBoxLayout(controls_widget)
        controls.setContentsMargins(0, 0, 0, 0)
        controls.addLayout(status)
        controls.addWidget(shortcuts)
        controls.addWidget(self.panels["right"])
        controls.addWidget(self.panels["left"])
        controls.addStretch(1)

        self.scene = SceneView(GripperModel())
        self.scene.setFixedWidth(SCENE_WIDTH)
        rotate = QtWidgets.QPushButton("위 그림 180° 회전")
        rotate.setCheckable(True)
        rotate.setFocusPolicy(QtCore.Qt.NoFocus)
        rotate.toggled.connect(self.scene.set_top_rotated)
        scene_column = QtWidgets.QVBoxLayout()
        scene_column.addWidget(rotate, alignment=QtCore.Qt.AlignRight)
        scene_column.addWidget(self.scene, stretch=1)

        log_box = QtWidgets.QGroupBox("실행 로그")
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(LOG_MAX_LINES)
        self.log_view.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        # 로그 창이 포커스를 가져가면 단축키가 창으로 가지 않는다.
        self.log_view.setFocusPolicy(QtCore.Qt.NoFocus)
        QtWidgets.QVBoxLayout(log_box).addWidget(self.log_view)
        log_box.setFixedWidth(LOG_WIDTH)

        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(controls_widget)
        layout.addLayout(scene_column)
        layout.addWidget(log_box)
        self.setFixedSize(*WINDOW_SIZE)

    def add_log(self, text):
        self.log_view.appendPlainText(f"{time.strftime('%H:%M:%S')}  {text}")

    def keyPressEvent(self, event):
        if self.on_key is not None and event.text():
            self.on_key(event.text())
        else:
            super().keyPressEvent(event)

    def _press(self, key):
        if self.on_key is not None:
            self.on_key(key)

    def handle(self, event):
        line = log_line(event)

        if line is not None:
            self.add_log(line)

        if isinstance(event, StageChanged):
            self._stage = event.text
            self._show_status_line()
        elif isinstance(event, ModeChanged):
            self._show_modes(event)
        elif isinstance(event, GripperChanged):
            self.panels[event.side].labels["gripper"].setText("닫힘" if event.closed else "열림")
            self.scene.set_closed(event.side, event.closed)
        elif isinstance(event, CollisionChanged):
            self._collision = (
                f"충돌 경고: {event.link1} - {event.link2} {event.distance * 1000:.1f} mm, 멀어지는 이동만 허용"
                if event.blocked else None
            )
            self._show_status_line()
        elif isinstance(event, StatusUpdated):
            self._show_status(event)
        elif isinstance(event, Stopped):
            self._stage = f"중단: {event.reason}"
            self._show_status_line()

    def _show_status_line(self):
        # 중단 > 충돌 경고 > 진행 단계 순으로 하나만 보여 준다.
        stopped = self._stage.startswith("중단")
        text = self._collision if self._collision is not None and not stopped else self._stage

        self.status_label.setText(text)
        self.status_label.setStyleSheet("color: red;" if text is not self._stage or stopped else "")

    def _show_modes(self, event):
        # 버튼은 누를 때 스스로 눌림 상태가 바뀌므로, 실제 모드로 다시 맞춘다.
        for arm_mode, button in self.mode_buttons.items():
            button.setChecked(event.arm_mode == arm_mode)

        for side, panel in self.panels.items():
            panel.blocks["lock"].set_current(event.locks[side])
            panel.blocks["frame"].set_current(event.frames[side])

    def _show_status(self, event):
        for side, panel in self.panels.items():
            panel.labels["target"].setText(f"{np.round(event.targets[side][:3, 3], 3)} m")
            panel.labels["actual"].setText(f"{np.round(event.actuals[side][:3, 3], 3)} m")
            panel.labels["input"].setText(" ".join(f"{v:+.2f}" for v in event.inputs[side]))

        self.scene.set_state(event.actuals, event.targets, event.tcp, event.torso)


def log_line(event):
    """실행 로그에 남길 한 줄. 주기적인 상태(StatusUpdated)는 남기지 않는다."""
    if isinstance(event, LogMessage):
        return event.text

    if isinstance(event, StageChanged):
        return f"상태: {event.text}"

    if isinstance(event, ModeChanged):
        arms = ", ".join(
            f"{SIDE_NAMES[side]} {MODE_NAMES[event.locks[side]]}·{MODE_NAMES[event.frames[side]]}"
            for side in ("left", "right")
        )
        return f"모드: {arms}, 팔로우 {ARM_MODE_NAMES[event.arm_mode]}"

    if isinstance(event, GripperChanged):
        return f"{SIDE_NAMES[event.side]} 그리퍼 {'닫힘' if event.closed else '열림'}"

    if isinstance(event, CollisionChanged):
        if event.blocked:
            return f"충돌 경고: {event.link1} - {event.link2} {event.distance * 1000:.1f} mm"

        return "충돌 경고 해제"

    if isinstance(event, Stopped):
        return f"정지: {event.reason}"

    return None


def run(events, on_close, on_key=None, startup_lines=()):
    """창을 띄우고 닫힐 때까지 기다린다. 닫히면 on_close()를 호출한다.

    창의 키 입력은 on_key(text)로 넘기고, startup_lines는 창이 뜨기 전 내용을 실행 로그 첫 줄로 넣는다.
    """
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MonitorWindow(on_key)

    for line in startup_lines:
        window.add_log(line)

    bridge = _Bridge()
    bridge.received.connect(window.handle)
    forward = bridge.received.emit
    events.subscribe(forward)

    # Qt 루프 중에도 Ctrl+C로 창을 닫을 수 있게 한다.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(200)

    window.show()

    try:
        app.exec()
    finally:
        # 창이 사라진 뒤 제어 스레드가 보내는 마지막 이벤트(Stopped)가 지워진 Qt 객체로 가지 않게 한다.
        events.unsubscribe(forward)
        on_close()
