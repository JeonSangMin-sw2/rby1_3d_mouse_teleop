"""두 팔 손목·그리퍼 사이 거리를 SDK 충돌 모델보다 정밀하게 잰다.

SDK 충돌 모델은 손목부터 그리퍼 끝까지를 arm_5 링크 캡슐 하나(반지름 7.5 cm)로 덮는다. 두 그리퍼가 서로를 향하면
캡슐끼리 먼저 닿아, 실제로 3 cm가 남았는데도 충돌 경고가 떴다(시뮬레이터: 캡슐 10.3 mm, 메시 29 mm).
손목은 그 캡슐에서 그리퍼 장착면(ee 원점) 위쪽 부분만 쓰고, 그리퍼는 model/의 메시를 쓴다.
둘 다 겉면을 점으로 바꿔 두 팔의 점 사이 최소 거리를 구한다.
"""

import numpy as np
from scipy.spatial import cKDTree

from .gripper import FINGER_MOUNT_X, FINGER_MOUNT_Z, MODEL_DIR, load_obj

# SDK가 쓰는 arm_5 충돌 캡슐 (arm_5 좌표, z축 방향). rby1-sdk models/rby1m/urdf/model_v1.2.urdf의 값이고,
# 이 값으로 계산한 캡슐 거리가 SDK detect_collisions_or_nearest_links 거리와 일치했다(시뮬레이터, 무작위 자세 6개).
CAPSULE_RADIUS_M = 0.075
CAPSULE_TOP_Z_M = -0.08565 + 0.2213 / 2
CAPSULE_BOTTOM_Z_M = -0.08565 - 0.2213 / 2
SAMPLE_SPACING_M = 0.004  # 겉면 점 간격(3 mm는 가까울 때 제어 주기 10 ms를 넘겼다). 점 사이 거리는 실제보다 최대 이만큼 멀게 나오므로 결과에서 뺀다.


def sample_triangles(vertices, faces, spacing):
    """삼각형 겉면을 면적에 비례해 점으로 바꾼다(꼭짓점 포함). 같은 입력이면 같은 점이 나온다."""
    rng = np.random.default_rng(0)
    triangles = vertices[faces]
    areas = 0.5 * np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1)
    counts = np.ceil(areas / spacing ** 2).astype(int)
    owner = np.repeat(np.arange(len(triangles)), counts)
    u, v = rng.random((2, len(owner)))
    flip = u + v > 1.0
    u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
    t = triangles[owner]
    inside = t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])
    return np.vstack([vertices, inside])


def sample_wrist(mount_z, spacing):
    """SDK arm_5 캡슐 중 그리퍼 장착면(z = mount_z) 위쪽의 겉면 점 (arm_5 좌표). 원통 옆면, 위 반구, 아래 원판."""
    r = CAPSULE_RADIUS_M
    points = []
    angles = np.linspace(0.0, 2.0 * np.pi, int(np.ceil(2.0 * np.pi * r / spacing)), endpoint=False)

    for z in np.arange(mount_z, CAPSULE_TOP_Z_M + spacing, spacing):
        points.append(np.column_stack([r * np.cos(angles), r * np.sin(angles), np.full(len(angles), min(z, CAPSULE_TOP_Z_M))]))

    for polar in np.arange(spacing / r, np.pi / 2.0, spacing / r):
        ring = np.linspace(0.0, 2.0 * np.pi, max(int(np.ceil(2.0 * np.pi * r * np.sin(polar) / spacing)), 1), endpoint=False)
        points.append(np.column_stack([
            r * np.sin(polar) * np.cos(ring), r * np.sin(polar) * np.sin(ring), np.full(len(ring), CAPSULE_TOP_Z_M + r * np.cos(polar)),
        ]))

    points.append([[0.0, 0.0, CAPSULE_TOP_Z_M + r]])

    for radius in np.arange(0.0, r, spacing):
        ring = np.linspace(0.0, 2.0 * np.pi, max(int(np.ceil(2.0 * np.pi * radius / spacing)), 1), endpoint=False)
        points.append(np.column_stack([radius * np.cos(ring), radius * np.sin(ring), np.full(len(ring), mount_z)]))

    return np.vstack(points)


def _transform(pose, points):
    return points @ pose[:3, :3].T + pose[:3, 3]


class ToolCollision:
    """mount_z: 그리퍼 장착면(ee 원점)의 arm_5 좌표 z. ee z축은 arm_5 z축과 같고 원점은 그 축 위에 있다."""

    def __init__(self, mount_z, model_dir=MODEL_DIR, spacing=SAMPLE_SPACING_M):
        self.spacing = spacing
        self.wrist = sample_wrist(mount_z, spacing)
        self.body = sample_triangles(*load_obj(model_dir / "EE_BODY.obj"), spacing)
        fingers = [load_obj(model_dir / name) for name in ("EE_FINGER_0.obj", "EE_FINGER_1.obj")]
        self.finger = np.vstack([sample_triangles(vertices, faces, spacing) for vertices, faces in fingers])
        self.mount_z = mount_z
        self._protrusion = {}

    def gripper_points(self, ee_pose, opening):
        """그리퍼 몸통·손가락 겉면 점 (base 좌표). opening: 손가락 하나가 벌어진 거리 [m]."""
        first = self.finger + [FINGER_MOUNT_X + opening, 0.0, FINGER_MOUNT_Z]
        second = self.finger * [-1.0, -1.0, 1.0] + [-FINGER_MOUNT_X - opening, 0.0, FINGER_MOUNT_Z]
        return _transform(ee_pose, np.vstack([self.body, first, second]))

    def protrusion(self, opening):
        """그리퍼가 SDK 캡슐 밖으로 나오는 최대 거리 [m] (0 이상). 벌린 정도마다 한 번만 계산해 둔다."""
        if opening not in self._protrusion:
            self._protrusion[opening] = self._compute_protrusion(opening)

        return self._protrusion[opening]

    def _compute_protrusion(self, opening):
        mount = np.eye(4)
        mount[2, 3] = self.mount_z
        points = self.gripper_points(mount, opening)
        axis_z = np.clip(points[:, 2], CAPSULE_BOTTOM_Z_M, CAPSULE_TOP_Z_M)
        outside = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2 + (points[:, 2] - axis_z) ** 2) - CAPSULE_RADIUS_M
        return max(float(outside.max()), 0.0)

    def distance(self, arm5_poses, ee_poses, openings, within, sides=("right", "left")):
        """두 팔 손목·그리퍼 사이 최소 거리와 그때의 가장 가까운 점(팔마다, base 좌표).

        within [m]보다 먼 거리는 필요 없으므로, 상대 팔 캡슐에서 그보다 먼 점은 미리 버려 계산을 줄인다
        (모두 쓰면 한 번에 약 30 ms로 제어 주기보다 길었다). 점 간격만큼 빼서 실제보다 가깝게 낸다.
        반환: (거리 [m], {side: 점}). within 안에 없으면 (inf, None).
        """
        clouds, axes = {}, {}

        for side in sides:
            clouds[side] = np.vstack([
                _transform(arm5_poses[side], self.wrist), self.gripper_points(ee_poses[side], openings[side]),
            ])
            top = _transform(arm5_poses[side], np.array([[0.0, 0.0, CAPSULE_TOP_Z_M]]))[0]
            bottom = _transform(arm5_poses[side], np.array([[0.0, 0.0, CAPSULE_BOTTOM_Z_M]]))[0]
            axes[side] = (top, bottom)

        first, second = sides
        kept = {}

        for side, other in ((first, second), (second, first)):
            # 상대 팔 점은 모두 상대 캡슐 축에서 반지름 + 그리퍼가 캡슐 밖으로 나오는 만큼 안에 있다.
            reach = CAPSULE_RADIUS_M + self.protrusion(openings[other]) + within + self.spacing
            kept[side] = clouds[side][_segment_distance(clouds[side], *axes[other]) < reach]

            if len(kept[side]) == 0:
                return np.inf, None

        distances, nearest = cKDTree(kept[second]).query(kept[first])
        best = int(np.argmin(distances))
        points = {first: kept[first][best], second: kept[second][nearest[best]]}
        return float(distances[best]) - self.spacing, points


def _segment_distance(points, start, end):
    """점들에서 선분(start-end)까지의 거리."""
    direction = end - start
    t = np.clip((points - start) @ direction / (direction @ direction), 0.0, 1.0)
    return np.linalg.norm(points - (start + t[:, None] * direction), axis=1)
