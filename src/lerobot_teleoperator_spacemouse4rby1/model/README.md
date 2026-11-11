# 그리퍼 모델

모니터 창의 그리퍼 그림에 쓰는 RB-Y1 M 모델 그리퍼 메시다. 수정하지 않고 복사만 했다.

| 파일 | 원본 |
|---|---|
| `EE_BODY.obj` | rby1-sdk v0.10.1 (`ed6917a`) `models/rby1m/mujoco/assets/EE_BODY/EE_BODY.obj` |
| `EE_FINGER_0.obj`, `EE_FINGER_1.obj` | 같은 위치의 `EE_FINGER/` (손가락 하나를 이루는 두 조각) |

좌표는 URDF의 링크 좌표계 기준 [m]이다.

- 몸통: `ee_right`/`ee_left` 좌표계. 손목 쪽 윗면이 원점이고 그리퍼는 -z 방향으로 뻗는다.
- 손가락: URDF `gripper_finger_r1`/`r2` 관절 기준. 몸통 원점에서 (±0.003, 0, -0.073)에 붙고,
  두 번째 손가락은 z축으로 180도 돌아 있다. 손가락 하나는 x 방향으로 0~0.05 m 벌어진다.
