# LeRobot Teleoperator SpaceMouse4Rby1

두 대의 3Dconnexion SpaceMouse로 Rainbow Robotics RB-Y1의
양팔과 그리퍼를 조작하는 LeRobot teleoperator 플러그인입니다.

## 주요 기능

- SpaceMouse 두 대로 양팔을 각각 조작
- SpaceMouse 버튼으로 그리퍼 열기와 닫기
- TCP를 기준으로 한 위치 및 자세 제어
- LeRobot `lerobot-teleoperate` 명령 지원

## 요구사항

- Linux
- Python 가상환경
- LeRobot
- [rby1-lerobot](https://github.com/RainbowRobotics/rby1-lerobot)
- RB-Y1 Python SDK
- SpaceMouse 두 대
- HIDAPI

아래 설치 방법은 LeRobot과 `rby1-lerobot`이 같은 Python 가상환경에
설치되어 있다고 가정합니다.

## 설치

가상환경을 활성화한 뒤 HIDAPI를 설치합니다.

```bash
# Ubuntu / Debian
sudo apt update
sudo apt install libhidapi-dev
```

저장소를 clone하고 설치합니다.

```bash
git clone https://github.com/ssongzoe/lerobot-teleoperator-spacemouse4rby1.git

cd lerobot-teleoperator-spacemouse4rby1

pip install -e .
```

설치 확인:

```bash
lerobot-teleoperate --help
```

`--teleop.type` 목록에 `spacemouse4rby1`,
`--robot.type` 목록에 `rby1`이 있는지 확인합니다.

## SpaceMouse 경로 확인

두 SpaceMouse를 연결하고 다음 명령을 실행합니다.

```bash
python check_mouse_path.py
```

안내에 따라 왼쪽 마우스와 오른쪽 마우스를 차례로 움직입니다.
각각 입력이 감지된 경로를 확인한 뒤 아래 실행 명령의
`--teleop.left_device`, `--teleop.right_device`에 넣습니다.

무선 수신기 하나가 여러 `/dev/hidraw*` 인터페이스로 표시될 수
있습니다. 목록에 보이는 경로가 각각 별도의 마우스라는 뜻은 아닙니다.

`/dev/hidraw*` 번호는 재연결이나 재부팅 후 바뀔 수 있으므로
실행 전에 다시 확인하세요.

장치를 열 수 없다면 HIDAPI 설치와 `hidraw` 장치의 udev 권한을
확인하세요.

## 퀵스타트

아래 장치 경로는 예시입니다. 먼저 `check_mouse_path.py`로 확인한
왼쪽과 오른쪽 경로로 바꾸세요.

```bash
lerobot-teleoperate \
  --robot.type=rby1 \
  --robot.address=192.168.30.1:50051 \
  --robot.action_mode=ee \
  --robot.use_gripper=true \
  --robot.move_to_ready_on_connect=true \
  --teleop.type=spacemouse4rby1 \
  --teleop.robot_address=192.168.30.1:50051 \
  --teleop.left_device=/dev/hidraw6 \
  --teleop.right_device=/dev/hidraw11 \
  --teleop.use_gripper=true \
  --teleop.translation_speed_m_s=0.03 \
  --teleop.rotation_speed_rad_s=0.15 \
  --fps=30 \
  --teleop_time_s=600
```

- `--teleop_time_s=600`: 10분 동안 실행합니다.
- 이 옵션을 생략하면 시간 제한 없이 실행합니다.
- `Ctrl+C`로 종료할 수 있습니다.
- 로봇과 teleoperator의 주소 및 `use_gripper` 설정은 일치해야 합니다.

시작 시 로봇이 ready pose로 이동하며, 그리퍼 초기화가 실행될 수
있습니다. 시작 전 로봇 주변 공간과 정지 방법을 확인하세요.

## 조작

기본 좌표 매핑은 `HID_Z_UP` 기준입니다.

| SpaceMouse 방향 | 로봇 base 방향 |
|---|---|
| 앞쪽 | +X |
| 오른쪽 | -Y |
| 위쪽 | +Z |

회전 입력에도 같은 좌표 변환이 적용됩니다. 마우스 배치와 로봇을
바라보는 방향에 따라 실제 조작 방향을 확인하세요.

현재 테스트한 무선 SpaceMouse의 버튼 매핑은 다음과 같습니다.

| 버튼 | 인덱스 | 동작 |
|---|---:|---|
| 왼쪽 | 0 | 그리퍼 닫기 |
| 오른쪽 | 14 | 그리퍼 열기 |

버튼 인덱스는 SpaceMouse 모델에 따라 다를 수 있습니다.

## TCP 설정

기본 TCP 오프셋은 `arm_6` 로컬 좌표계 기준입니다.

```python
tcp_offset_m = (0.0, 0.0, -0.2011)
```

RB-Y1 기본 그리퍼 양쪽 손가락 링크 원점 사이의 중간점에
해당합니다.

TCP를 기준으로 이동과 회전을 계산하고, 로봇에는 변환된
`arm_6` 목표 자세를 보냅니다.

## 참고사항

- 손을 놓은 뒤 로봇의 정지 반응은 목표 추종과 컨트롤러 설정에 따라
  달라질 수 있습니다.
- 무선 마우스와 수신기의 연결이 끊겨도 `/dev/hidraw*` 경로는
  남아 있을 수 있습니다. 경로 존재만으로 무선 연결 상태를
  확인할 수는 없습니다.
- 실제 로봇 사용 전 장치 방향, 속도, 그리퍼 동작 및 정지 방법을
  충분히 확인하세요.

## 참고 프로젝트

- [LeRobot](https://github.com/huggingface/lerobot)
- [rby1-lerobot](https://github.com/RainbowRobotics/rby1-lerobot)
- [rby1-sdk](https://github.com/RainbowRobotics/rby1-sdk)
- [PySpaceMouse](https://spacemouse.kubaandrysek.cz/)
