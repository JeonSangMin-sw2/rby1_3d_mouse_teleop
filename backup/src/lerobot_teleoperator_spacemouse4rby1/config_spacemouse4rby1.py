"""Configuration for dual SpaceMouse teleoperation of RB-Y1."""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("spacemouse4rby1")
@dataclass(kw_only=True)
class SpaceMouse4Rby1Config(TeleoperatorConfig):
    # SpaceMouse 장치 경로: 실행할 때 반드시 지정
    left_device: str
    right_device: str = ""

    # True면 왼쪽 마우스 한 대로 테스트
    single_device: bool = False

    # 초기 pose 조회에 사용할 로봇 주소
    robot_address: str = "127.0.0.1:50051"   #"192.168.30.1:50051"
    robot_model: str = "auto"

    # 입력 처리
    deadzone: float = 0.08
    input_timeout_s: float = 0.08

    # 마우스를 최대한 밀었을 때의 목표 이동 속도
    translation_speed_m_s: float = 0.03
    rotation_speed_rad_s: float = 0.15

    # 그리퍼: 0=닫힘, 1=열림
    use_gripper: bool = True
    gripper_open_button: int = 14
    gripper_close_button: int = 0

    # 실제 pose와 목표 pose 사이의 최대 허용 차이
    max_target_lead_m: float = 0.05
    max_target_rotation_rad: float = 0.35

    # arm_6 로컬 좌표계에서 TCP까지의 오프셋 [m], 기본 그리퍼 기준 계산값임
    # URDF의 양쪽 손가락 링크 원점 사이 중간점
    tcp_offset_m: tuple[float, float, float] = (0.0, 0.0, -0.2011)

    # 초당 정규화 그리퍼 위치 변화량
    gripper_speed: float = 0.5

    def __post_init__(self):
        self.left_device = self.left_device.strip()
        self.right_device = self.right_device.strip()

        if not self.left_device:
            raise ValueError("left_device must be provided.")

        if not self.single_device and not self.right_device:
            raise ValueError(
                "right_device must be provided in dual-device mode."
            )

        if self.right_device and self.left_device == self.right_device:
            raise ValueError(
                "Left and right must use different SpaceMouse devices."
            )

        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1).")

        for name in (
            "input_timeout_s",
            "translation_speed_m_s",
            "rotation_speed_rad_s",
            "gripper_speed",
        ):
            if not 0.0 < getattr(self, name) < float("inf"):
                raise ValueError(f"{name} must be finite and positive.")

        if self.gripper_open_button < 0 or self.gripper_close_button < 0:
            raise ValueError("Gripper button indices must be non-negative.")

        if self.gripper_open_button == self.gripper_close_button:
            raise ValueError(
                "Gripper open and close buttons must be different."
            )