"""LeRobot teleoperator 설정. 동작 값은 settings.py의 전역 변수를 쓴다."""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("spacemouse4rby1")
@dataclass(kw_only=True)
class SpaceMouse4Rby1Config(TeleoperatorConfig):
    pass
