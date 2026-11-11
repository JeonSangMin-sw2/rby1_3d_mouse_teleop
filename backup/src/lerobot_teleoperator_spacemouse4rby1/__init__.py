from .config_spacemouse4rby1 import SpaceMouse4Rby1Config
from .spacemouse4rby1 import (
    SpaceMouse4Rby1,
    install_lerobot_bridge,
)

install_lerobot_bridge()

__all__ = [
    "SpaceMouse4Rby1Config",
    "SpaceMouse4Rby1",
]