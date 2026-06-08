from enum import Enum

class RobotState(Enum):
    """Enum for robot states shared between files."""
    IDLE = "idle"
    MOVING = "moving"
    IRRIGATING = "irrigating"
    FILLING = "filling"
    ERROR = "error"