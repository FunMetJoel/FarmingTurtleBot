from enum import Enum

class RobotState(Enum):
    """Enum for robot states shared between files."""
    IDLE = "idle"
    MOVING = "moving"
    IRRIGATING = "irrigating"
    FILLING = "filling"
    ERROR = "error"

class SystemMode(Enum):
    """Enum for state of the DT system."""
    DISCOVER = 0
    MAP = 1
    IDLE = 2
    SIMULATING = 3
    IRRIGATING = 4
    FILLING = 5
    ERROR = 6