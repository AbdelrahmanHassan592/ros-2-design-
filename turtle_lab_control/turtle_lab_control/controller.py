"""Pure, replaceable velocity strategy, independent of ROS transport."""
from enum import Enum, auto
import math
from .common import POSITION_TOLERANCE, YAW_TOLERANCE, normalize

class Phase(Enum):
    """The three phases of pose control."""
    ROTATE = auto()
    DRIVE = auto()
    FINAL_YAW = auto()

class ProportionalController:
    """Strategy for rotating, driving and aligning the final heading."""
    LINEAR_GAIN = 1.5
    ANGULAR_GAIN = 4.0
    ALIGN_TOLERANCE = 0.03

    def __init__(self) -> None:
        self.phase = Phase.ROTATE

    def velocity(self, x: float, y: float, yaw: float, tx: float,
                 ty: float, target_yaw: float) -> tuple[float, float, float, float, bool]:
        """Return linear/angular commands, errors and completion state."""
        distance = math.hypot(tx - x, ty - y)
        final_error = normalize(target_yaw - yaw)
        heading = normalize(math.atan2(ty - y, tx - x) - yaw)
        if distance <= POSITION_TOLERANCE:
            self.phase = Phase.FINAL_YAW
        if self.phase == Phase.ROTATE and abs(heading) <= self.ALIGN_TOLERANCE:
            self.phase = Phase.DRIVE
        if self.phase == Phase.FINAL_YAW:
            return (0.0, self.ANGULAR_GAIN * final_error, distance,
                    abs(final_error), abs(final_error) <= YAW_TOLERANCE)
        linear = (self.LINEAR_GAIN * distance * max(0.0, math.cos(heading))
                  if self.phase == Phase.DRIVE else 0.0)
        return linear, self.ANGULAR_GAIN * heading, distance, abs(heading), False
