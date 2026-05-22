from abc import ABC, abstractmethod


class DroneInterface(ABC):

    @abstractmethod
    def get_position(self) -> tuple[float, float, float]:
        """Return current position as (x, y, z) in metres relative to takeoff origin."""

    @abstractmethod
    def get_ranges(self) -> dict[str, float]:
        """Return obstacle distances in metres for each direction.

        Keys: 'front', 'back', 'left', 'right', 'up'.
        999.0 indicates no obstacle within sensor range.
        """

    @abstractmethod
    def set_velocity(self, vx: float, vy: float, vz: float, yaw: float) -> None:
        """Command body-frame velocity.

        Args:
            vx: Forward velocity in m/s.
            vy: Lateral velocity in m/s (positive = right).
            vz: Vertical velocity in m/s (positive = up).
            yaw: Yaw rate in rad/s.
        """

    @abstractmethod
    def takeoff(self, height: float = 1.5) -> None:
        """Arm, switch to guided mode, and climb to height metres. Blocks until reached."""

    @abstractmethod
    def land(self) -> None:
        """Initiate landing sequence and block until the vehicle is disarmed."""

    @abstractmethod
    def emergency_stop(self) -> None:
        """Immediately zero velocity and hold position. Safe to call at any time."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Return True if the backend has an active link to the vehicle."""
