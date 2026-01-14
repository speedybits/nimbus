"""
Base behavior class for Nimbus.

Behaviors are modular components that determine what the robot
should do at any given moment. They receive sensor data and
output velocity commands.

Behaviors can be:
- Built-in (idle, wander, goto, patrol)
- User-defined plugins
"""

from abc import ABC, abstractmethod
from typing import Optional
from nimbus.core.state import RobotContext, Velocity, SensorSnapshot


class Behavior(ABC):
    """
    Abstract base class for robot behaviors.

    Subclasses implement the `compute` method to determine
    velocity commands based on sensor data.

    Usage:
        class MyBehavior(Behavior):
            name = "my_behavior"
            description = "Does something cool"

            def compute(self, context: RobotContext) -> Optional[Velocity]:
                # Return velocity or None to pass to lower priority
                return Velocity(0.1, 0.0)
    """

    # Override these in subclasses
    name: str = "unnamed"
    description: str = "No description"
    priority: int = 0  # Higher = more priority

    def __init__(self):
        self._active = False

    @abstractmethod
    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """
        Compute velocity command based on current context.

        Args:
            context: Current robot state and sensor data

        Returns:
            Velocity command, or None to delegate to lower-priority behavior
        """
        pass

    def activate(self) -> None:
        """Called when this behavior becomes active."""
        self._active = True

    def deactivate(self) -> None:
        """Called when this behavior is deactivated."""
        self._active = False

    @property
    def is_active(self) -> bool:
        """Check if behavior is currently active."""
        return self._active

    def reset(self) -> None:
        """Reset behavior state (called on reactivation)."""
        pass

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name='{self.name}', active={self._active})"


class BehaviorManager:
    """
    Manages multiple behaviors with priority-based arbitration.

    Behaviors are executed in priority order. The first behavior
    that returns a non-None velocity wins.
    """

    def __init__(self):
        self._behaviors: dict[str, Behavior] = {}
        self._active_behavior: Optional[str] = None

    def register(self, behavior: Behavior) -> None:
        """Register a behavior."""
        self._behaviors[behavior.name] = behavior

    def unregister(self, name: str) -> None:
        """Unregister a behavior by name."""
        if name in self._behaviors:
            behavior = self._behaviors[name]
            if behavior.is_active:
                behavior.deactivate()
            del self._behaviors[name]
            if self._active_behavior == name:
                self._active_behavior = None

    def activate(self, name: str) -> bool:
        """
        Activate a behavior by name.

        Deactivates the currently active behavior first.

        Returns:
            True if activation succeeded, False if behavior not found
        """
        if name not in self._behaviors:
            return False

        # Deactivate current
        if self._active_behavior and self._active_behavior in self._behaviors:
            self._behaviors[self._active_behavior].deactivate()

        # Activate new
        self._behaviors[name].reset()
        self._behaviors[name].activate()
        self._active_behavior = name
        return True

    def deactivate_all(self) -> None:
        """Deactivate all behaviors."""
        for behavior in self._behaviors.values():
            if behavior.is_active:
                behavior.deactivate()
        self._active_behavior = None

    def compute(self, context: RobotContext) -> Velocity:
        """
        Compute velocity from active behavior.

        Returns:
            Velocity command (defaults to stop if no behavior active)
        """
        if self._active_behavior and self._active_behavior in self._behaviors:
            behavior = self._behaviors[self._active_behavior]
            if behavior.is_active:
                velocity = behavior.compute(context)
                if velocity is not None:
                    return velocity

        # Default: stop
        return Velocity.stop()

    @property
    def active_behavior(self) -> Optional[str]:
        """Name of currently active behavior."""
        return self._active_behavior

    @property
    def available_behaviors(self) -> list[str]:
        """List of registered behavior names."""
        return list(self._behaviors.keys())

    def get_behavior(self, name: str) -> Optional[Behavior]:
        """Get a behavior by name."""
        return self._behaviors.get(name)
