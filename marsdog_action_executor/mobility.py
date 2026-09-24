"""Chassis backend abstraction — decouples behavior execution from the chassis.

The action executor resolves ``behavior -> Stage -> ACT_*`` without knowing the
physical platform.  The chassis backend owns two capabilities:

  * **velocity outlet** — publish a body-frame velocity command
    (``TwistCommand`` = ``vx/vy/vyaw``).  Consumed by the closed-loop adapters
    (target approach, attention tracking, stationary expression, UWB follow)
    and by platform action plans.
  * **semantic action execution** — translate an ``ACT_*`` into platform motion
    (``execute_step``).

The Go2 backend maps its supported
``ACT_*`` to Unitree SportMode requests (sit/stand/move/dance) and publishes
closed-loop velocity via ``Move(vx, vy, vyaw)``.

The body-frame velocity command is carried by :class:`TwistCommand` from
``adapters.velocity``.  Its ``(linear_x, linear_y, angular_z)`` triple is the
platform-neutral ``(vx, vy, vyaw)`` body velocity used by both platforms.
"""

from __future__ import annotations

from typing import Any, Protocol

from .adapters.velocity import TwistCommand


class ChassisBackend(Protocol):
    """Chassis backend: velocity outlet + semantic action execution.

    Implementations are selected at launch time via ``chassis_type``
    (``go2`` or ``lite3``).  The behavior layer only talks to this
    protocol, never to a concrete publisher or motion adapter.
    """

    def publish_velocity(self, command: TwistCommand) -> None:
        """Publish a body-frame velocity command (vx/vy/vyaw)."""
        ...

    def execute_step(
        self,
        unit_config: dict[str, Any],
        ctx: Any = None,
        duration: float | None = None,
    ) -> bool:
        """Execute the motion mapped from one ``ACT_*`` unit."""
        ...

    def hold_position(self, duration_sec: float | None = None) -> bool:
        """Force zero velocity, optionally reaffirming it for a duration."""
        ...

    def cancel_step(self, step: Any = None) -> None:
        """Stop the active motion and publish zero velocity."""
        ...

    def emergency_stop(self) -> None:
        """Immediately stop and publish redundant zero-velocity commands."""
        ...
