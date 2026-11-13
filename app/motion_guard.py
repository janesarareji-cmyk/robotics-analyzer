"""Motion guard (U3.4): one place that turns a wanted q change into an allowed one.

Every motion input (joint sliders, Drag TCP, IK Apply, gripper, playback) asks the guard.
``filter(q_new)`` returns the last collision-free q on the straight joint segment from the
current q (``CollisionService.first_free``) and a ``blocked`` flag; ``reject_jump(q_new)`` is for
inputs that jump to a far pose (IK solution): the whole target is refused when it collides.
``status`` receives the text "Blocked: A touches B". The check at the final q is published through
the service (``collision_changed``, red highlight).
"""
from __future__ import annotations

from typing import Any, Callable, Sequence

from app.collision_service import CollisionService
from app.kin_utils import current_q


class MotionGuard:
    def __init__(self, service: CollisionService, get_model: Callable[[], Any],
                 status: Callable[[str], None] | None = None):
        self.service = service
        self._get_model = get_model
        self._status = status
        self.text = ""                         # text of the last block ("" when it was free)
        self.blocked_count = 0

    @property
    def active(self) -> bool:
        return self.service.enabled and self.service.world is not None

    def _report(self, text: str) -> None:
        self.text = text
        if text:
            self.blocked_count += 1
            if self._status is not None:
                self._status(text)

    def filter(self, q_new: Sequence[float], q_old: Sequence[float] | None = None):
        """``(q_final, blocked)``: ``q_new`` itself when free, else the last free q on the move."""
        model = self._get_model()
        if not self.active or model is None:
            return list(map(float, q_new)), False
        old = current_q(model) if q_old is None else list(q_old)
        final = self.service.first_free(old, q_new)
        blocked = max(abs(a - b) for a, b in zip(final, q_new)) > 1e-12 if len(final) else False
        self.service.check(final)               # publish the state (highlight)
        if blocked:
            blk = self.service.last_block
            self._report(f"Blocked: {blk.text()}" if blk is not None else "Blocked by collision")
        else:
            self.text = ""
        return [float(v) for v in final], blocked

    def reject_jump(self, q_new: Sequence[float]):
        """``(q_new, False)`` when ``q_new`` is free; ``(None, True)`` when it collides."""
        if not self.active:
            return list(map(float, q_new)), False
        res = self.service.check(q_new, emit=False)
        if res.ok:
            self.text = ""
            return list(map(float, q_new)), False
        self._report(f"Blocked: {res.text()}")
        return None, True
