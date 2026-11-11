"""Simulation core (R6.1). No Qt dependency."""
from .engine import SimClock, SimEngine, rk4_step
from .state import SimState

__all__ = ["SimClock", "SimEngine", "SimState", "rk4_step"]
