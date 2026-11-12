"""Sample planner plugin: straight joint-space line, no collision check (demo only)."""
from types import SimpleNamespace

import numpy as np


def plan(model, start_q, goal_q, collision=None, limits=None, steps=20, **kwargs):
    start = np.asarray(start_q, float)
    goal = np.asarray(goal_q, float)
    if goal.shape != start.shape:
        return SimpleNamespace(path=None, success=False, time=0.0, iterations=0,
                               reason="linear planner needs a joint vector goal")
    s = np.linspace(0.0, 1.0, int(steps))[:, None]
    path = start + s * (goal - start)
    path[0], path[-1] = start, goal           # exact end points
    return SimpleNamespace(path=path, success=True, time=0.0, iterations=1, reason="")


def register(host):
    host.register_planner("Linear", plan)
