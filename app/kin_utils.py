"""Kinematics helpers shared by UI and IK glue (R4.3)."""


def dof_limits(model):
    """Return ``(lo, hi)`` lists of size ``model.dof()``.

    Limits come from the master joint of each DOF (``q_index_map()``).
    Mimic joint limits are not enforced in R4.
    """
    n = model.dof()
    lo = [0.0] * n
    hi = [0.0] * n
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx < 0:
            continue
        lim = model.joints[j_idx].limits
        lo[q_idx] = float(lim.lower)
        hi[q_idx] = float(lim.upper)
    return lo, hi
