"""Kinematics helpers shared by UI and IK glue (R4.3, R4.5)."""
import core_bridge as cb


def singularity_at(cm, q, mode="full", threshold=1e-3):
    """Singularity indicators of the TCP Jacobian at ``q`` (R4.5).

    ``cb.jacobian_tcp`` then ``cb.analyze_singularity``. Returns the dict.
    """
    return cb.analyze_singularity(cb.jacobian_tcp(cm, q), mode=mode, threshold=threshold)


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


def current_q(model):
    """Return the current q vector (``model.dof()`` entries) from the joint q values."""
    q = [0.0] * model.dof()
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx >= 0:
            q[q_idx] = float(model.joints[j_idx].q)
    return q


def dof_joint_types(model):
    """Per DOF "revolute" / "prismatic" (type of the master joint of ``q_index_map()``)."""
    out = ["revolute"] * model.dof()
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx >= 0:
            out[q_idx] = str(model.joints[j_idx].type).split(".")[-1].lower()
    return out


def write_q(model, q):
    """Write a q vector (one entry per DOF) into the master joints' ``q``."""
    if len(q) != model.dof():
        raise ValueError("q needs one entry per DOF")
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx >= 0:
            model.joints[j_idx].q = float(q[q_idx])


class CompiledCache:
    """Compiled kinematic / dynamics model cache keyed on a change counter (R4.9).

    ``invalidate()`` bumps the counter (robot replaced or edited). ``get_cm`` /
    ``get_dm`` rebuild only when the counter moved, never on a q change.
    """

    def __init__(self):
        self.version = 0
        self.compile_count = 0
        self.dyn_build_count = 0
        self._cm = None
        self._dm = None
        self._cm_version = -1
        self._dm_version = -1

    def invalidate(self):
        self.version += 1

    def get_cm(self, model):
        if self._cm is None or self._cm_version != self.version:
            self._cm = cb.compile(model)
            self._cm_version = self.version
            self.compile_count += 1
        return self._cm

    def get_dm(self, model):
        cm = self.get_cm(model)
        if self._dm is None or self._dm_version != self.version:
            self._dm = cb.build_dyn_model(model, cm)
            self._dm_version = self.version
            self.dyn_build_count += 1
        return self._dm
