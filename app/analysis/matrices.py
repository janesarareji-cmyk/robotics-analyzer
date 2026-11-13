"""Live matrices of the arm at one q (U4b.5). No Qt dependency.

``compute_matrices`` gives the 4x4 tool pose (base frame and world frame), the transform of one
chosen joint (``T_parent_child`` and the cumulative ``T_base_link``), the joint values, the 6 x N
TCP Jacobian and its manipulability / sigma_min. ``format_matrix`` / ``export_text`` write them as
plain text, a numpy expression or LaTeX; the numpy text can be pasted into Python and read back.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

import core_bridge as cb
from app.kin_utils import SINGULAR_WARN, current_q, dof_joint_types

FORMATS = ("text", "numpy", "latex")


@dataclass
class MatrixData:
    q: np.ndarray
    dof_names: list
    dof_types: list                          # "revolute" / "prismatic" per DOF
    T_base_tcp: np.ndarray                   # tool pose in the robot base frame
    T_world_tcp: np.ndarray | None           # only when the base pose is not the identity
    joint_names: list = field(default_factory=list)
    joint: int = 0
    T_parent_child: np.ndarray | None = None
    T_base_link: np.ndarray | None = None
    jacobian: np.ndarray | None = None       # (6, N), world frame
    manipulability: float = 0.0
    sigma_min: float = 0.0
    sigma_max: float = 0.0

    @property
    def near_singular(self) -> bool:
        return self.sigma_min < SINGULAR_WARN


def base_iso(model: Any) -> np.ndarray:
    """4x4 pose of the robot base in the world."""
    return np.asarray(cb.pose_to_iso(model.base), float)


def _dof_names(model: Any) -> list:
    out = [""] * model.dof()
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx >= 0:
            out[q_idx] = model.joints[j_idx].name or f"joint{j_idx + 1}"
    return out


def joint_names(model: Any) -> list:
    return [(j.name or f"joint{i + 1}") for i, j in enumerate(model.joints)]


def compute_matrices(model: Any, q: Sequence[float] | None = None, fk: dict | None = None,
                     cm: Any = None, joint: int = 0, jacobian: bool = True) -> MatrixData:
    """Matrices at ``q`` (default: the model q). ``fk`` = a ``cb.fk`` result for that q (saves a
    second FK), ``cm`` = compiled model (needed for the Jacobian)."""
    qv = np.asarray(current_q(model) if q is None else q, float)
    if fk is None:
        fk = cb.fk(model, list(qv))
    T0 = base_iso(model)
    ident = np.allclose(T0, np.eye(4), atol=1e-12)
    T_w = np.asarray(fk["tcp"], float)
    inv0 = np.linalg.inv(T0)
    names = joint_names(model)
    d = MatrixData(qv, _dof_names(model), dof_joint_types(model), inv0 @ T_w,
                   None if ident else T_w, names)
    if names:
        joint = min(max(int(joint), 0), len(names) - 1)
        from app.panels.model_panel import joint_links
        p, c = joint_links(model)[joint]
        frames = fk["frames"]
        Tp, Tc = np.asarray(frames[p], float), np.asarray(frames[c], float)
        d.joint = joint
        d.T_parent_child = np.linalg.inv(Tp) @ Tc
        d.T_base_link = inv0 @ Tc
    if jacobian:
        cm = cm if cm is not None else cb.compile(model)
        J = np.asarray(cb.jacobian_tcp(cm, list(qv)), float)
        d.jacobian = J
        s = cb.analyze_singularity(J)
        d.manipulability = float(s["manipulability"])
        d.sigma_min = float(s["sigma_min"])
        d.sigma_max = float(s["sigma_max"])
    return d


# ---------------------------------------------------------------------------- formatting

def pose_for_units(T: np.ndarray, units: str) -> np.ndarray:
    """Copy of a 4x4 pose with the translation in ``units`` ("m" or "mm")."""
    out = np.array(T, float)
    if units == "mm":
        out[:3, 3] *= 1000.0
    elif units != "m":
        raise ValueError("units must be 'm' or 'mm'")
    return out


def format_matrix(M: Any, fmt: str = "text", decimals: int = 4, name: str | None = None) -> str:
    """One matrix as text, a numpy expression or a LaTeX bmatrix."""
    A = np.atleast_2d(np.asarray(M, float))
    rows = [[f"{v:.{decimals}f}" for v in r] for r in A]
    if fmt == "text":
        w = max(len(x) for r in rows for x in r)
        body = "\n".join("  ".join(x.rjust(w) for x in r) for r in rows)
        return (f"{name} =\n" if name else "") + body
    if fmt == "numpy":
        body = "np.array([" + ",\n".join(("          " if i else "") + "[" + ", ".join(r) + "]"
                                         for i, r in enumerate(rows)) + "])"
        return f"{name} = {body}" if name else body
    if fmt == "latex":
        body = "\\begin{bmatrix}\n" + " \\\\\n".join("  " + " & ".join(r) for r in rows) \
            + "\n\\end{bmatrix}"
        return f"{name} = {body}" if name else body
    raise ValueError(f"fmt must be one of {FORMATS}")


def q_row(d: MatrixData, units_deg: bool = True) -> np.ndarray:
    """Joint values for display: degrees for revolute, millimetres for prismatic DOFs."""
    out = d.q.copy()
    for i, t in enumerate(d.dof_types):
        out[i] = np.degrees(out[i]) if t == "revolute" else out[i] * 1000.0
    return out


def export_text(d: MatrixData, fmt: str = "text", units: str = "mm", decimals: int = 4,
                with_jacobian: bool = True) -> str:
    """All matrices of ``d`` in one block, in ``fmt``."""
    parts = [format_matrix(pose_for_units(d.T_base_tcp, units), fmt, decimals, "T_base_tcp")]
    if d.T_world_tcp is not None:
        parts.append(format_matrix(pose_for_units(d.T_world_tcp, units), fmt, decimals,
                                   "T_world_tcp"))
    if d.T_parent_child is not None:
        jn = d.joint_names[d.joint]
        parts.append(format_matrix(pose_for_units(d.T_parent_child, units), fmt, decimals,
                                   "T_parent_child"))
        parts.append(format_matrix(pose_for_units(d.T_base_link, units), fmt, decimals,
                                   "T_base_link"))
        if fmt == "text":
            parts[-2] = f"# joint {jn}\n" + parts[-2]
    qv = q_row(d)
    parts.append(format_matrix(qv.reshape(1, -1), fmt, decimals, "q"))
    if with_jacobian and d.jacobian is not None:
        parts.append(format_matrix(d.jacobian, fmt, decimals, "J"))
    head = {"text": "# units: translation in " + units + ", q in deg (revolute) / mm (prismatic)\n",
            "numpy": "import numpy as np\n# translation in " + units
                     + ", q in deg (revolute) / mm (prismatic)\n",
            "latex": "% translation in " + units + ", q in deg / mm\n"}[fmt]
    return head + "\n\n".join(parts) + "\n"


def parse_numpy_block(text: str) -> dict:
    """Read the ``name = np.array([...])`` assignments of an ``export_text(fmt="numpy")`` block."""
    text = "\n".join(ln for ln in text.splitlines() if not ln.startswith("import"))
    ns: dict = {"np": np}
    exec(compile(text, "<matrices>", "exec"), {"__builtins__": {}}, ns)     # noqa: S102
    return {k: np.asarray(v, float) for k, v in ns.items() if k != "np"}


def parse_latex_block(text: str) -> dict:
    """Read the ``name = \\begin{bmatrix} ... \\end{bmatrix}`` blocks of ``export_text("latex")``."""
    import re
    out = {}
    for name, body in re.findall(r"(\w+) = \\begin\{bmatrix\}(.*?)\\end\{bmatrix\}", text, re.S):
        rows = [r for r in body.replace("\n", " ").split("\\\\") if r.strip()]
        out[name] = np.array([[float(x) for x in r.split("&")] for r in rows])
    return out
