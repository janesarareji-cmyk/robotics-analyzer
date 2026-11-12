"""R8.3: report data. One ``ReportData`` is rendered to HTML (html_report) and PDF (pdf_report).

``build_report_data`` collects the sections; a section without data is left out and named in
``missing`` (never an error). Graph images are PNG bytes drawn with matplotlib Agg (no GUI).
"""
from __future__ import annotations

import datetime as _dt
import io
from dataclasses import dataclass, field

import numpy as np

import core_bridge as cb

GRAPH_KINDS = ("joint_pos", "joint_torque", "tcp_xyz", "energy")


@dataclass
class ReportData:
    title: str = "Robot report"
    robot_name: str = ""
    app_version: str = ""
    date: str = ""
    n_dof: int = 0
    convention: str = ""                    # "Standard DH" / "Modified DH" when DH joints exist
    tcp_link: str = ""
    base: list = field(default_factory=list)      # [x, y, z, roll, pitch, yaw]
    tool: list = field(default_factory=list)
    joints: list = field(default_factory=list)    # dicts: name type kinematics parent child
    #                                               lower upper vel effort mimic
    dh_rows: list = field(default_factory=list)   # dicts: name a alpha d theta
    origin_rows: list = field(default_factory=list)   # dicts: name xyz rpy axis
    mass: dict | None = None                # total, links [(name, mass, com)], com_world
    workspace: dict | None = None           # volume, filled, voxel_size, n_samples, bounds
    graphs: list = field(default_factory=list)    # (title, png bytes)
    plan: dict | None = None                # planner, success, time, length, iterations, ...
    collision: dict | None = None           # fit_mode, n_shapes, n_edits, n_env
    missing: list = field(default_factory=list)   # names of omitted sections


def _enum(x) -> str:
    return str(x).split(".")[-1]


def _joint_rows(model):
    rows = []
    links = [(int(p), int(c)) for p, c in cb.effective_links(model)]
    names = [lk.name or f"link{i}" for i, lk in enumerate(model.links)]
    for i, j in enumerate(model.joints):
        p, c = links[i]
        mimic = ""
        if j.mimic is not None:
            mimic = f"{j.mimic.joint} x{j.mimic.multiplier:g} + {j.mimic.offset:g}"
        rows.append(dict(name=j.name, type=_enum(j.type).lower(), kinematics=_enum(j.kinematics),
                         parent=names[p] if p < len(names) else str(p),
                         child=names[c] if c < len(names) else str(c),
                         lower=float(j.limits.lower), upper=float(j.limits.upper),
                         vel=float(j.limits.vel), effort=float(j.limits.effort), mimic=mimic))
    return rows


def _param_rows(model):
    modified = _enum(model.convention).lower() == "modified"
    dh, oa = [], []
    for j in model.joints:
        if _enum(j.kinematics).lower() == "originaxis":
            ax = [float(a) for a in j.axis]
            oa.append(dict(name=j.name, xyz=[float(v) for v in j.origin.xyz],
                           rpy=[float(v) for v in j.origin.rpy], axis=ax))
        else:
            d = j.mdh if modified else j.dh
            dh.append(dict(name=j.name, a=float(d.a), alpha=float(d.alpha), d=float(d.d),
                           theta=float(d.theta)))
    return dh, oa


def _mass_section(model):
    masses = [(lk.name or f"link{i}", float(lk.inertia.mass), [float(c) for c in lk.inertia.com])
              for i, lk in enumerate(model.links)]
    total = sum(m for _n, m, _c in masses)
    if not total > 0:
        return None
    com_world = None
    try:
        frames = cb.fk(model, [0.0] * int(model.dof()))["frames"]
        acc = np.zeros(3)
        for k, lk in enumerate(model.links):
            m = float(lk.inertia.mass)
            if m > 0:
                F = np.asarray(frames[k], float)
                acc += m * (F @ np.append(np.asarray(lk.inertia.com, float), 1.0))[:3]
        com_world = [float(v) for v in acc / total]
    except Exception:  # noqa: BLE001
        com_world = None
    return dict(total=float(total), links=[m for m in masses if m[1] > 0], com_world=com_world)


def _workspace_section(res):
    if not res or not isinstance(res, dict) or "volume" not in res:
        return None
    out = dict(volume=float(res["volume"]), filled=int(res.get("filled", 0)),
               voxel_size=float(res.get("voxel_size", 0.0)), n_samples=int(res.get("n_samples", 0)),
               bounds=None)
    try:
        from app.analysis.workspace_analysis import voxel_centers
        c = np.asarray(voxel_centers(res), float)
        if len(c):
            h = out["voxel_size"] / 2
            out["bounds"] = [[float(v) for v in c.min(axis=0) - h],
                             [float(v) for v in c.max(axis=0) + h]]
    except Exception:  # noqa: BLE001
        pass
    return out


def _plan_section(plan, violations):
    if plan is None:
        return None
    out = dict(planner=str(getattr(plan, "planner", "")), success=bool(plan.success),
               time=float(plan.time), length=float(plan.length), iterations=int(plan.iterations),
               reason=str(plan.reason), n_waypoints=0 if plan.path is None else len(plan.path),
               violations=None)
    if violations is not None:
        out["violations"] = [dict(kind=v.kind, dof=int(v.dof), t=float(v.t), value=float(v.value),
                                  limit=float(v.limit)) for v in violations.violations]
    return out


def _collision_section(cfg, n_shapes):
    if cfg is None and not n_shapes:
        return None
    return dict(fit_mode=getattr(cfg, "fit_mode", "obb"), n_shapes=int(n_shapes),
                n_edits=len(getattr(cfg, "edits", {}) or {}),
                n_env=len(getattr(cfg, "environment", []) or []))


def render_graph_png(series, title: str = "", size=(640, 300), dpi=100) -> bytes:
    """PNG bytes of a line plot of ``series`` (objects with name, unit, t, y). Agg backend."""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    fig = Figure(figsize=(size[0] / dpi, size[1] / dpi), dpi=dpi)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    units = []
    for s in series:
        ax.plot(np.asarray(s.t), np.asarray(s.y), linewidth=1.2, label=f"{s.name} [{s.unit}]")
        if s.unit not in units:
            units.append(s.unit)
    ax.set_xlabel("t [s]")
    ax.set_ylabel(", ".join(units))
    ax.set_title(title, fontsize=10)
    ax.grid(True, alpha=0.3)
    if series:
        ax.legend(fontsize=7, loc="best")
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    return buf.getvalue()


def graphs_from_arrays(arrays, model, kinds=GRAPH_KINDS):
    """``[(title, png)]`` for the recorder arrays (``{name: (t, data)}``); empty when no data."""
    from app.graphs.series import KIND_LABELS, graph_data
    from app.kin_utils import dof_joint_types

    if not arrays or "q" not in arrays:
        return []
    t, _q = arrays["q"]
    if len(t) == 0:
        return []
    names = [""] * int(model.dof())
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx >= 0:
            names[q_idx] = str(model.joints[j_idx].name)
    types = list(dof_joint_types(model))
    out = []
    for kind in kinds:
        ser = graph_data(arrays, kind, names=names, joint_types=types)
        out.append((KIND_LABELS.get(kind, kind), render_graph_png(ser, KIND_LABELS.get(kind, kind))))
    return out


def build_report_data(model, name: str = "", workspace=None, arrays=None, plan=None,
                      violations=None, collision_config=None, n_collision_shapes: int = 0,
                      app_version: str | None = None, title: str = "Robot report") -> ReportData:
    """Collect the report sections. All arguments after ``model`` are optional.

    ``workspace``: result of ``analyze_workspace``. ``arrays``: ``Recorder.export_arrays()``.
    ``plan``: ``PlanResult``; ``violations``: ``ViolationReport`` of the planned trajectory.
    A section without data is omitted and listed in ``missing``.
    """
    if app_version is None:
        from app.sim.export import APP_VERSION as app_version
    names = [lk.name or f"link{i}" for i, lk in enumerate(model.links)]
    tcp = len(names) - 1 if model.tcp_link == -1 else int(model.tcp_link)
    dh, oa = _param_rows(model)
    rd = ReportData(
        title=title, robot_name=name or model.metadata.name or "robot", app_version=str(app_version),
        date=_dt.datetime.now().strftime("%Y-%m-%d %H:%M"), n_dof=int(model.dof()),
        convention=(_enum(model.convention) + " DH") if dh else "",
        tcp_link=names[tcp] if 0 <= tcp < len(names) else "",
        base=[float(v) for v in list(model.base.xyz) + list(model.base.rpy)],
        tool=[float(v) for v in list(model.tool.xyz) + list(model.tool.rpy)],
        joints=_joint_rows(model), dh_rows=dh, origin_rows=oa)
    rd.mass = _mass_section(model)
    rd.workspace = _workspace_section(workspace)
    if arrays:
        try:
            rd.graphs = graphs_from_arrays(arrays, model)
        except Exception:  # noqa: BLE001
            rd.graphs = []
    rd.plan = _plan_section(plan, violations)
    rd.collision = _collision_section(collision_config, n_collision_shapes)
    for key, label in (("mass", "mass and COM (no link has mass)"),
                       ("workspace", "workspace (not computed)"),
                       ("graphs", "graphs (no recorded run)"),
                       ("plan", "plan results (no plan)"),
                       ("collision", "collision settings (none)")):
        if not getattr(rd, key):
            rd.missing.append(label)
    return rd
