"""
app/importers/urdf.py
R3.3: URDF file -> URM RobotModel (+ ImportReport). No meshes, no UI.

Entry point: import_urdf(path) -> ImportResult. Never raises for bad input.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional

import core_bridge as cb

from . import visuals as vis
from .import_report import (
    ImportReport,
    ImportResult,
    SEV_ERROR,
    SEV_INFO,
    SEV_WARNING,
    URDF_BAD_XML,
    URDF_CONTINUOUS_LIMITS,
    URDF_CYCLE,
    URDF_DUPLICATE_NAME,
    URDF_EXTRA_VISUAL,
    URDF_FILE_NOT_FOUND,
    URDF_IGNORED_ELEMENT,
    URDF_INVALID_MODEL,
    URDF_MESH_FORMAT,
    URDF_MESH_MISSING,
    URDF_MESH_TOO_LARGE,
    URDF_MIMIC_UNKNOWN,
    URDF_MISSING_LIMIT,
    URDF_MISSING_LINK,
    URDF_MULTIPLE_PARENTS,
    URDF_MULTIPLE_ROOTS,
    URDF_NO_INERTIAL,
    URDF_ROOT_NO_INERTIAL,
    URDF_TCP_CHOSEN,
    URDF_UNKNOWN_JOINT_TYPE,
    URDF_UNSUPPORTED_JOINT,
    URDF_XACRO,
    URDF_ZERO_AXIS,
    URM_PHYSICAL,
)

# Element kinds skipped (R3.5 handles visual/collision/material,
# R3.7 handles inertial).
_IGNORED_KINDS = (
    "transmission",
    "gazebo",
)

_SUPPORTED_TYPES = ("revolute", "continuous", "prismatic", "fixed")
_UNSUPPORTED_TYPES = ("floating", "planar")
_ZERO_AXIS_EPS = 1e-9
_XACRO_RE = re.compile(r"</?xacro:")


class _Fatal(Exception):
    """Stops the import. Turned into one error issue by import_urdf()."""

    def __init__(self, code: str, message: str, element: Optional[str] = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.element = element


class _RawJoint:
    """Joint as read from XML, before the tree is numbered."""

    __slots__ = ("name", "type_s", "parent", "child", "el", "limit_el")

    def __init__(self, name, type_s, parent, child, el, limit_el):
        self.name = name
        self.type_s = type_s
        self.parent = parent
        self.child = child
        self.el = el
        self.limit_el = limit_el


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def import_urdf(path, search_roots=None) -> ImportResult:
    """Import a URDF file. Errors go to the report; model is None on failure.

    `search_roots`: extra folders used to resolve package:// mesh paths.
    """
    report = ImportReport()
    try:
        root = _read_root(path)
        model, visuals, collisions = _build(
            root, report, Path(path).resolve().parent, list(search_roots or [])
        )
    except _Fatal as exc:
        report.add(SEV_ERROR, exc.code, exc.message, exc.element)
        return ImportResult(None, report)
    return ImportResult(model, report, visuals, collisions)


# ---------------------------------------------------------------------------
# Reading the file
# ---------------------------------------------------------------------------

def _read_root(path) -> ET.Element:
    p = Path(path)
    try:
        data = p.read_bytes()
    except OSError as exc:
        raise _Fatal(URDF_FILE_NOT_FOUND, f"cannot read '{p}': {exc.strerror or exc}")

    text = data.decode("utf-8", errors="replace")
    if "xmlns:xacro" in text or _XACRO_RE.search(text) or "${" in text:
        raise _Fatal(
            URDF_XACRO,
            "xacro not supported, convert with `xacro` first",
        )

    try:
        root = ET.fromstring(data)
    except ET.ParseError as exc:
        raise _Fatal(URDF_BAD_XML, f"XML parse error: {exc}")

    if root.tag != "robot":
        raise _Fatal(URDF_BAD_XML, f"root tag is <{root.tag}>, expected <robot>")
    return root


# ---------------------------------------------------------------------------
# Small parsing helpers
# ---------------------------------------------------------------------------

def _vec(s: Optional[str], n: int, default, element: Optional[str] = None) -> List[float]:
    if s is None:
        return [float(x) for x in default]
    parts = s.split()
    if len(parts) != n:
        raise _Fatal(URDF_BAD_XML, f"expected {n} numbers, got '{s}'", element)
    try:
        return [float(p) for p in parts]
    except ValueError:
        raise _Fatal(URDF_BAD_XML, f"bad number in '{s}'", element)


def _num(s: Optional[str], default: float, element: Optional[str] = None) -> float:
    if s is None:
        return float(default)
    try:
        return float(s)
    except ValueError:
        raise _Fatal(URDF_BAD_XML, f"bad number '{s}'", element)


def _pose(el: Optional[ET.Element], element: Optional[str]):
    p = cb.Pose()
    if el is None:
        p.xyz = [0.0, 0.0, 0.0]
        p.rpy = [0.0, 0.0, 0.0]
    else:
        p.xyz = _vec(el.get("xyz"), 3, (0.0, 0.0, 0.0), element)
        p.rpy = _vec(el.get("rpy"), 3, (0.0, 0.0, 0.0), element)
    return p


# ---------------------------------------------------------------------------
# Model build
# ---------------------------------------------------------------------------

def _build(root: ET.Element, report: ImportReport, urdf_dir: Path,
           search_roots: List[str]):
    """Returns (model | None, visuals, collisions)."""
    _report_ignored(root, report)

    # Links: names must be unique.
    link_names: List[str] = []
    for el in root.findall("link"):
        name = el.get("name", "")
        if name in link_names:
            raise _Fatal(URDF_DUPLICATE_NAME, f"duplicate link name '{name}'", name)
        link_names.append(name)

    # Joints: type, names, parent/child lookup, multiple parents, axis.
    raw: List[_RawJoint] = []
    joint_names = set()
    child_seen: Dict[str, str] = {}
    for el in root.findall("joint"):
        name = el.get("name", "")
        if name in joint_names:
            raise _Fatal(URDF_DUPLICATE_NAME, f"duplicate joint name '{name}'", name)
        joint_names.add(name)

        type_s = el.get("type", "")
        if type_s in _UNSUPPORTED_TYPES:
            raise _Fatal(
                URDF_UNSUPPORTED_JOINT,
                f"joint '{name}' has type '{type_s}', not supported",
                name,
            )
        if type_s not in _SUPPORTED_TYPES:
            raise _Fatal(
                URDF_UNKNOWN_JOINT_TYPE,
                f"joint '{name}' has unknown type '{type_s}'",
                name,
            )

        parent_el = el.find("parent")
        child_el = el.find("child")
        parent = parent_el.get("link", "") if parent_el is not None else ""
        child = child_el.get("link", "") if child_el is not None else ""
        if parent not in link_names or child not in link_names:
            raise _Fatal(
                URDF_MISSING_LINK,
                f"joint '{name}' names link '{parent}' -> '{child}', "
                "a link is missing",
                name,
            )

        if child in child_seen:
            raise _Fatal(
                URDF_MULTIPLE_PARENTS,
                f"link '{child}' is child of joints '{child_seen[child]}' and '{name}'",
                child,
            )
        child_seen[child] = name

        if type_s != "fixed":
            axis_el = el.find("axis")
            axis = _vec(
                axis_el.get("xyz") if axis_el is not None else None,
                3,
                (1.0, 0.0, 0.0),
                name,
            )
            if math.sqrt(sum(a * a for a in axis)) < _ZERO_AXIS_EPS:
                raise _Fatal(URDF_ZERO_AXIS, f"joint '{name}' has a zero-length axis", name)

        raw.append(_RawJoint(name, type_s, parent, child, el, el.find("limit")))

    # Tree: root = the one link that is no joint's child.
    child_names = set(child_seen)
    roots = [n for n in link_names if n not in child_names]
    if not roots:
        raise _Fatal(URDF_CYCLE, "no root link: the joints form a cycle")
    if len(roots) > 1:
        raise _Fatal(
            URDF_MULTIPLE_ROOTS,
            "more than one root link: " + ", ".join(f"'{r}'" for r in roots),
        )
    root_name = roots[0]

    # BFS numbering: root = 0, children in file order of their joints.
    index: Dict[str, int] = {root_name: 0}
    depth: Dict[str, int] = {root_name: 0}
    order: List[str] = [root_name]
    queue = deque([root_name])
    while queue:
        cur = queue.popleft()
        for rj in raw:
            if rj.parent != cur:
                continue
            index[rj.child] = len(order)
            depth[rj.child] = depth[cur] + 1
            order.append(rj.child)
            queue.append(rj.child)

    if len(order) != len(link_names):
        unreached = [n for n in link_names if n not in index]
        raise _Fatal(
            URDF_CYCLE,
            "links not reached from root '" + root_name + "': "
            + ", ".join(f"'{n}'" for n in unreached),
        )

    # Canonical joint order: joints[i].child_link == i + 1.
    raw.sort(key=lambda rj: index[rj.child])

    # Joints -> cb.Joint.
    joints = []
    for rj in raw:
        joints.append(_build_joint(rj, index, joint_names, report))

    # Links -> cb.Link (name + inertial data).
    link_els = {el.get("name", ""): el for el in root.findall("link")}
    links = []
    for i, name in enumerate(order):
        lk = cb.Link()
        lk.name = name
        _read_inertial(link_els[name], lk, i == 0, report)
        links.append(lk)

    # tcp_link: deepest link, ties -> lowest index.
    tcp_idx = 0
    for i, name in enumerate(order):
        if depth[name] > depth[order[tcp_idx]]:
            tcp_idx = i
    # R8.3: our own exporter writes <robotics_analyzer tcp_link="name"/>; other tools ignore it.
    hint = root.find("robotics_analyzer")
    if hint is not None and hint.get("tcp_link") in index:
        tcp_idx = index[hint.get("tcp_link")]

    model = cb.RobotModel()
    md = cb.RobotMetadata()
    md.name = root.get("name") or "Unnamed Robot"
    model.metadata = md
    for lk in links:
        model.links.append(lk)
    for j in joints:
        model.joints.append(j)
    model.tcp_link = tcp_idx

    report.add(
        SEV_INFO,
        URDF_TCP_CHOSEN,
        f"tcp_link set to link '{order[tcp_idx]}' (index {tcp_idx}). "
        "It can be changed later (R3.4 'Set TCP link').",
        order[tcp_idx],
    )

    messages = model.validate()
    if messages:
        for msg in messages:
            report.add(SEV_ERROR, URDF_INVALID_MODEL, msg)
        return None, [], []
    for msg in model.validate_physical():
        report.add(SEV_WARNING, URM_PHYSICAL, msg)
    visuals, collisions = _parse_geometry(root, index, report, urdf_dir, search_roots)
    return model, visuals, collisions


def _read_inertial(link_el: ET.Element, lk, is_root: bool,
                   report: ImportReport) -> None:
    """<inertial> -> Link.inertia. Tensor is rotated into the link frame."""
    name = link_el.get("name", "")
    el = link_el.find("inertial")
    if el is None:
        if is_root:
            report.add(
                SEV_INFO, URDF_ROOT_NO_INERTIAL,
                f"root link '{name}' has no <inertial>; mass stays 0", name,
            )
        else:
            report.add(
                SEV_WARNING, URDF_NO_INERTIAL,
                f"link '{name}' has no <inertial>; mass stays 0", name,
            )
        return
    origin = _pose(el.find("origin"), name)
    mass_el = el.find("mass")
    mass = _num(mass_el.get("value") if mass_el is not None else None, 0.0, name)
    in_el = el.find("inertia")
    g = (lambda k: _num(in_el.get(k), 0.0, name)) if in_el is not None         else (lambda k: 0.0)
    ixx, ixy, ixz, iyy, iyz, izz = (
        g("ixx"), g("ixy"), g("ixz"), g("iyy"), g("iyz"), g("izz"))
    import numpy as np

    rot = np.asarray(cb.pose_to_iso(origin), dtype=float)[:3, :3]
    tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    t = rot @ tensor @ rot.T
    inertia = cb.Inertia()
    inertia.mass = mass
    inertia.com = [float(x) for x in origin.xyz]
    inertia.inertia_tensor = [float(t[0, 0]), float(t[0, 1]), float(t[0, 2]),
                              float(t[1, 1]), float(t[1, 2]), float(t[2, 2])]
    lk.inertia = inertia


def _build_joint(rj: _RawJoint, index: Dict[str, int], joint_names: set,
                 report: ImportReport) -> cb.Joint:
    name = rj.name
    j = cb.Joint()
    j.name = name
    j.parent_link = index[rj.parent]
    j.child_link = index[rj.child]
    j.kinematics = cb.JointKinematics.OriginAxis
    j.origin = _pose(rj.el.find("origin"), name)

    if rj.type_s == "fixed":
        j.type = cb.JointType.Fixed
        j.limits = cb.default_limits(cb.JointType.Fixed)
        j.q = 0.0
        return j

    if rj.type_s == "prismatic":
        j.type = cb.JointType.Prismatic
    else:
        j.type = cb.JointType.Revolute

    axis_el = rj.el.find("axis")
    axis = _vec(
        axis_el.get("xyz") if axis_el is not None else None,
        3,
        (1.0, 0.0, 0.0),
        name,
    )
    norm = math.sqrt(sum(a * a for a in axis))
    j.axis = [a / norm for a in axis]

    j.limits = _joint_limits(rj, report)
    j.q = min(max(0.0, j.limits.lower), j.limits.upper)

    mimic_el = rj.el.find("mimic")
    if mimic_el is not None:
        target = mimic_el.get("joint", "")
        if target not in joint_names:
            raise _Fatal(
                URDF_MIMIC_UNKNOWN,
                f"joint '{name}' mimics '{target}', which is not a joint",
                name,
            )
        mi = cb.Mimic()
        mi.joint = target
        mi.multiplier = _num(mimic_el.get("multiplier"), 1.0, name)
        mi.offset = _num(mimic_el.get("offset"), 0.0, name)
        j.mimic = mi

    return j


def _joint_limits(rj: _RawJoint, report: ImportReport) -> cb.JointLimits:
    name = rj.name
    jtype = cb.JointType.Prismatic if rj.type_s == "prismatic" else cb.JointType.Revolute
    dflt = cb.default_limits(jtype)
    lim = cb.JointLimits()
    lim_el = rj.limit_el

    if rj.type_s == "continuous":
        report.add(
            SEV_WARNING,
            URDF_CONTINUOUS_LIMITS,
            f"continuous joint '{name}' mapped to limits [-pi, pi]",
            name,
        )
        lim.lower = -math.pi
        lim.upper = math.pi
    elif lim_el is None:
        lim.lower = dflt.lower
        lim.upper = dflt.upper
    else:
        lim.lower = _num(lim_el.get("lower"), 0.0, name)
        lim.upper = _num(lim_el.get("upper"), 0.0, name)

    if lim_el is None:
        report.add(
            SEV_WARNING,
            URDF_MISSING_LIMIT,
            f"joint '{name}' has no <limit>; using defaults",
            name,
        )
        lim.vel = dflt.vel
        lim.effort = dflt.effort
        return lim

    lim.vel = _num(lim_el.get("velocity"), dflt.vel, name) \
        if lim_el.get("velocity") is not None else dflt.vel
    lim.effort = _num(lim_el.get("effort"), dflt.effort, name) \
        if lim_el.get("effort") is not None else dflt.effort
    missing = [k for k in ("velocity", "effort") if lim_el.get(k) is None]
    if missing:
        report.add(
            SEV_WARNING,
            URDF_MISSING_LIMIT,
            f"joint '{name}' <limit> has no {' or '.join(missing)}; using defaults",
            name,
        )
    return lim


def _report_ignored(root: ET.Element, report: ImportReport) -> None:
    """Info once per skipped element kind."""
    seen: List[str] = []
    for el in root.iter():
        if el.tag in _IGNORED_KINDS and el.tag not in seen:
            seen.append(el.tag)
    for kind in seen:
        report.add(
            SEV_INFO,
            URDF_IGNORED_ELEMENT,
            f"<{kind}> elements are skipped on import",
            None,
        )


# ---------------------------------------------------------------------------
# Visual / collision geometry (R3.5)
# ---------------------------------------------------------------------------

def _named_materials(root: ET.Element) -> Dict[str, List[float]]:
    out: Dict[str, List[float]] = {}
    for m in root.findall("material"):
        c = m.find("color")
        if c is not None and c.get("rgba") is not None:
            out[m.get("name", "")] = _vec(
                c.get("rgba"), 4, vis.DEFAULT_RGBA, m.get("name")
            )
    return out


def _rgba(el: ET.Element, named: Dict[str, List[float]], link: str) -> List[float]:
    m = el.find("material")
    if m is None:
        return list(vis.DEFAULT_RGBA)
    c = m.find("color")
    if c is not None and c.get("rgba") is not None:
        return _vec(c.get("rgba"), 4, vis.DEFAULT_RGBA, link)
    return list(named.get(m.get("name", ""), vis.DEFAULT_RGBA))


def _make_shape(el: ET.Element, link_name: str, link_index: int,
                named: Dict[str, List[float]]) -> Optional[vis.LinkVisual]:
    geom = el.find("geometry")
    if geom is None:
        return None
    org = el.find("origin")
    xyz = _vec(org.get("xyz") if org is not None else None, 3, (0, 0, 0), link_name)
    rpy = _vec(org.get("rpy") if org is not None else None, 3, (0, 0, 0), link_name)
    rgba = _rgba(el, named, link_name)

    shape = None
    kind = ""
    for kind in vis.KINDS:
        shape = geom.find(kind)
        if shape is not None:
            break
    if shape is None:
        return None

    if kind == "box":
        params = {"size": _vec(shape.get("size"), 3, (1, 1, 1), link_name)}
    elif kind == "cylinder":
        params = {"radius": _num(shape.get("radius"), 1.0, link_name),
                  "length": _num(shape.get("length"), 1.0, link_name)}
    elif kind == "sphere":
        params = {"radius": _num(shape.get("radius"), 1.0, link_name)}
    else:
        params = {
            "filename": shape.get("filename", ""),
            "scale": _vec(shape.get("scale"), 3, (1, 1, 1), link_name),
        }
        if not params["filename"]:
            return None
    return vis.LinkVisual(link_index, link_name, kind, params, xyz, rpy, rgba)


def _resolve_mesh_visual(v: vis.LinkVisual, urdf_dir: Path, search_roots,
                         report: Optional[ImportReport]) -> None:
    """Resolve the mesh path; warn (visuals only) and set fallback."""
    fname = v.params.get("filename", "")
    path = vis.resolve_mesh(fname, urdf_dir, search_roots)
    if path is None:
        v.fallback = True
        if report is not None:
            report.add(
                SEV_WARNING, URDF_MESH_MISSING,
                f"mesh '{fname}' of link '{v.link_name}' not found; "
                "a placeholder cube is drawn", v.link_name,
            )
        return
    v.mesh_path = path
    if not vis.mesh_format_ok(path):
        v.fallback = True
        if report is not None:
            report.add(
                SEV_WARNING, URDF_MESH_FORMAT,
                f"mesh '{fname}' of link '{v.link_name}' has format "
                f"'{Path(path).suffix}'; only .stl .obj .ply are read. "
                "A placeholder cube is drawn", v.link_name,
            )
        return
    try:
        mesh = vis.read_mesh_file(path)
    except Exception as exc:
        v.fallback = True
        if report is not None:
            report.add(
                SEV_WARNING, URDF_MESH_FORMAT,
                f"mesh '{fname}' of link '{v.link_name}' cannot be read: {exc}",
                v.link_name,
            )
        return
    if report is not None and vis.triangle_count(mesh) > vis.MAX_TRIANGLES:
        report.add(
            SEV_WARNING, URDF_MESH_TOO_LARGE,
            f"mesh '{fname}' of link '{v.link_name}' has more than "
            f"{vis.MAX_TRIANGLES} triangles; it is not drawn", v.link_name,
        )


def _parse_geometry(root: ET.Element, index: Dict[str, int], report: ImportReport,
                    urdf_dir: Path, search_roots: List[str]):
    named = _named_materials(root)
    visuals: List[vis.LinkVisual] = []
    collisions: List[vis.LinkVisual] = []
    for link_el in root.findall("link"):
        name = link_el.get("name", "")
        li = index[name]
        first = True
        for el in link_el.findall("visual"):
            if not first:
                report.add(
                    SEV_INFO, URDF_EXTRA_VISUAL,
                    f"link '{name}' has more than one <visual>; extra ones are skipped",
                    name,
                )
                continue
            v = _make_shape(el, name, li, named)
            if v is None:
                continue
            first = False
            if v.kind == "mesh":
                _resolve_mesh_visual(v, urdf_dir, search_roots, report)
            visuals.append(v)
        for el in link_el.findall("collision"):
            v = _make_shape(el, name, li, named)
            if v is None:
                continue
            if v.kind == "mesh":
                _resolve_mesh_visual(v, urdf_dir, search_roots, None)
            collisions.append(v)
    return visuals, collisions
