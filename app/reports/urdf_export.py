"""R8.3: URM -> URDF export (no Qt).

``export_urdf(model, path, visuals, collisions)`` writes a URDF file (and copies meshes to
``<folder>/meshes``). DH joints are converted to origin/axis on export only: the DH child
frame sits behind the joint axis motion (``post`` of the compiled model), so the frame offset
``post`` is pushed into the next joint origins and into the link inertial / visual / collision
poses. The robot base pose is folded into the origins of the joints that start at the base
link; a tool offset becomes one extra fixed joint + link. The tcp link is stored in a
``<robotics_analyzer tcp_link="..."/>`` element (ignored by other URDF readers).
"""
from __future__ import annotations

import math
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import core_bridge as cb


@dataclass
class UrdfExport:
    path: str | None
    xml: str
    warnings: list = field(default_factory=list)
    meshes: list = field(default_factory=list)     # relative mesh paths written next to the file


# ------------------------------------------------------------------ small math
def _rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    if axis == "x":
        T[1:3, 1:3] = [[c, -s], [s, c]]
    elif axis == "y":
        T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, s, -s, c
    else:
        T[0:2, 0:2] = [[c, -s], [s, c]]
    return T


def _trans(x=0.0, y=0.0, z=0.0):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def _pose_matrix(p) -> np.ndarray:
    r, pi, y = (float(v) for v in p.rpy)
    T = _rot("z", y) @ _rot("y", pi) @ _rot("x", r)
    T[:3, 3] = [float(v) for v in p.xyz]
    return T


def _matrix_xyz_rpy(T):
    """xyz and rpy (R = Rz(yaw) Ry(pitch) Rx(roll)) of a 4x4 pose."""
    R = T[:3, :3]
    cp = math.hypot(R[0, 0], R[1, 0])
    if cp > 1e-9:
        roll, pitch, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], cp), \
            math.atan2(R[1, 0], R[0, 0])
    else:                                  # gimbal lock: yaw = 0
        pitch = math.copysign(math.pi / 2, -R[2, 0])
        roll = math.atan2(R[0, 1], R[1, 1]) if R[2, 0] < 0 else math.atan2(-R[0, 1], R[1, 1])
        yaw = 0.0
    return [float(v) for v in T[:3, 3]], [roll, pitch, yaw]


def _f(x) -> str:
    return repr(float(x))


def _v3(v) -> str:
    return " ".join(_f(x) for x in v)


def _enum(x) -> str:
    return str(x).split(".")[-1].lower()


# --------------------------------------------------------------- joint geometry
def _pre_post_axis(model, j, modified: bool):
    """(pre, post, axis) of one joint; same formulas as the compiled model."""
    t = _enum(j.type)
    pre, post, axis = np.eye(4), np.eye(4), [0.0, 0.0, 1.0]
    if _enum(j.kinematics) == "originaxis":
        pre = _pose_matrix(j.origin)
        if t != "fixed":
            a = np.asarray(list(j.axis), float)
            axis = list(a / np.linalg.norm(a))
        return pre, post, axis
    if modified:
        d = j.mdh
        a, al, dd, th = d.a, d.alpha, d.d, d.theta
        if t == "fixed":
            pre = _rot("x", al) @ _trans(a) @ _rot("z", th) @ _trans(z=dd)
        elif t == "revolute":
            pre, post = _rot("x", al) @ _trans(a) @ _rot("z", th), _trans(z=dd)
        else:
            pre = _rot("x", al) @ _trans(a) @ _rot("z", th) @ _trans(z=dd)
    else:
        d = j.dh
        a, al, dd, th = d.a, d.alpha, d.d, d.theta
        if t == "fixed":
            pre = _rot("z", th) @ _trans(z=dd) @ _trans(a) @ _rot("x", al)
        elif t == "revolute":
            pre, post = _rot("z", th), _trans(z=dd) @ _trans(a) @ _rot("x", al)
        else:
            pre, post = _rot("z", th) @ _trans(z=dd), _trans(a) @ _rot("x", al)
    return pre, post, axis


def _unique(names: list[str], prefix: str) -> list[str]:
    seen, out = set(), []
    for i, n in enumerate(names):
        n = str(n).strip() or f"{prefix}{i}"
        base, k = n, 1
        while n in seen:
            n = f"{base}_{k}"
            k += 1
        seen.add(n)
        out.append(n)
    return out


def _sub(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def _origin(parent, T):
    xyz, rpy = _matrix_xyz_rpy(T)
    _sub(parent, "origin", xyz=_v3(xyz), rpy=_v3(rpy))


# ------------------------------------------------------------------- the writer
def urdf_xml(model, visuals=(), collisions=(), mesh_names: dict | None = None):
    """Build the URDF text. ``mesh_names``: source mesh path -> path written in the file.

    Returns ``(xml_text, warnings, tcp_link_name)``. Raises ``ValueError`` for an invalid model.
    """
    msgs = list(model.validate())
    if msgs or len(model.links) == 0:
        raise ValueError("model is not valid: " + "; ".join(msgs or ["no links"]))
    mesh_names = mesh_names or {}
    warnings: list[str] = []
    eff = [(int(p), int(c)) for p, c in cb.effective_links(model)]
    n_links = len(model.links)
    modified = _enum(model.convention) == "modified"
    lname = _unique([lk.name for lk in model.links], "link")
    jname = _unique([j.name for j in model.joints], "joint")
    by_joint = {j.name: jname[i] for i, j in enumerate(model.joints)}
    base = _pose_matrix(model.base)
    tool = _pose_matrix(model.tool)
    tcp = n_links - 1 if model.tcp_link == -1 else int(model.tcp_link)

    P = {i: np.eye(4) for i in range(n_links)}       # frame offset: DH child frame = joint frame @ P
    joint_data = []
    for i, j in enumerate(model.joints):
        pre, post, axis = _pre_post_axis(model, j, modified)
        p, c = eff[i]
        P[c] = post
        joint_data.append((i, j, p, c, pre, axis))

    root = ET.Element("robot", name=(model.metadata.name or "robot"))
    _sub(root, "robotics_analyzer", tcp_link="")      # filled at the end

    # links
    vis_by, col_by = {}, {}
    for v in visuals:
        vis_by.setdefault(int(v.link_index), []).append(v)
    for v in collisions:
        col_by.setdefault(int(v.link_index), []).append(v)
    for k, lk in enumerate(model.links):
        el = _sub(root, "link", name=lname[k])
        inert = lk.inertia
        tens = [float(x) for x in inert.inertia_tensor]
        if inert.mass > 0 or any(tens):
            R, t = P[k][:3, :3], P[k][:3, 3]
            com = R @ np.asarray(inert.com, float) + t
            I = np.array([[tens[0], tens[1], tens[2]], [tens[1], tens[3], tens[4]],
                          [tens[2], tens[4], tens[5]]])
            I = R @ I @ R.T
            ie = _sub(el, "inertial")
            _sub(ie, "origin", xyz=_v3(com), rpy="0.0 0.0 0.0")
            _sub(ie, "mass", value=_f(inert.mass))
            _sub(ie, "inertia", ixx=_f(I[0, 0]), ixy=_f(I[0, 1]), ixz=_f(I[0, 2]),
                 iyy=_f(I[1, 1]), iyz=_f(I[1, 2]), izz=_f(I[2, 2]))
        for tag, items in (("visual", vis_by.get(k, [])), ("collision", col_by.get(k, []))):
            for n, v in enumerate(items):
                se = _sub(el, tag)
                vo = np.eye(4)
                vo[:3, :3] = _pose_matrix_rot(v)
                vo[:3, 3] = [float(x) for x in v.origin_xyz]
                _origin(se, P[k] @ vo)
                g = _sub(se, "geometry")
                _geometry(g, v, mesh_names, warnings)
                if tag == "visual":
                    m = _sub(se, "material", name=f"{lname[k]}_mat{n}")
                    _sub(m, "color", rgba=_v3(v.rgba))

    # joints
    for i, j, p, c, pre, axis in joint_data:
        t = _enum(j.type)
        origin = (base if p == 0 else np.eye(4)) @ P[p] @ pre
        je = _sub(root, "joint", name=jname[i], type=t)
        _sub(je, "parent", link=lname[p])
        _sub(je, "child", link=lname[c])
        _origin(je, origin)
        if t != "fixed":
            _sub(je, "axis", xyz=_v3(axis))
            lim = j.limits
            _sub(je, "limit", lower=_f(lim.lower), upper=_f(lim.upper), effort=_f(lim.effort),
                 velocity=_f(lim.vel))
            if j.mimic is not None:
                tgt = j.mimic.joint
                _sub(je, "mimic", joint=by_joint.get(tgt, tgt),
                     multiplier=_f(j.mimic.multiplier), offset=_f(j.mimic.offset))

    # tool offset -> extra fixed joint and link
    tcp_name = lname[tcp]
    tool_T = P[tcp] @ tool
    if not np.array_equal(tool_T, np.eye(4)):
        tl = _unique(lname + [f"{lname[tcp]}_tool"], "link")[-1]
        tj = _unique(jname + [f"{lname[tcp]}_tool_joint"], "joint")[-1]
        _sub(root, "link", name=tl)
        je = _sub(root, "joint", name=tj, type="fixed")
        _sub(je, "parent", link=lname[tcp])
        _sub(je, "child", link=tl)
        _origin(je, tool_T)
        tcp_name = tl
    root.find("robotics_analyzer").set("tcp_link", tcp_name)

    ET.indent(root)
    text = '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding="unicode") + "\n"
    return text, warnings, tcp_name


def _pose_matrix_rot(v):
    r, p, y = (float(x) for x in v.origin_rpy)
    return (_rot("z", y) @ _rot("y", p) @ _rot("x", r))[:3, :3]


def _geometry(g, v, mesh_names, warnings):
    if v.kind == "box":
        _sub(g, "box", size=_v3(v.params["size"]))
    elif v.kind == "cylinder":
        _sub(g, "cylinder", radius=_f(v.params["radius"]), length=_f(v.params["length"]))
    elif v.kind == "sphere":
        _sub(g, "sphere", radius=_f(v.params["radius"]))
    else:
        src = v.mesh_path
        fname = mesh_names.get(src) if src else None
        if fname is None:
            fname = v.params.get("filename", "")
            warnings.append(f"mesh of link '{v.link_name}' not found; kept '{fname}'")
        attrs = {"filename": fname}
        scale = [float(s) for s in v.params.get("scale", (1, 1, 1))]
        if scale != [1.0, 1.0, 1.0]:
            attrs["scale"] = _v3(scale)
        _sub(g, "mesh", **attrs)


def export_urdf(model, path=None, visuals=(), collisions=(), copy_meshes: bool = True) -> UrdfExport:
    """Write ``model`` as URDF to ``path`` (None: only build the text).

    ``visuals`` / ``collisions``: ``LinkVisual`` lists (see ``app.importers.visuals``). Meshes
    that exist on disk are copied to ``<folder of path>/meshes`` and referenced by a relative
    path (``package://`` is replaced). Returns ``UrdfExport`` (path, xml, warnings, meshes).
    """
    visuals, collisions = list(visuals or []), list(collisions or [])
    mesh_names: dict = {}
    used: set = set()
    if path is not None and copy_meshes:
        for v in visuals + collisions:
            if v.kind != "mesh" or not v.mesh_path or v.mesh_path in mesh_names:
                continue
            if not Path(v.mesh_path).is_file():
                continue
            base = Path(v.mesh_path).name
            name, k = base, 1
            while name in used:
                name = f"{Path(base).stem}_{k}{Path(base).suffix}"
                k += 1
            used.add(name)
            mesh_names[v.mesh_path] = "meshes/" + name
    text, warnings, _tcp = urdf_xml(model, visuals, collisions, mesh_names)
    out = UrdfExport(None if path is None else str(path), text, warnings)
    if path is None:
        return out
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if mesh_names:
        (p.parent / "meshes").mkdir(exist_ok=True)
        for src, rel in mesh_names.items():
            shutil.copyfile(src, p.parent / rel)
            out.meshes.append(rel)
    p.write_text(text, encoding="utf-8")
    return out
