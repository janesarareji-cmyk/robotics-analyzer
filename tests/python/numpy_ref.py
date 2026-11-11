"""Independent numpy oracle for kinematics tests (R4.2, reused by R4.3 to R4.10).

Pure numpy. Never imports ``core_bridge`` and never calls ``cb.fk`` /
``cb.compile``. ``spec_from_model`` only READS plain attributes of a
``RobotModel`` (names, types, parameters) and rebuilds the joint transforms
with numpy formulas written here.

Spec format (plain dict, see ``spec_from_model``)::

    {"n_links": int, "n_dof": int, "tcp": int,
     "base": 4x4, "tool": 4x4,
     "joints": [{"parent": int, "child": int, "type": "revolute"|"prismatic"|"fixed",
                 "pre": 4x4, "post": 4x4, "axis": (3,), "src": int,
                 "mult": float, "offset": float}, ...]}

Joint transform: ``A = pre @ motion(axis, mult*q[src] + offset) @ post``,
``frames[child] = frames[parent] @ A``, ``frames[0] = base``.
"""
import numpy as np


# --------------------------------------------------------------- primitives
def hom(R=None, t=None):
    """4x4 homogeneous matrix from rotation R (3x3) and translation t (3)."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def rot_axis(axis, angle):
    """Rotation matrix about a (not necessarily unit) axis, Rodrigues formula."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def rz(t):
    return hom(rot_axis((0, 0, 1), t))


def rx(t):
    return hom(rot_axis((1, 0, 0), t))


def tx(v):
    return hom(t=(v, 0, 0))


def tz(v):
    return hom(t=(0, 0, v))


def rpy_matrix(r, p, y):
    """R = Rz(y) Ry(p) Rx(r) (URM fixed-axis XYZ convention)."""
    return rot_axis((0, 0, 1), y) @ rot_axis((0, 1, 0), p) @ rot_axis((1, 0, 0), r)


def pose_matrix(xyz, rpy):
    return hom(rpy_matrix(*rpy), xyz)


def so3_log(R):
    """Rotation vector of R (angle*axis), valid for angles well below pi."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if ang < 1e-9:
        return 0.5 * v
    return v * (ang / (2.0 * np.sin(ang)))


# ----------------------------------------------------------------------- FK
def fk_numpy(spec, q):
    """World frames of all links. Returns ``(frames, tcp)`` (list of 4x4, 4x4)."""
    q = np.asarray(q, float)
    assert q.size == spec["n_dof"], "q size mismatch"
    frames = [np.eye(4) for _ in range(spec["n_links"])]
    frames[0] = np.array(spec["base"], float)
    for j in sorted(spec["joints"], key=lambda j: j["child"]):
        A = np.array(j["pre"], float)
        if j["type"] != "fixed":
            qe = j["mult"] * q[j["src"]] + j["offset"]
            ax = np.asarray(j["axis"], float)
            if j["type"] == "revolute":
                A = A @ hom(rot_axis(ax, qe))
            else:
                A = A @ hom(t=ax / np.linalg.norm(ax) * qe)
        A = A @ j["post"]
        frames[j["child"]] = frames[j["parent"]] @ A
    tcp = frames[spec["tcp"]] @ spec["tool"]
    return frames, tcp


def point_numpy(spec, q, link, point_local=(0, 0, 0)):
    """World position of a point fixed in link ``link``."""
    frames, _ = fk_numpy(spec, q)
    return (frames[link] @ np.append(np.asarray(point_local, float), 1.0))[:3]


def numeric_jacobian(spec, q, link, point_local=(0, 0, 0), h=1e-6):
    """6 x n_dof Jacobian by central differences (world frame).

    Rows 0-2: d p / d q of the point. Rows 3-5: ``log(R(q+h) R(q-h)^T) / 2h``
    with R the rotation of link ``link``.
    """
    q = np.asarray(q, float)
    pl = np.append(np.asarray(point_local, float), 1.0)
    J = np.zeros((6, spec["n_dof"]))
    for k in range(spec["n_dof"]):
        qp, qm = q.copy(), q.copy()
        qp[k] += h
        qm[k] -= h
        fp, _ = fk_numpy(spec, qp)
        fm, _ = fk_numpy(spec, qm)
        J[:3, k] = ((fp[link] @ pl)[:3] - (fm[link] @ pl)[:3]) / (2 * h)
        J[3:, k] = so3_log(fp[link][:3, :3] @ fm[link][:3, :3].T) / (2 * h)
    return J


def numeric_jacobian_tcp(spec, q, h=1e-6):
    """Numeric Jacobian at the TCP point (tool translation) of link ``tcp``."""
    return numeric_jacobian(spec, q, spec["tcp"], spec["tool"][:3, 3], h)


# --------------------------------------------------------- spec from a model
def _enum_name(x):
    return str(x).split(".")[-1].lower()


def spec_from_model(model, independent_mimic=False):
    """Build a plain spec from a ``RobotModel`` (reads attributes only).

    ``independent_mimic=True`` treats every mimic joint as its own DOF
    (mult 1, offset 0): used to check master columns against the slave's.
    """
    joints = model.joints
    nj = len(joints)
    legacy = nj >= 1 and all(j.parent_link == 0 and j.child_link == 0 for j in joints)
    links = ([(i, i + 1) for i in range(nj)] if legacy
             else [(j.parent_link, j.child_link) for j in joints])
    types = [_enum_name(j.type) for j in joints]
    names = {j.name: i for i, j in enumerate(joints)}
    mimic = [None if (independent_mimic or j.mimic is None) else j.mimic for j in joints]

    qmap, n_dof = [-1] * nj, 0
    for i in range(nj):
        if types[i] != "fixed" and mimic[i] is None:
            qmap[i] = n_dof
            n_dof += 1

    modified = _enum_name(model.convention) == "modified"
    out = []
    for i, j in enumerate(joints):
        pre, post, axis = np.eye(4), np.eye(4), np.array([0.0, 0.0, 1.0])
        if _enum_name(j.kinematics) == "originaxis":
            pre = pose_matrix(list(j.origin.xyz), list(j.origin.rpy))
            if types[i] != "fixed":
                axis = np.asarray(list(j.axis), float)
                axis = axis / np.linalg.norm(axis)
        else:
            d = j.mdh if modified else j.dh
            a, al, dd, th = d.a, d.alpha, d.d, d.theta
            if modified:
                if types[i] == "fixed":
                    pre = rx(al) @ tx(a) @ rz(th) @ tz(dd)
                elif types[i] == "revolute":
                    pre, post = rx(al) @ tx(a) @ rz(th), tz(dd)
                else:
                    pre = rx(al) @ tx(a) @ rz(th) @ tz(dd)
            else:
                if types[i] == "fixed":
                    pre = rz(th) @ tz(dd) @ tx(a) @ rx(al)
                elif types[i] == "revolute":
                    pre, post = rz(th), tz(dd) @ tx(a) @ rx(al)
                else:
                    pre, post = rz(th) @ tz(dd), tx(a) @ rx(al)
        src, mult, off = -1, 1.0, 0.0
        if types[i] != "fixed":
            if mimic[i] is None:
                src = qmap[i]
            else:
                cur, A, B = i, 1.0, 0.0
                while mimic[cur] is not None:
                    m = mimic[cur]
                    B = A * m.offset + B
                    A = A * m.multiplier
                    cur = names[m.joint]
                src, mult, off = qmap[cur], A, B
        out.append({"parent": links[i][0], "child": links[i][1], "type": types[i],
                    "pre": pre, "post": post, "axis": axis,
                    "src": src, "mult": mult, "offset": off})
    n_links = max(len(model.links), 1)
    tcp = n_links - 1 if model.tcp_link == -1 else model.tcp_link
    return {"n_links": n_links, "n_dof": n_dof, "tcp": tcp,
            "base": pose_matrix(list(model.base.xyz), list(model.base.rpy)),
            "tool": pose_matrix(list(model.tool.xyz), list(model.tool.rpy)),
            "joints": out}


# ----------------------------------------------------------------- dynamics
def inertias_from_model(model):
    """Per-link ``(mass, com(3), Ic(3x3))`` read from ``RobotModel.links[k].inertia``.

    ``Ic`` is built from ``inertia_tensor = (ixx, ixy, ixz, iyy, iyz, izz)``.
    """
    out = []
    for lk in model.links:
        i = lk.inertia
        t = list(i.inertia_tensor)
        Ic = np.array([[t[0], t[1], t[2]], [t[1], t[3], t[4]], [t[2], t[4], t[5]]], float)
        out.append((float(i.mass), np.array(list(i.com), float), Ic))
    return out


def _richardson(f, x, h):
    """4th-order central derivative of an array valued f at scalar-shifted x."""
    d1 = (f(h) - f(-h)) / (2 * h)
    d2 = (f(2 * h) - f(-2 * h)) / (4 * h)
    return (4 * d1 - d2) / 3.0


def numeric_jacobian4(spec, q, link, point_local=(0, 0, 0), h=1e-3):
    """Like ``numeric_jacobian`` but 4th order (Richardson on the central
    difference; rotation rows via ``log(R(q+h) R(q-h)^T) / 2h``).

    Needed by ``numpy_dynamics``: its second derivative of M by differences
    would amplify the noise of a 2nd order, h = 1e-6 Jacobian.
    """
    q = np.asarray(q, float)
    pl = np.append(np.asarray(point_local, float), 1.0)
    J = np.zeros((6, spec["n_dof"]))

    def pair(k, s):
        qp, qm = q.copy(), q.copy()
        qp[k] += s
        qm[k] -= s
        fp, _ = fk_numpy(spec, qp)
        fm, _ = fk_numpy(spec, qm)
        lin = (fp[link] @ pl)[:3] - (fm[link] @ pl)[:3]
        ang = so3_log(fp[link][:3, :3] @ fm[link][:3, :3].T)
        return np.concatenate([lin, ang]) / (2 * s)

    for k in range(spec["n_dof"]):
        J[:, k] = (4 * pair(k, h) - pair(k, 2 * h)) / 3.0
    return J


def _mass_matrix_numpy(spec, inertias, q):
    n = spec["n_dof"]
    frames, _ = fk_numpy(spec, q)
    M = np.zeros((n, n))
    for k in range(1, spec["n_links"]):
        m, c, Ic = inertias[k]
        if m == 0.0 and not np.any(Ic):
            continue
        J = numeric_jacobian4(spec, q, k, c)
        R = frames[k][:3, :3]
        Iw = R @ Ic @ R.T
        M += m * J[:3].T @ J[:3] + J[3:].T @ Iw @ J[3:]
    return M


def numpy_dynamics(spec, inertias, q, qd, g):
    """Independent Lagrangian reference. Returns ``(M, c, grav)``.

    ``tau = M qdd + c + grav`` with ``c = C(q, qd) qd`` (Christoffel form,
    ``dM/dq`` by 4th-order central differences) and
    ``grav = dV/dq`` with ``V = -sum m g . p_com`` (the actuator torque that
    holds the robot against gravity; the generalised force of gravity itself
    is ``-dV/dq``, and RNEA returns the actuator torque).
    Never calls ``core_bridge``. ``g`` is the world gravity vector.
    """
    q = np.asarray(q, float)
    qd = np.asarray(qd, float)
    g = np.asarray(g, float)
    n = spec["n_dof"]
    M = _mass_matrix_numpy(spec, inertias, q)

    def V(qq):
        frames, _ = fk_numpy(spec, qq)
        v = 0.0
        for k in range(1, spec["n_links"]):
            m, c, _ = inertias[k]
            v -= m * (g @ (frames[k] @ np.append(c, 1.0))[:3])
        return v

    grav = np.zeros(n)
    dM = []
    for i in range(n):
        def shift(s, i=i):
            qq = q.copy()
            qq[i] += s
            return qq
        grav[i] = _richardson(lambda s: np.array(V(shift(s))), None, 1e-3)
        dM.append(_richardson(lambda s: _mass_matrix_numpy(spec, inertias, shift(s)), None, 1e-3))
    c = np.zeros(n)
    for k in range(n):
        for i in range(n):
            for j in range(n):
                cijk = 0.5 * (dM[i][k, j] + dM[j][k, i] - dM[k][i, j])
                c[k] += cijk * qd[i] * qd[j]
    return M, c, grav
