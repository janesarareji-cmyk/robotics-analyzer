"""Random test robots (R4.2, reused by R4.3 to R4.10).

``random_robot`` returns a ``core_bridge.RobotModel`` that passes
``model.validate()``.
"""
import numpy as np
import core_bridge as cb


def pose(xyz, rpy):
    """Build a ``cb.Pose`` from two 3-sequences."""
    p = cb.Pose()
    p.xyz = [float(v) for v in xyz]
    p.rpy = [float(v) for v in rpy]
    return p


def random_robot(seed, n, kinds=("origin_axis",), prismatic=0.0, branch=False,
                 mimic=False, modified=False):
    """Random robot with a serial chain of ``n`` joints (links 0..n).

    kinds     : joint kinematics drawn per joint from ``"dh"`` / ``"origin_axis"``
                (a string is accepted too). DH joints use the model convention.
    prismatic : probability that a chain joint is prismatic (else revolute).
    branch    : add one extra revolute origin_axis joint ``jb`` (child link n+1)
                hanging off a random interior link; ``tcp_link`` stays ``n``
                (end of the chain), so the branch DOF is off the active path.
    mimic     : add one extra origin_axis joint ``jm`` (last joint) that mimics
                ``j0`` (multiplier 0.5..1.5, offset 0.1); same type as ``j0``.
    modified  : model convention Modified (``mdh`` params) instead of Standard.
    Chain joints are ``j0..j{n-1}`` with explicit links ``(i, i+1)``. Base and
    tool are random poses. Limits: +-pi (revolute), +-0.5 (prismatic).
    DOF order = URM joint order of non-fixed, non-mimic joints.
    """
    rng = np.random.default_rng(seed)
    if isinstance(kinds, str):
        kinds = (kinds,)
    m = cb.RobotModel()
    m.convention = cb.DHConvention.Modified if modified else cb.DHConvention.Standard
    m.base = pose(rng.uniform(-0.3, 0.3, 3), rng.uniform(-1, 1, 3))
    m.tool = pose(rng.uniform(-0.2, 0.2, 3), rng.uniform(-1, 1, 3))
    n_extra = int(bool(branch)) + int(bool(mimic))
    for i in range(n + n_extra + 1):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)

    def make(name, parent, child, kind, prism):
        j = cb.Joint()
        j.name = name
        j.parent_link, j.child_link = parent, child
        j.type = cb.JointType.Prismatic if prism else cb.JointType.Revolute
        lim = cb.JointLimits()
        lim.lower, lim.upper = (-0.5, 0.5) if prism else (-np.pi, np.pi)
        lim.vel, lim.effort = 1.0, 10.0
        j.limits = lim
        if kind == "dh":
            p = cb.MDHParams() if modified else cb.DHParams()
            p.a = float(rng.uniform(-0.5, 0.5))
            p.alpha = float(rng.uniform(-np.pi, np.pi))
            p.d = float(rng.uniform(-0.3, 0.3))
            p.theta = float(rng.uniform(-1, 1))
            if modified:
                j.mdh = p
            else:
                j.dh = p
            j.kinematics = cb.JointKinematics.DH
        else:
            j.kinematics = cb.JointKinematics.OriginAxis
            j.origin = pose(rng.uniform(-0.5, 0.5, 3), rng.uniform(-np.pi, np.pi, 3))
            ax = rng.normal(size=3)
            j.axis = [float(v) for v in ax / np.linalg.norm(ax)]
        return j

    for i in range(n):
        kind = str(rng.choice(list(kinds)))
        prism = bool(rng.random() < prismatic)
        m.joints.append(make(f"j{i}", i, i + 1, kind, prism))
    nxt = n + 1
    if branch:
        parent = int(rng.integers(0, max(n, 1)))
        m.joints.append(make("jb", parent, nxt, "origin_axis", False))
        nxt += 1
    if mimic:
        parent = int(rng.integers(0, nxt))
        mj = make("jm", parent, nxt, "origin_axis", m.joints[0].type == cb.JointType.Prismatic)
        mm = cb.Mimic()
        mm.joint = "j0"
        mm.multiplier = float(rng.uniform(0.5, 1.5))
        mm.offset = 0.1
        mj.mimic = mm
        m.joints.append(mj)
    m.tcp_link = n
    return m


def random_q(model, rng):
    """Random q (size ``model.dof()``), inside the limits of each DOF's joint."""
    q = []
    for j in model.joints:
        if j.type == cb.JointType.Fixed or j.mimic is not None:
            continue
        q.append(float(rng.uniform(j.limits.lower, j.limits.upper)))
    return q


def random_inertias(model, rng, skip=()):
    """Give every link except link 0 (and ``skip``) random physical inertia.

    Mass 0.5..2 kg, COM within 0.3 m, full SPD tensor about the COM
    (random rotation of a diagonal), stored as (ixx, ixy, ixz, iyy, iyz, izz).
    """
    for k, lk in enumerate(model.links):
        if k == 0 or k in skip:
            continue
        A = rng.normal(size=(3, 3))
        Q, _ = np.linalg.qr(A)
        I = Q @ np.diag(rng.uniform(0.02, 0.2, 3)) @ Q.T
        i = cb.Inertia()
        i.mass = float(rng.uniform(0.5, 2.0))
        i.com = [float(v) for v in rng.uniform(-0.3, 0.3, 3)]
        i.inertia_tensor = [float(I[0, 0]), float(I[0, 1]), float(I[0, 2]),
                            float(I[1, 1]), float(I[1, 2]), float(I[2, 2])]
        lk.inertia = i


def fk_test_robots():
    """Robots for the batch FK tests (R5.2): name -> RobotModel (all compile)."""
    from pathlib import Path
    from app.importers import import_urdf

    out = {}
    six = import_urdf(str(Path(__file__).parent / "fixtures" / "urdf" / "six_dof_with_gripper.urdf")).model
    six.tcp_link = [i for i, lk in enumerate(six.links) if lk.name == "l6"][0]
    out["six_dof_fixture"] = six
    out["dh_6r"] = random_robot(11, 6, kinds="dh")
    out["origin_axis_12r"] = random_robot(12, 12)
    tree = random_robot(13, 8, kinds=("origin_axis", "dh"), prismatic=0.25, branch=True, mimic=True)
    tree.joints[2].type = cb.JointType.Fixed
    tree.joints[5].type = cb.JointType.Fixed
    out["tree_mimic_fixed"] = tree
    return out


def sample_q(model, n, seed=42):
    """Uniform random q (n, dof) inside the DOF limits."""
    from app.kin_utils import dof_limits
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(seed)
    return rng.uniform(lo, hi, size=(n, len(lo)))
