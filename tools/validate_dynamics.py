"""Independent validation of the app's kinematics + dynamics against MuJoCo
(or, optionally, Pinocchio in a separate env).

    python tools/validate_dynamics.py --backend mujoco [--robots ur5 panda]
           [--urdf extra.urdf ...] [--samples 20] [--seed 0]
           [--out docs/validation/dynamics-validation.md]
    python tools/validate_dynamics.py --backend pinocchio --pinocchio-python PATH

Per robot the app side runs `core_bridge` (compile, fk, jacobian_tcp,
build_dyn_model, inverse_dynamics, mass_matrix, gravity_torque) on seeded random
(q, qd, qdd) within the joint limits; the reference side computes the same
quantities. Joints are matched by name. Mimic joints are locked at 0 (made
fixed) on both sides; the mimic master stays a normal DOF.
"""
import argparse
import json
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import core_bridge as cb  # noqa: E402
from app.importers.urdf import import_urdf  # noqa: E402

G = np.array([0.0, 0.0, -9.81])
TOL = {"fk": 1e-10, "jac": 1e-9, "M": 1e-9, "tau": 1e-9, "g": 1e-9}


# ------------------------------------------------------------------ URDF prep
def prepare_urdf(src, dst, mujoco_compiler=True):
    """Write a kinematics+inertia only copy of `src` to `dst`.

    Removes visual / collision / dynamics, converts mimic joints to fixed.
    Returns the list of locked (mimic) joint names.
    """
    tree = ET.parse(src)
    root = tree.getroot()
    for link in root.iter("link"):
        for tag in ("visual", "collision"):
            for e in link.findall(tag):
                link.remove(e)
    locked = []
    for j in root.iter("joint"):
        for e in j.findall("dynamics"):
            j.remove(e)
        mim = j.find("mimic")
        if mim is not None:
            j.remove(mim)
            j.set("type", "fixed")
            for e in j.findall("limit"):
                j.remove(e)
            locked.append(j.get("name"))
    if mujoco_compiler:
        mj = ET.SubElement(root, "mujoco")
        ET.SubElement(mj, "compiler", fusestatic="false", discardvisual="true",
                      balanceinertia="false", inertiafromgeom="false",
                      boundmass="0", boundinertia="0")
    tree.write(dst)
    return locked


def load_app(urdf_path, locked):
    res = import_urdf(str(urdf_path))
    model = res.model
    if model is None:
        raise RuntimeError("import failed: %s" % res.report)
    for j in model.joints:
        if j.name in locked or j.mimic is not None:
            j.mimic = None
            j.type = cb.JointType.Fixed
    return model


def dof_info(model):
    """[(joint name, lower, upper)] for movable joints in dof order."""
    qmap = model.q_index_map()
    rows = {}
    for j, k in zip(model.joints, qmap):
        if k >= 0:
            lo, hi = j.limits.lower, j.limits.upper
            if not (np.isfinite(lo) and np.isfinite(hi)) or hi <= lo:
                lo, hi = -np.pi, np.pi
            rows[k] = (j.name, lo, hi)
    return [rows[k] for k in sorted(rows)]


# ------------------------------------------------------------------ app side
def app_eval(model, samples):
    cm = cb.compile(model)
    dm = cb.build_dyn_model(model, cm)
    names = [l.name for l in model.links]
    out = []
    for s in samples:
        q, qd, qdd = list(s["q"]), list(s["qd"]), list(s["qdd"])
        f = cb.fk_compiled(cm, q)
        out.append({
            "frames": {n: np.asarray(T) for n, T in zip(names, f["frames"])},
            "J": np.asarray(cb.jacobian_tcp(cm, q)),
            "M": np.asarray(cb.mass_matrix(dm, q)),
            "g": np.asarray(cb.gravity_torque(dm, q, tuple(G))),
            "tau": np.asarray(cb.inverse_dynamics(dm, q, qd, qdd, tuple(G))),
        })
    return out


# ------------------------------------------------------------------ mujoco
def mujoco_eval(urdf_prepped, dofs, samples, tcp_link):
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(urdf_prepped))
    m.opt.gravity[:] = G
    m.opt.disableflags |= (mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
                           | mujoco.mjtDisableBit.mjDSBL_CONTACT
                           | mujoco.mjtDisableBit.mjDSBL_LIMIT)
    m.dof_damping[:] = 0
    m.dof_armature[:] = 0
    m.dof_frictionloss[:] = 0
    d = mujoco.MjData(m)
    qadr, vadr = [], []
    for name, _, _ in dofs:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError("joint %s missing in MuJoCo model" % name)
        qadr.append(m.jnt_qposadr[jid])
        vadr.append(m.jnt_dofadr[jid])
    if m.nv != len(dofs):
        raise RuntimeError("MuJoCo nv %d != app dof %d" % (m.nv, len(dofs)))
    body_names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i): i
                  for i in range(1, m.nbody)}
    tcp_id = body_names[tcp_link]
    out = []
    for s in samples:
        d.qpos[:] = 0
        d.qvel[:] = 0
        d.qacc[:] = 0
        for k in range(len(dofs)):
            d.qpos[qadr[k]] = s["q"][k]
            d.qvel[vadr[k]] = s["qd"][k]
        mujoco.mj_forward(m, d)
        for k in range(len(dofs)):  # mj_forward overwrites qacc: set it after
            d.qacc[vadr[k]] = s["qdd"][k]
        frames = {}
        for n, i in body_names.items():
            T = np.eye(4)
            T[:3, :3] = d.xmat[i].reshape(3, 3)
            T[:3, 3] = d.xpos[i]
            frames[n] = T
        # Jacobian at the TCP link origin, world aligned, rows [v; w]
        jp = np.zeros((3, m.nv))
        jr = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jp, jr, d.xpos[tcp_id], tcp_id)
        J = np.vstack([jp, jr])
        M = np.zeros((m.nv, m.nv))
        if hasattr(d, "qM"):  # MuJoCo 3.x signature
            mujoco.mj_fullM(m, M, d.qM)
        else:  # MuJoCo 4.x
            mujoco.mj_fullM(m, d, M)
        tau = np.zeros(m.nv)
        mujoco.mj_rne(m, d, 1, tau)
        qv, qa = d.qvel.copy(), d.qacc.copy()
        d.qvel[:] = 0
        d.qacc[:] = 0
        mujoco.mj_forward(m, d)
        g = np.zeros(m.nv)
        mujoco.mj_rne(m, d, 0, g)
        d.qvel[:], d.qacc[:] = qv, qa
        P = vadr
        out.append({"frames": frames, "J": J[:, P], "M": M[np.ix_(P, P)],
                    "g": g[P], "tau": tau[P]})
    return out, "MuJoCo " + mujoco.mj_versionString()


# --------------------------------------------------------------- pinocchio
def pinocchio_eval(pin_python, urdf_prepped, locked, dofs, samples, tcp_link):
    names = [n for n, _, _ in dofs]
    cfg = {"urdf": str(urdf_prepped), "mimic_fixed": [], "tcp_link": tcp_link,
           "gravity": G.tolist(),
           "samples": [{k: {n: float(s[k][i]) for i, n in enumerate(names)}
                        for k in ("q", "qd", "qdd")} for s in samples]}
    with tempfile.TemporaryDirectory() as td:
        a, b = Path(td, "in.json"), Path(td, "out.json")
        a.write_text(json.dumps(cfg))
        subprocess.run([pin_python, str(ROOT / "tools" / "_pinocchio_worker.py"),
                        str(a), str(b)], check=True)
        res = json.loads(b.read_text())
    order = [res["joints"].index(n) for n in names]
    out = []
    for r in res["samples"]:
        out.append({
            "frames": {k: np.array(v) for k, v in r["frames"].items()},
            "J": np.array(r["J"])[:, order],
            "M": np.array(r["M"])[np.ix_(order, order)],
            "g": np.array(r["g"])[order], "tau": np.array(r["tau"])[order]})
    return out, "Pinocchio " + res["version"]


# ------------------------------------------------------------------ compare
def _rel(a, b):
    return float(np.max(np.abs(a - b)) / max(1.0, np.max(np.abs(b))))


def compare(app, ref):
    r = {k: [0.0, 0.0] for k in ("fk", "jac", "M", "tau", "g")}  # abs, rel
    nframes = 0
    for a, b in zip(app, ref):
        common = [n for n in a["frames"] if n in b["frames"]]
        nframes = len(common)
        for n in common:
            e = float(np.max(np.abs(a["frames"][n] - b["frames"][n])))
            r["fk"][0] = max(r["fk"][0], e)
        for key, k in (("J", "jac"), ("M", "M"), ("tau", "tau"), ("g", "g")):
            r[k][0] = max(r[k][0], float(np.max(np.abs(a[key] - b[key]))))
            r[k][1] = max(r[k][1], _rel(a[key], b[key]))
    r["fk"][1] = r["fk"][0]
    r["nframes"] = nframes
    return r


def passes(r):
    ok = r["fk"][0] <= TOL["fk"]
    for k in ("jac", "M", "tau", "g"):
        ok = ok and r[k][1] <= TOL[k]
    return ok


def validate_robot(urdf, backend="mujoco", n=20, seed=0, pin_python=None):
    """Return (result dict, backend version string)."""
    urdf = Path(urdf)
    with tempfile.TemporaryDirectory() as td:
        prepped = Path(td) / (urdf.stem + "_prepped.urdf")
        locked = prepare_urdf(urdf, prepped, mujoco_compiler=(backend == "mujoco"))
        model = load_app(prepped, set(locked))
        dofs = dof_info(model)
        rng = np.random.default_rng(seed)
        samples = []
        for _ in range(n):
            samples.append({
                "q": [rng.uniform(lo, hi) for _, lo, hi in dofs],
                "qd": list(rng.normal(size=len(dofs))),
                "qdd": list(rng.normal(size=len(dofs)))})
        app = app_eval(model, samples)
        tcp_name = model.links[model.tcp_link].name
        if backend == "mujoco":
            ref, ver = mujoco_eval(prepped, dofs, samples, tcp_name)
        else:
            ref, ver = pinocchio_eval(pin_python, prepped, locked, dofs,
                                      samples, tcp_name)
    res = compare(app, ref)
    res["dof"] = len(dofs)
    res["locked"] = locked
    return res, ver


# -------------------------------------------------------------------- report
def fmt(x):
    return "%.1e" % x


def write_report(rows, ver, backend, out, n, seed):
    L = ["# Dynamics validation", "",
         "Independent check of the app kinematics and dynamics against **%s**." % ver,
         "", "Generated by `tools/validate_dynamics.py` (backend `%s`, %d seeded "
         "random samples per robot, seed %d, q within joint limits, qd and qdd "
         "standard normal, gravity (0, 0, -9.81))." % (backend, n, seed), "",
         "## How to run", "",
         "```", "python -m pip install mujoco",
         "python tools/validate_dynamics.py --backend mujoco", "```", "",
         "Options: `--robots ur5 panda` (names under `robots/`), `--urdf FILE` "
         "(extra URDF), `--samples N`, `--seed S`, `--out FILE`.", "",
         "## What is compared", "",
         "- FK: world pose (4x4) of every link frame that exists on both sides "
         "(max abs element error).",
         "- Jacobian: 6 x n at the TCP link origin, world-aligned, rows "
         "[linear; angular] (MuJoCo `mj_jac`).",
         "- Mass matrix M: app `mass_matrix` vs `mj_fullM`.",
         "- tau: app `inverse_dynamics(q, qd, qdd, g)` vs `mj_rne` with qacc "
         "(RNEA, includes Coriolis and gravity).",
         "- g: gravity torque (qd = qdd = 0).", "",
         "Tolerances: FK %s m, Jacobian / M / tau / g %s relative "
         "(max abs error divided by max(1, max abs reference))." %
         (fmt(TOL["fk"]), fmt(TOL["jac"])), "",
         "## Setup notes", "",
         "- URDF is preprocessed: visual, collision and dynamics elements are "
         "removed (only kinematics and inertials matter); the same file is "
         "loaded by the app importer and MuJoCo.",
         "- MuJoCo compiler: `fusestatic=false`, `discardvisual=true`, "
         "`balanceinertia=false`, `inertiafromgeom=false`; damping, armature, "
         "frictionloss set to 0; constraints, contacts and limits disabled.",
         "- Joints are matched by name. Mimic joints are made fixed (locked at "
         "0) on both sides; the mimic master stays a normal DOF. A mimic "
         "equality constraint is not used.", "",
         "## Results", "",
         "| robot | DOF | frames | FK abs | J rel | M rel | tau rel | g rel | result |",
         "|---|---|---|---|---|---|---|---|---|"]
    for name, r in rows:
        if isinstance(r, str):
            L.append("| %s | | | | | | | | ERROR: %s |" % (name, r))
            continue
        L.append("| %s | %d | %d | %s | %s | %s | %s | %s | %s |" % (
            name, r["dof"], r["nframes"], fmt(r["fk"][0]), fmt(r["jac"][1]),
            fmt(r["M"][1]), fmt(r["tau"][1]), fmt(r["g"][1]),
            "PASS" if passes(r) else "FAIL"))
    locks = [(n, r["locked"]) for n, r in rows
             if not isinstance(r, str) and r["locked"]]
    if locks:
        L += ["", "Locked mimic joints: " + "; ".join(
            "%s: %s" % (n, ", ".join(l)) for n, l in locks) + "."]
    L += ["", "## Pinocchio (optional)", "",
          "Pinocchio has no Python 3.13 Windows pip wheel, so it runs in a "
          "separate conda env. The `pinocchio` backend calls "
          "`tools/_pinocchio_worker.py` with that env's python (JSON in, JSON "
          "out). **This backend could not be tested here** (no pinocchio in "
          "the dev environment).", "",
          "```",
          "micromamba create -n pin -c conda-forge python=3.12 pinocchio numpy",
          "# or: conda create -n pin -c conda-forge python=3.12 pinocchio numpy",
          "micromamba run -n pin python -c \"import sys; print(sys.executable)\"",
          "python tools/validate_dynamics.py --backend pinocchio "
          "--pinocchio-python C:\\path\\to\\envs\\pin\\python.exe "
          "--out docs/validation/pinocchio-report.md", "```", ""]
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_text("\n".join(L) + "\n", encoding="utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--backend", choices=["mujoco", "pinocchio"], default="mujoco")
    ap.add_argument("--robots", nargs="*", default=None)
    ap.add_argument("--urdf", nargs="*", default=[])
    ap.add_argument("--samples", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--pinocchio-python", default=None)
    ap.add_argument("--out", default=str(ROOT / "docs/validation/dynamics-validation.md"))
    a = ap.parse_args(argv)
    if a.backend == "pinocchio" and not a.pinocchio_python:
        ap.error("--pinocchio-python is required for the pinocchio backend")
    files = []
    for p in sorted((ROOT / "robots").glob("*/*.urdf")):
        if a.robots is None or p.parent.name in a.robots:
            files.append((p.parent.name, p))
    for u in a.urdf:
        files.append((Path(u).stem, Path(u)))
    rows, ver = [], a.backend
    for name, p in files:
        try:
            r, ver = validate_robot(p, a.backend, a.samples, a.seed, a.pinocchio_python)
        except Exception as e:  # report and go on
            r = "%s: %s" % (type(e).__name__, str(e).splitlines()[0][:150])
        rows.append((name, r))
        print(name, r if isinstance(r, str) else ("PASS" if passes(r) else "FAIL"),
              flush=True)
    write_report(rows, ver, a.backend, a.out, a.samples, a.seed)
    print("wrote", a.out)
    bad = any(isinstance(r, str) or not passes(r) for _, r in rows)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
