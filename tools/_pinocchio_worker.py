"""Pinocchio worker. Runs INSIDE a separate env that has pinocchio + numpy.

Usage: python tools/_pinocchio_worker.py in.json out.json
in.json : {"urdf": path, "mimic_fixed": [joint names to lock at 0],
           "tcp_link": name, "gravity": [0,0,-9.81],
           "samples": [{"q": {joint: value}, "qd": {...}, "qdd": {...}}]}
out.json: {"version": str, "joints": [names], "samples": [{"frames": {link: 4x4},
           "J": 6xn (rows v,w, world-aligned at TCP origin, columns in "joints" order),
           "M": nxn, "g": n, "tau": n}]}

NOT tested in the main dev environment (no pinocchio wheel for Python 3.13 Windows).
"""
import json
import sys


def main(inp, outp):
    import numpy as np
    import pinocchio as pin

    cfg = json.load(open(inp))
    model = pin.buildModelFromUrdf(cfg["urdf"])
    model.gravity.linear = np.array(cfg["gravity"])
    lock_ids = [model.getJointId(n) for n in cfg.get("mimic_fixed", [])
                if model.existJointName(n)]
    if lock_ids:
        model = pin.buildReducedModel(model, lock_ids, np.zeros(model.nq))
    data = model.createData()
    names = [model.names[i] for i in range(1, model.njoints)
             if model.joints[i].nq == 1]
    idx_q = [model.joints[model.getJointId(n)].idx_q for n in names]
    idx_v = [model.joints[model.getJointId(n)].idx_v for n in names]
    out = {"version": pin.__version__, "joints": names, "samples": []}
    fid = model.getFrameId(cfg["tcp_link"])
    for s in cfg["samples"]:
        q = np.zeros(model.nq)
        qd = np.zeros(model.nv)
        qdd = np.zeros(model.nv)
        for n, iq, iv in zip(names, idx_q, idx_v):
            q[iq] = s["q"][n]
            qd[iv] = s["qd"][n]
            qdd[iv] = s["qdd"][n]
        pin.forwardKinematics(model, data, q, qd, qdd)
        pin.updateFramePlacements(model, data)
        frames = {}
        for f in model.frames:
            if f.type == pin.FrameType.BODY:
                frames[f.name] = data.oMf[model.getFrameId(f.name)].homogeneous.tolist()
        J = pin.computeFrameJacobian(model, data, q, fid,
                                     pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        M = pin.crba(model, data, q)
        M = np.triu(M) + np.triu(M, 1).T
        tau = pin.rnea(model, data, q, qd, qdd)
        g = pin.computeGeneralizedGravity(model, data, q)
        out["samples"].append({
            "frames": frames,
            "J": J[:, idx_v].tolist(),
            "M": M[np.ix_(idx_v, idx_v)].tolist(),
            "g": g[idx_v].tolist(),
            "tau": tau[idx_v].tolist(),
        })
    json.dump(out, open(outp, "w"))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
