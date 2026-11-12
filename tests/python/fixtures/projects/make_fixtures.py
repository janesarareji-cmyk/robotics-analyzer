"""
R8.4: generator of the stored project fixtures (run once, the files are committed).

    python tests/python/fixtures/projects/make_fixtures.py

Writes, next to this file, one REAL archive per project format version
(v1.0.0 ... v1.3.0, each with exactly the members that version wrote) and a
set of corrupt samples. The tests import ``reference_project`` and compare the
opened fixtures with it, so the reference model is built in code, not stored.

Layouts:
  1.0.0  project_metadata, robot_model, physical_properties, simulation_settings
  1.1.0  + visuals.json, assets/
  1.2.0  + trajectory.json
  1.3.0  + collision.json, configs.json
"""
from __future__ import annotations

import json
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
for extra in (REPO, REPO / "bindings", REPO / "tests" / "python"):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

import numpy as np  # noqa: E402

MESH = HERE.parent / "meshes" / "cube.stl"
STAMP = (2026, 11, 12, 0, 0, 0)

VERSIONS = ("1.0.0", "1.1.0", "1.2.0", "1.3.0")
# archive members present per version (assets/ handled by prefix)
MEMBERS = {
    "1.0.0": {"project_metadata.json", "robot_model.json", "physical_properties.json",
              "simulation_settings.json"},
    "1.1.0": {"visuals.json", "assets/"},
    "1.2.0": {"trajectory.json"},
    "1.3.0": {"collision.json", "configs.json"},
}


def members_of(version: str) -> set[str]:
    out: set[str] = set()
    for v in VERSIONS:
        out |= MEMBERS[v]
        if v == version:
            break
    return out


def reference_project(mesh_path: str | None = None):
    """The full (1.3.0) reference project, built in code and deterministic."""
    import core_bridge as cb  # noqa: F401
    from app.configs.store import ConfigStore
    from app.importers.visuals import LinkVisual
    from app.planning.collision import CollisionConfig, EnvShape
    from app.project.project_data import ProjectData, ProjectMetadata, SimulationSettings
    from app.sim.trajectory import Trajectory, Waypoint
    from robots import random_inertias, random_robot

    model = random_robot(7, 6)
    random_inertias(model, np.random.default_rng(5))
    meta = ProjectMetadata(
        name="fixture six dof", description="R8.4 reference",
        project_id="00000000-0000-4000-8000-0000000084aa",
        created_at="2026-11-12T00:00:00+00:00", modified_at="2026-11-12T00:00:00+00:00")
    mesh = str(mesh_path or MESH)
    p = ProjectData(
        metadata=meta,
        simulation_settings=SimulationSettings(
            integrator="rk45", physics_timestep=0.0005, gravity=[0.0, 0.0, -9.81],
            record_every=2),
        robot_model=model,
        physical_properties={"note": "reference", "masses": [1.0, 2.0]},
    )
    p.visuals = [
        LinkVisual(1, "l1", "mesh", {"filename": "cube.stl", "scale": [1, 1, 1]},
                   origin_xyz=[0.0, 0.0, 0.1], mesh_path=mesh),
        LinkVisual(2, "l2", "box", {"size": [0.1, 0.2, 0.3]}),
    ]
    p.collisions = [LinkVisual(1, "l1", "sphere", {"radius": 0.05})]
    qs = np.linspace(-0.5, 0.5, 18).reshape(3, 6)
    p.trajectory = Trajectory([Waypoint(t, q) for t, q in zip([0.0, 1.0, 2.5], qs)],
                              "cubic", True)
    cfg = CollisionConfig(fit_mode="sphere")
    cfg.set_check(0, 1, True)
    cfg.set_check(3, 1, False)
    cfg.environment.append(EnvShape("box", {"size": [0.2, 0.3, 0.4]}, [1, 2, 3],
                                    [0.1, 0.2, 0.3], "table"))
    p.collision = cfg
    p.configs = ConfigStore({"home": [0.0] * 6, "pose_a": [0.1, -0.2, 0.3, 0.0, 0.5, -0.6]})
    return p


def _rewrite(src: Path, dst: Path, keep, patch=None, stored=False) -> None:
    """Copy members of ``src`` to ``dst`` (fixed timestamps) for which keep(name) is true."""
    comp = zipfile.ZIP_STORED if stored else zipfile.ZIP_DEFLATED
    with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", comp) as zout:
        for info in zin.infolist():
            if not keep(info.filename):
                continue
            data = zin.read(info.filename)
            if patch:
                data = patch(info.filename, data)
            zi = zipfile.ZipInfo(info.filename, STAMP)
            zi.compress_type = comp
            zout.writestr(zi, data)


def _set_meta(**fields):
    def patch(name, data):
        if name != "project_metadata.json":
            return data
        d = json.loads(data)
        d.update(fields)
        return json.dumps(d, indent=2).encode()
    return patch


def main(out_dir: Path = HERE) -> None:
    from app.project.serializer import save

    out_dir = Path(out_dir)
    full = out_dir / "_full.robotproj"
    save(reference_project(), full)

    for v in VERSIONS:
        wanted = members_of(v)
        keep = (lambda n, w=wanted: n in w or (n.startswith("assets/") and "assets/" in w))
        _rewrite(full, out_dir / f"v{v}.robotproj", keep, _set_meta(format_version=v))

    cur = out_dir / "v1.3.0.robotproj"

    # ---- corrupt samples -------------------------------------------------
    (out_dir / "corrupt_not_a_zip.robotproj").write_bytes(b"this is not a zip archive\n" * 4)
    raw = cur.read_bytes()
    (out_dir / "corrupt_truncated.robotproj").write_bytes(raw[: len(raw) // 2])

    # bad CRC: stored (uncompressed) archive, one payload byte of robot_model.json flipped
    stored = out_dir / "_stored.robotproj"
    _rewrite(full, stored, lambda n: True, _set_meta(format_version="1.3.0"), stored=True)
    blob = bytearray(stored.read_bytes())
    with zipfile.ZipFile(stored) as zf:
        info = zf.getinfo("robot_model.json")
    off = info.header_offset + 30 + len(info.filename.encode()) + len(info.extra) + 20
    blob[off] ^= 0x01
    (out_dir / "corrupt_bad_crc.robotproj").write_bytes(bytes(blob))
    stored.unlink()

    _rewrite(full, out_dir / "corrupt_no_metadata.robotproj",
             lambda n: n != "project_metadata.json")

    def bad_json(name, data):
        if name == "simulation_settings.json":
            return b'{\n  "integrator": "rk4",\n  "physics_timestep": ,\n  "x": 1\n}'
        return _set_meta(format_version="1.3.0")(name, data)
    _rewrite(full, out_dir / "corrupt_bad_json.robotproj", lambda n: True, bad_json)

    def bad_schema(name, data):
        if name == "simulation_settings.json":
            d = json.loads(data)
            d["integrator"] = "bogus"
            return json.dumps(d).encode()
        return _set_meta(format_version="1.3.0")(name, data)
    _rewrite(full, out_dir / "corrupt_bad_schema.robotproj", lambda n: True, bad_schema)

    def no_name(name, data):
        if name == "project_metadata.json":
            d = json.loads(data)
            del d["name"]
            return json.dumps(d).encode()
        return data
    _rewrite(full, out_dir / "corrupt_no_name.robotproj", lambda n: True, no_name)

    def bad_robot(name, data):
        if name == "robot_model.json":
            return b'{"links": "not a list", "joints": 5}'
        return _set_meta(format_version="1.3.0")(name, data)
    _rewrite(full, out_dir / "corrupt_bad_robot.robotproj", lambda n: True, bad_robot)

    _rewrite(full, out_dir / "corrupt_missing_asset.robotproj",
             lambda n: not n.startswith("assets/"), _set_meta(format_version="1.3.0"))
    _rewrite(full, out_dir / "corrupt_unknown_version.robotproj", lambda n: True,
             _set_meta(format_version="2.0.0"))
    _rewrite(full, out_dir / "corrupt_newer_version.robotproj", lambda n: True,
             _set_meta(format_version="1.9.0"))
    full.unlink()
    print("fixtures written to", out_dir)


if __name__ == "__main__":
    main()
