"""S1/S2 (security audit 2026-11-15): asset names and mesh paths from untrusted projects."""
from __future__ import annotations

import json
import os
import zipfile
from pathlib import Path

import pytest

from app.importers import visuals as vis
from app.importers.visuals import LinkVisual
from app.project.project_data import ProjectData
from app.project.serializer import load, save


def _visual(mesh_path, fallback=False, kind="mesh"):
    return LinkVisual(link_index=0, link_name="l0", kind=kind,
                      params={"filename": "x"}, mesh_path=mesh_path, fallback=fallback)


def _project(tmp_path, visuals):
    p = ProjectData.new("p", "d")
    p.visuals = visuals
    out = tmp_path / "a.robotproj"
    save(p, out)
    return out


def _add_members(path, members):
    with zipfile.ZipFile(path, "a") as zf:
        for n, b in members.items():
            zf.writestr(zipfile.ZipInfo(n), b)


def _tree(root: Path):
    return sorted(str(p.relative_to(root)) for p in root.rglob("*"))


# ---------------------------------------------------------------- S1
BAD_NAMES = ["D:numpy.py", "C:x.txt", "..\\x.py", "a:b", "a:b.stl", "CON", "con.stl", "NUL.txt",
             "a.", "x" * 200, "sub/x.stl", "a b.stl", "x.stl:stream"]


@pytest.mark.parametrize("bad", BAD_NAMES)
def test_bad_asset_names_write_nothing_outside(tmp_path, monkeypatch, bad):
    cwd = tmp_path / "cwd"
    cwd.mkdir()
    out = _project(tmp_path, [_visual(None, fallback=True)])
    _add_members(out, {"assets/" + bad: b"evil", "assets/good.stl": b"solid x\nendsolid x\n"})
    before = _tree(tmp_path)
    monkeypatch.chdir(cwd)
    data = load(out)
    assert _tree(cwd) == []
    assert _tree(tmp_path) == before
    assert [p.name for p in Path(data.asset_dir).iterdir()] == ["good.stl"]
    data.cleanup_assets()


def test_normal_names_load_and_round_trip(tmp_path):
    mesh = tmp_path / "my_arm-1.v2.stl"
    mesh.write_bytes(b"solid x\nendsolid x\n")
    out = _project(tmp_path, [_visual(str(mesh))])
    data = load(out)
    v = data.visuals[0]
    assert v.mesh_path and Path(v.mesh_path).name == "my_arm-1.v2.stl"
    assert Path(v.mesh_path).read_bytes() == mesh.read_bytes()
    assert not v.fallback
    out2 = tmp_path / "b.robotproj"
    save(data, out2)
    assert Path(load(out2).visuals[0].mesh_path).name == "my_arm-1.v2.stl"


def test_save_sanitises_odd_file_names(tmp_path):
    mesh = tmp_path / "my part (1).stl"
    mesh.write_bytes(b"solid x\nendsolid x\n")
    out = _project(tmp_path, [_visual(str(mesh))])
    with zipfile.ZipFile(out) as zf:
        assets = [n for n in zf.namelist() if n.startswith("assets/")]
    assert assets == ["assets/my_part__1_.stl"]
    v = load(out).visuals[0]
    assert v.mesh_path and Path(v.mesh_path).is_file()


def test_reserved_name_on_save_is_renamed(tmp_path):
    mesh = tmp_path / "nul.stl"
    mesh.write_bytes(b"solid x\nendsolid x\n")
    out = _project(tmp_path, [_visual(str(mesh))])
    v = load(out).visuals[0]
    assert v.mesh_path and Path(v.mesh_path).is_file()


def test_unsafe_asset_reference_becomes_fallback(tmp_path):
    out = _project(tmp_path, [_visual("assets/D:numpy.py")])
    _add_members(out, {"assets/D:numpy.py": b"x"})
    v = load(out).visuals[0]
    assert v.fallback and v.mesh_path is None


def test_is_safe_asset_name_unit():
    assert vis.is_safe_asset_name("a-b_c.1.STL")
    for bad in BAD_NAMES + [".", ".."]:
        assert not vis.is_safe_asset_name(bad), bad


# ---------------------------------------------------------------- S2
@pytest.mark.parametrize("p", ["\\\\host\\share\\m.stl", "//host/share/m.stl", "file://host/s/m.stl",
                               "\\\\?\\C:\\x.stl", "\\\\.\\pipe\\x", "C:\\a\\m.stl:ads"])
def test_remote_paths_detected(p):
    assert vis.is_remote_or_device_path(p)


@pytest.mark.parametrize("p", ["C:\\a\\m.stl", "/home/u/m.stl", "rel/m.stl", "file:///C:/a/m.stl",
                               "file://localhost/C:/a/m.stl"])
def test_local_paths_ok(p):
    assert not vis.is_remote_or_device_path(p)


def test_open_never_stats_outside_assets(tmp_path, monkeypatch):
    foreign = tmp_path / "foreign.stl"
    foreign.write_bytes(b"solid x\nendsolid x\n")
    out = _project(tmp_path, [_visual(None, fallback=True)])
    with zipfile.ZipFile(out) as zf:
        items = {n: zf.read(n) for n in zf.namelist()}
    vj = json.loads(items["visuals.json"])
    unc = "\\\\host\\share\\m.stl"
    vj["visuals"] = [
        {"link_index": 0, "link_name": "a", "kind": "mesh", "mesh_path": unc},
        {"link_index": 1, "link_name": "b", "kind": "mesh", "mesh_path": str(foreign)},
    ]
    items["visuals.json"] = json.dumps(vj).encode()
    with zipfile.ZipFile(out, "w") as zf:
        for n, b in items.items():
            zf.writestr(n, b)
    seen = []
    real_isfile = os.path.isfile
    monkeypatch.setattr(os.path, "isfile", lambda p: (seen.append(str(p)), real_isfile(p))[1])
    data = load(out)
    assert all(v.fallback and v.mesh_path is None for v in data.visuals)
    assert not any("host" in s for s in seen)
    # a UNC mesh_path set in memory is never stat'ed either: placeholder cube
    seen.clear()
    assert vis.load_visual_mesh(_visual(unc), 0.1) is not None
    assert not any("host" in s for s in seen)


def test_save_does_not_embed_secret_or_unc(tmp_path):
    secret = tmp_path / "id_ed25519"
    secret.write_text("PRIVATE KEY")
    txt = tmp_path / "notes.txt"
    txt.write_text("x")
    dirpath = tmp_path / "dir.stl"
    dirpath.mkdir()
    out = _project(tmp_path, [_visual(str(secret)), _visual(str(txt)), _visual(str(dirpath)),
                              _visual("\\\\host\\s\\m.stl")])
    with zipfile.ZipFile(out) as zf:
        assert [n for n in zf.namelist() if n.startswith("assets/")] == []
        assert b"PRIVATE KEY" not in b"".join(zf.read(n) for n in zf.namelist())
    data = load(out)
    assert all(v.fallback and v.mesh_path is None for v in data.visuals)


def test_urdf_unc_and_file_host_not_resolved(tmp_path, monkeypatch):
    real_is_file = Path.is_file
    seen = []
    monkeypatch.setattr(Path, "is_file", lambda self: (seen.append(str(self)), real_is_file(self))[1])
    for fn in ("\\\\host\\share\\m.stl", "file://host/share/m.stl"):
        assert vis.resolve_mesh(fn, tmp_path) is None
    assert not any("host" in s for s in seen)


def test_urdf_relative_mesh_still_resolves(tmp_path):
    (tmp_path / "meshes").mkdir()
    m = tmp_path / "meshes" / "a.stl"
    m.write_bytes(b"solid x\nendsolid x\n")
    assert vis.resolve_mesh("meshes/a.stl", tmp_path) == str(m.resolve())
    assert vis.resolve_mesh("file:///" + m.as_posix(), tmp_path) == str(m.resolve())
