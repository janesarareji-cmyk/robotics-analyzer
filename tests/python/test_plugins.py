"""R8.2: plugin manifest, loader, permissions, isolation, sample plugins.

Plugins live in tmp_path folders (never the real APPDATA). Permission dialogs are injected
callbacks; the default QMessageBox path is monkeypatched. Oracles: numpy interpolation, numpy FK
(`numpy_ref`), own file-system facts.
"""
import json
import shutil
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import core_bridge as cb

from app.plugins.host import PluginCallError, guarded_call
from app.plugins.loader import PluginManager
from app.plugins.manifest import ManifestError, api_compatible, parse_manifest

from .numpy_ref import fk_numpy, spec_from_model
from .test_cartesian_traj import planar_2r

EXAMPLES = Path(__file__).parent.parent.parent / "plugins_examples"


class DictSettings:
    def __init__(self):
        self.d = {}

    def get(self, k, default=None):
        return self.d.get(k, default)

    def set(self, k, v):
        self.d[k] = v


def manifest(name="p", kind="planner", entry="mod:register", perms=(), **over):
    m = {"name": name, "version": "1.0", "api_version": "1.0", "entry_point": entry,
         "kind": kind, "permissions": list(perms)}
    m.update(over)
    return m


def mk(root, name, code, kind="planner", entry="mod:register", perms=(), man=None):
    d = Path(root) / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "plugin.json").write_text(json.dumps(man or manifest(name, kind, entry, perms)))
    (d / "mod.py").write_text(code)
    return d


def mgr(root, settings=None, ask=None, **kw):
    kw.setdefault("load_timeout", 2.0)
    kw.setdefault("call_timeout", 2.0)
    return PluginManager(settings or DictSettings(), folder=str(root), ask=ask, **kw)


def status(m, name):
    return m.result(name).status


PLAN_OK = '''
from types import SimpleNamespace
import numpy as np
def plan(model, start_q, goal_q, collision=None, limits=None, **kw):
    s = np.asarray(start_q, float); g = np.asarray(goal_q, float)
    return SimpleNamespace(path=np.vstack([s, g]), success=True, time=0.0, iterations=1, reason="")
def register(host):
    host.register_planner("P", plan)
'''


def plan_code(body):
    return (
        "from types import SimpleNamespace\nimport numpy as np, time\n"
        "def plan(model, start_q, goal_q, collision=None, limits=None, **kw):\n"
        + "\n".join("    " + ln for ln in body.strip().splitlines())
        + "\ndef register(host):\n    host.register_planner('P', plan)\n")


def call_plan(m, name="p", start=(0.0, 0.0), goal=(0.5, -0.5), **kw):
    model = planar_2r()
    pp = m.plugin(name).planners[0]
    return pp(model, list(start), list(goal), limits=([-3.2, -3.2], [3.2, 3.2]), **kw)


# --------------------------------------------------------------------------- manifest
@pytest.mark.parametrize("over,snippet", [
    ({"bogus": 1}, "unknown key"),
    ({"kind": "exporter"}, "unknown kind"),
    ({"permissions": ["root"]}, "unknown permission"),
    ({"permissions": ["network", "network"]}, "duplicates"),
    ({"permissions": "file_read"}, "list of strings"),
    ({"api_version": "2.0"}, "not compatible"),
    ({"api_version": "1.99"}, "not compatible"),
    ({"api_version": "latest"}, "api_version"),
    ({"entry_point": "no_colon"}, "module:function"),
    ({"entry_point": "../x:f"}, "module:function"),
    ({"name": "bad name!"}, "name"),
    ({"version": "x"}, "version"),
])
def test_manifest_rejects(over, snippet):
    with pytest.raises(ManifestError) as ei:
        parse_manifest(manifest(**over))
    assert snippet in str(ei.value)


def test_manifest_missing_key_and_shapes():
    m = manifest()
    del m["entry_point"]
    with pytest.raises(ManifestError, match="missing key"):
        parse_manifest(m)
    with pytest.raises(ManifestError, match="object"):
        parse_manifest([1, 2])
    ok = parse_manifest(manifest(perms=["file_read", "gpu"]))
    assert ok.permissions == ("file_read", "gpu") and ok.module == "mod" and ok.function == "register"


def test_api_compat_rule():
    assert api_compatible("1.0", "1.0") and api_compatible("1.0", "1.3")
    assert not api_compatible("1.4", "1.3") and not api_compatible("2.0", "1.3")
    assert not api_compatible("x", "1.0")


def test_bad_manifest_files_rejected_others_survive(tmp_path):
    mk(tmp_path, "good", PLAN_OK, man=manifest("good"))
    d = tmp_path / "badjson"
    d.mkdir()
    (d / "plugin.json").write_text("{not json")
    mk(tmp_path, "badkind", PLAN_OK, man=manifest("badkind", kind="nope"))
    mk(tmp_path, "oldapi", PLAN_OK, man=manifest("oldapi", api_version="2.0"))
    m = mgr(tmp_path)
    m.scan()
    assert status(m, "good") == "ok"
    for n in ("badjson", "badkind", "oldapi"):
        r = m.result(n)
        assert r.status == "rejected" and r.message
    assert "JSON" in m.result("badjson").message
    assert [r.name for r in m.results] == ["badjson", "badkind", "good", "oldapi"]


def test_manifest_stage_runs_no_plugin_code(tmp_path):
    marker = tmp_path / "ran.txt"
    code = f"open(r'{marker}', 'w').write('x')\n" + PLAN_OK
    mk(tmp_path / "pl", "rej", code, man=manifest("rej", api_version="9.9"))
    s = DictSettings()
    s.set("plugins/enabled/off", "0")
    mk(tmp_path / "pl", "off", code, man=manifest("off"))
    m = mgr(tmp_path / "pl", s)
    m.scan()
    assert status(m, "rej") == "rejected" and status(m, "off") == "disabled"
    assert not marker.exists()


def test_duplicate_name_rejected(tmp_path):
    mk(tmp_path, "a", PLAN_OK, man=manifest("same"))
    mk(tmp_path, "b", PLAN_OK, man=manifest("same"))
    m = mgr(tmp_path)
    m.scan()
    assert [r.status for r in m.results] == ["ok", "rejected"]
    assert "duplicate" in m.results[1].message


def test_missing_folder_is_empty(tmp_path):
    m = mgr(tmp_path / "nope")
    assert m.scan() == []


# --------------------------------------------------------------------------- loading errors
def test_load_errors_isolated(tmp_path):
    mk(tmp_path, "good", PLAN_OK, man=manifest("good"))
    mk(tmp_path, "raise_import", "raise RuntimeError('boom at import')\n", man=manifest("raise_import"))
    mk(tmp_path, "exit_import", "import sys\nsys.exit(3)\n", man=manifest("exit_import"))
    mk(tmp_path, "no_func", "x = 1\n", man=manifest("no_func"))
    mk(tmp_path, "register_raises", "def register(host):\n    raise ValueError('bad register')\n",
       man=manifest("register_raises"))
    mk(tmp_path, "registers_nothing", "def register(host):\n    pass\n", man=manifest("registers_nothing"))
    mk(tmp_path, "wrong_kind", "def register(host):\n    host.register_importer('x', 'X (*.x)', lambda p: None)\n",
       man=manifest("wrong_kind"))
    d = mk(tmp_path, "no_module", "", man=manifest("no_module", entry="absent:register"))
    (d / "mod.py").unlink()
    m = mgr(tmp_path)
    m.scan()
    assert status(m, "good") == "ok"
    for n in ("raise_import", "exit_import", "no_func", "register_raises", "registers_nothing",
              "wrong_kind", "no_module"):
        assert status(m, n) == "error", n
    assert "boom at import" in m.result("raise_import").message
    assert "not found" in m.result("no_func").message
    assert "not found" in m.result("no_module").message
    assert "bad register" in m.result("register_raises").message
    assert "cannot register" in m.result("wrong_kind").message
    assert len(m.planners()) == 1


def test_hang_at_import_times_out(tmp_path):
    mk(tmp_path, "good", PLAN_OK, man=manifest("good"))
    mk(tmp_path, "hang", "import threading\nthreading.Event().wait(3)\n", man=manifest("hang"))
    m = mgr(tmp_path, load_timeout=0.3)
    t0 = time.monotonic()
    m.scan()
    assert time.monotonic() - t0 < 2.5
    assert status(m, "hang") == "error" and "timeout" in m.result("hang").message
    assert status(m, "good") == "ok"


def test_entry_point_outside_folder_is_error(tmp_path):
    (tmp_path / "outside.py").write_text(PLAN_OK)
    d = tmp_path / "pl" / "esc"
    d.mkdir(parents=True)
    (d / "plugin.json").write_text(json.dumps(manifest("esc", entry="outside:register")))
    m = mgr(tmp_path / "pl")
    m.scan()
    assert status(m, "esc") == "error"


def test_two_plugins_same_module_name_do_not_clash(tmp_path):
    for n, label in (("one", "One"), ("two", "Two")):
        mk(tmp_path, n, f"def register(host):\n    host.register_planner('{label}', lambda *a, **k: None)\n",
           man=manifest(n))
    m = mgr(tmp_path)
    m.scan()
    assert sorted(p.label for p in m.planners()) == ["One", "Two"]


# --------------------------------------------------------------------------- sample plugins
@pytest.fixture
def samples(tmp_path):
    root = tmp_path / "plugins"
    for d in EXAMPLES.iterdir():
        if d.is_dir():
            shutil.copytree(d, root / d.name, ignore=shutil.ignore_patterns("__pycache__"))
    return root


def test_sample_plugins_load(samples):
    m = mgr(samples, ask=lambda *a: True)
    m.scan()
    assert {r.name: r.status for r in m.results} == {
        "joint_info_panel": "ok", "linear_planner": "ok", "urm_json_importer": "ok"}
    assert [p.label for p in m.planners()] == ["Linear"]
    assert [i.label for i in m.importers()] == ["URM JSON"]
    assert [a.title for a in m.analysis_panels()] == ["Joint info"]


def test_sample_planner_works(samples):
    m = mgr(samples)
    m.scan()
    pp = m.planners()[0]
    model = planar_2r()
    start, goal = [0.1, -0.2], [1.1, 0.7]
    res = pp(model, start, goal, limits=([-3.2, -3.2], [3.2, 3.2]), time_limit=1.0, seed=0,
             planner="plugin:linear_planner:Linear", progress=lambda *a: None)
    s = np.linspace(0, 1, 20)[:, None]
    ref = np.array(start) + s * (np.array(goal) - np.array(start))     # numpy oracle
    assert res.success and res.path.shape == (20, 2)
    assert np.allclose(res.path, ref, atol=1e-12)
    assert res.planner == "Linear" and res.length == pytest.approx(
        np.sum(np.linalg.norm(np.diff(ref, axis=0), axis=1)))
    # a pose goal makes this plugin return a clean failure: not garbage, not disabled
    bad = pp(model, start, np.eye(4), limits=([-3.2, -3.2], [3.2, 3.2]))
    assert not bad.success and "joint vector" in bad.reason
    assert m.plugin("linear_planner").active


def test_sample_importer_works(samples, tmp_path):
    asked = []
    m = mgr(samples, ask=lambda p, perm, text: asked.append((p, perm)) or True)
    m.scan()
    model = planar_2r(0.8, 0.4)
    f = tmp_path / "robot.json"
    f.write_text(model.to_json())
    res = m.importers()[0].import_file(str(f))
    assert asked == [("urm_json_importer", "file_read")]
    assert res.model.dof() == 2 and res.model.validate() == []
    q = np.array([0.3, -0.9])
    ref = fk_numpy(spec_from_model(model), q)[1]
    got = fk_numpy(spec_from_model(res.model), q)[1]
    assert np.allclose(got, ref, atol=1e-12)
    # the permission is now remembered: no second dialog
    m.importers()[0].import_file(str(f))
    assert len(asked) == 1


def test_sample_analysis_panel(samples, qapp):
    proj = SimpleNamespace(name="p", file_path=None, robot_model=planar_2r(), visuals=[], collisions=[])
    holder = {"p": None}
    m = mgr(samples, project_provider=lambda: holder["p"])
    m.scan()
    pa = m.analysis_panels()[0]
    w = pa.build(m.project_view())
    from PySide6.QtWidgets import QLabel
    label = w.findChild(QLabel)
    assert label.text() == "No robot."
    holder["p"] = proj                                   # live view: refresh sees the new project
    from PySide6.QtWidgets import QPushButton
    w.findChild(QPushButton).click()
    assert label.text().startswith("2 DOF") and "j0" in label.text() and "3.142" in label.text()
    with pytest.raises(AttributeError):
        m.project_view().robot = 1                        # read-only


# --------------------------------------------------------------------------- isolation
def test_planner_raises_disables_for_session(tmp_path):
    mk(tmp_path, "p", plan_code("raise RuntimeError('kaboom')"))
    m = mgr(tmp_path)
    m.scan()
    seen = []
    m.changed.connect(lambda: seen.append(1))
    with pytest.raises(PluginCallError) as ei:
        call_plan(m)
    assert ei.value.reason == "raised" and "kaboom" in str(ei.value)
    r = m.result("p")
    assert r.status == "disabled" and "kaboom" in r.message and "session" in r.message
    assert seen and m.planners() == []
    with pytest.raises(PluginCallError) as ei2:                  # stays disabled
        m.plugin("p").planners[0](planar_2r(), [0, 0], [1, 1], limits=([-3, -3], [3, 3]))
    assert ei2.value.reason == "disabled"
    # an explicit re-enable (toggle) reloads it
    assert m.set_enabled("p", False) and m.set_enabled("p", True)
    assert status(m, "p") == "ok" and len(m.planners()) == 1


@pytest.mark.parametrize("body,snippet", [
    ("return 'not a plan result'", "expected a PlanResult"),
    ("return SimpleNamespace(path=[[0, 0, 0]], success=True)", "shape"),
    ("return SimpleNamespace(path=np.array([[0.0, 0.0], [np.nan, 0.0]]), success=True)", "NaN"),
    ("return SimpleNamespace(path=np.array([[0.1, 0.0], [0.5, -0.5]]), success=True)", "start"),
    ("return SimpleNamespace(path=np.array([[0.0, 0.0], [0.5, -0.4]]), success=True)", "end at goal"),
    ("return SimpleNamespace(path=np.array([[0.0, 0.0], [9.0, 0.0], [0.5, -0.5]]), success=True)", "limits"),
    ("return SimpleNamespace(path='abc', success=True)", "numeric"),
    ("return SimpleNamespace(path=None, success='yes')", "bool"),
])
def test_planner_garbage_detected(tmp_path, body, snippet):
    mk(tmp_path, "p", plan_code(body))
    m = mgr(tmp_path)
    m.scan()
    with pytest.raises(PluginCallError) as ei:
        call_plan(m)
    assert ei.value.reason == "garbage" and snippet in str(ei.value)
    assert status(m, "p") == "disabled" and m.planners() == []


def test_planner_valid_failure_is_not_garbage(tmp_path):
    mk(tmp_path, "p", plan_code("return SimpleNamespace(path=None, success=False, reason='no way')"))
    m = mgr(tmp_path)
    m.scan()
    res = call_plan(m)
    assert not res.success and res.reason == "no way" and status(m, "p") == "ok"


def test_planner_hang_times_out_and_is_disabled(tmp_path):
    mk(tmp_path, "p", plan_code("time.sleep(3)\nreturn None"))
    m = mgr(tmp_path)
    m.scan()
    pp = m.plugin("p").planners[0]
    pp.timeout = 0.3
    t0 = time.monotonic()
    with pytest.raises(PluginCallError) as ei:
        call_plan(m)
    assert time.monotonic() - t0 < 2.0
    assert ei.value.reason == "timeout" and "abandoned" in str(ei.value)
    assert status(m, "p") == "disabled"


def test_planner_cancel_gives_grace_then_gives_up(tmp_path):
    mk(tmp_path, "p", plan_code("time.sleep(3)\nreturn None"))
    m = mgr(tmp_path)
    m.scan()
    evt = threading.Event()
    evt.set()
    t0 = time.monotonic()
    with pytest.raises(PluginCallError) as ei:
        # long time_limit would allow 20 s: cancel ends the wait after the grace period
        m.plugin("p").call(lambda: time.sleep(3), cancel=evt, timeout=20)
    assert 1.5 < time.monotonic() - t0 < 3.0 and "cancel" in str(ei.value)


def test_planner_in_worker_thread(tmp_path):
    mk(tmp_path, "p", PLAN_OK)
    m = mgr(tmp_path)
    m.scan()
    out = {}
    t = threading.Thread(target=lambda: out.setdefault("r", call_plan(m)))
    t.start()
    t.join(5)
    assert out["r"].success and np.allclose(out["r"].path, [[0, 0], [0.5, -0.5]])


IMPORT_CODE = '''
import core_bridge as cb
def register(host):
    host.register_importer("X", "X files (*.x)", imp)
def imp(path):
    {body}
'''


def imp_plugin(tmp_path, body):
    mk(tmp_path, "p", IMPORT_CODE.replace("{body}", body), kind="importer")
    m = mgr(tmp_path)
    m.scan()
    return m


def test_importer_wrong_type_and_invalid_model_and_raise(tmp_path):
    for body, snippet in (
        ("return 42", "expected ImportResult"),
        ("return type('R', (), {'model': 'nope'})()", "expected RobotModel"),
        ("m = cb.RobotModel(); j = cb.Joint(); j.name = 'a'; m.joints.append(j); return m", "validate()"),
        ("raise OSError('cannot open')", "cannot open"),
    ):
        m = imp_plugin(tmp_path / body[:6].replace(" ", "_"), body)
        with pytest.raises(PluginCallError) as ei:
            m.importers()[0].import_file("whatever.x")
        assert snippet in str(ei.value), body
        assert status(m, "p") == "disabled" and m.importers() == []


def test_importer_valid_failure_result_passes(tmp_path):
    m = imp_plugin(tmp_path, "return type('R', (), {'model': None})()")
    res = m.importers()[0].import_file("a.x")
    assert res.model is None and status(m, "p") == "ok"


def test_importer_hang(tmp_path):
    m = imp_plugin(tmp_path, "import time; time.sleep(3)")
    imp = m.importers()[0]
    imp.timeout = 0.3
    with pytest.raises(PluginCallError) as ei:
        imp.import_file("a.x")
    assert ei.value.reason == "timeout" and status(m, "p") == "disabled"


def test_analysis_raises_or_wrong_type(tmp_path, qapp):
    code = ("def register(host):\n    host.register_analysis_panel('T', f)\n"
            "def f(view):\n    {b}\n")
    for n, b in (("a", "raise RuntimeError('panel boom')"), ("b", "return 'not a widget'")):
        root = tmp_path / n
        mk(root, "p", code.replace("{b}", b), kind="analysis")
        m = mgr(root)
        m.scan()
        with pytest.raises(PluginCallError):
            m.analysis_panels()[0].build(m.project_view())
        assert status(m, "p") == "disabled" and m.analysis_panels() == []


def test_guarded_call_basics():
    assert guarded_call(lambda a, b=1: a + b, (2,), {"b": 3}) == 5
    with pytest.raises(PluginCallError) as ei:
        guarded_call(lambda: 1 / 0)
    assert ei.value.reason == "raised" and "ZeroDivisionError" in str(ei.value)
    with pytest.raises(PluginCallError) as ei:
        guarded_call(lambda: (_ for _ in ()).throw(SystemExit(1)))
    assert ei.value.reason == "raised"


# --------------------------------------------------------------------------- permissions
def perm_plugin(tmp_path, perms, settings=None, ask=None, body="pass"):
    code = ("def register(host):\n    host.register_planner('P', lambda *a, **k: None)\n"
            "    global H\n    H = host\n")
    mk(tmp_path, "p", code, perms=perms)
    m = mgr(tmp_path, settings, ask)
    m.scan()
    return m, m.plugin("p").host


def test_permission_granted_asked_once_and_persisted(tmp_path):
    f = tmp_path / "pl" / "p" / "note.txt"
    s = DictSettings()
    calls = []
    m, host = perm_plugin(tmp_path / "pl", ["file_read"], s, lambda p, perm, text: calls.append(text) or True)
    f.write_text("hello")
    assert host.read_text(f) == "hello" and host.read_bytes(f) == b"hello"
    assert len(calls) == 1 and "not a sandbox" in calls[0] and "file_read" in calls[0]
    assert m.permissions.state("p", "file_read") == "granted"
    # a new manager with the same settings does not ask again
    m2, host2 = perm_plugin(tmp_path / "pl", ["file_read"], s, lambda *a: pytest.fail("asked again"))
    assert host2.read_text(f) == "hello"


def test_permission_denied_raises_inside_plugin_and_is_caught_by_host(tmp_path):
    asked = []
    code = ("def register(host):\n    host.register_planner('P', f)\n    global H\n    H = host\n"
            "def f(model, s, g, **kw):\n    return H.read_text(r'%s')\n")
    d = tmp_path / "pl"
    target = d / "p" / "x.txt"
    mk(d, "p", code % target, perms=["file_read"])
    target.write_text("data")
    m = mgr(d, ask=lambda *a: asked.append(1) or False)
    m.scan()
    with pytest.raises(PluginCallError) as ei:
        call_plan(m)
    assert ei.value.reason == "permission" and "denied" in str(ei.value)
    assert m.permissions.state("p", "file_read") == "denied"
    assert status(m, "p") == "ok"                                     # a denial does not disable
    with pytest.raises(PluginCallError):
        call_plan(m)
    assert asked == [1]                                               # denial is remembered
    host = m.plugin("p").host
    with pytest.raises(PermissionError):
        host.read_text(target)                                        # raw host call: PermissionError


def test_permission_not_declared_never_asks(tmp_path):
    m, host = perm_plugin(tmp_path, [], ask=lambda *a: pytest.fail("must not ask"))
    f = tmp_path / "p" / "n.txt"
    f.write_text("x")
    with pytest.raises(PermissionError, match="did not declare"):
        host.read_text(f)
    with pytest.raises(PermissionError, match="did not declare"):
        host.write_text(tmp_path / "p" / "data" / "o.txt", "x")
    assert not hasattr(host, "http_get") and not hasattr(host, "gpu_info")


def test_file_scope_enforced_even_when_granted(tmp_path):
    asked = []
    m, host = perm_plugin(tmp_path / "pl", ["file_read", "file_write"],
                          ask=lambda *a: asked.append(a[1]) or True)
    outside = tmp_path / "secret.txt"
    outside.write_text("s")
    with pytest.raises(PermissionError, match="outside"):
        host.read_text(outside)
    with pytest.raises(PermissionError, match="outside"):
        host.read_text(tmp_path / "pl" / "p" / ".." / ".." / "secret.txt")   # traversal
    assert asked == []                                  # out of scope: refused before any dialog
    # user picked file: readable (the app grants it for the import call only)
    host._allow_read(outside)
    assert host.read_text(outside) == "s"
    host._clear_allowed()
    with pytest.raises(PermissionError):
        host.read_text(outside)
    # write only inside <plugin>/data
    host.write_text(host.data_dir + "/o.txt", "out")
    assert (Path(host.data_dir) / "o.txt").read_text() == "out"
    with pytest.raises(PermissionError, match="outside"):
        host.write_text(tmp_path / "pl" / "p" / "mod.py", "evil")
    with pytest.raises(PermissionError, match="outside"):
        host.write_text(outside, "evil")
    assert outside.read_text() == "s" and "def register" in (tmp_path / "pl" / "p" / "mod.py").read_text()
    assert sorted(asked) == ["file_read", "file_write"]


def test_network_absent_without_declaration_and_asked_when_declared(tmp_path, monkeypatch):
    m, host = perm_plugin(tmp_path / "a", [])
    assert not hasattr(host, "http_get")
    with pytest.raises(AttributeError):
        host.http_get("http://x")
    import urllib.request

    class Resp:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def read(self):
            return b"payload"

    seen = []
    monkeypatch.setattr(urllib.request, "urlopen", lambda url, timeout=0: seen.append(url) or Resp())
    answers = iter([False, True])
    m2, host2 = perm_plugin(tmp_path / "b", ["network"], ask=lambda *a: next(answers))
    with pytest.raises(PermissionError, match="denied"):
        host2.http_get("http://example.invalid/a")
    assert seen == []                                               # nothing left the host
    m2.permissions.revoke("p", "network")                           # revoke -> asks again
    assert host2.http_get("http://example.invalid/a") == b"payload" and seen == ["http://example.invalid/a"]


def test_revoke_and_gpu(tmp_path):
    n = []
    m, host = perm_plugin(tmp_path, ["gpu"], ask=lambda *a: n.append(1) or True)
    info = host.gpu_info()
    assert isinstance(info, dict) and "available" in info and n == [1]
    host.gpu_info()
    assert n == [1]
    m.permissions.revoke("p")
    assert m.permissions.state("p", "gpu") == "ask"
    host.gpu_info()
    assert n == [1, 1]


def test_default_dialog_is_qmessagebox(tmp_path, qapp, monkeypatch):
    import app.plugins.permissions as pm
    texts = []
    yes = pm.QMessageBox.StandardButton.Yes

    def fake_question(parent, title, text, *a, **k):
        texts.append(text)
        return yes

    monkeypatch.setattr(pm.QMessageBox, "question", staticmethod(fake_question))
    s = DictSettings()
    m, host = perm_plugin(tmp_path / "pl", ["file_read"], s, ask=None)
    f = tmp_path / "pl" / "p" / "a.txt"
    f.write_text("z")
    assert host.read_text(f) == "z" and len(texts) == 1 and "cooperative guard" in texts[0]
    assert s.get("plugins/perm/p/file_read") == "granted"


def test_default_dialog_from_worker_thread(tmp_path, qapp, monkeypatch):
    import app.plugins.permissions as pm
    from PySide6.QtCore import QThread
    where = []
    yes = pm.QMessageBox.StandardButton.Yes
    monkeypatch.setattr(pm.QMessageBox, "question", staticmethod(
        lambda *a, **k: where.append(QThread.currentThread()) or yes))
    m, host = perm_plugin(tmp_path / "pl", ["file_read"], ask=None)
    f = tmp_path / "pl" / "p" / "a.txt"
    f.write_text("z")
    out = {}
    t = threading.Thread(target=lambda: out.setdefault("v", host.read_text(f)), daemon=True)
    t.start()
    deadline = time.monotonic() + 5
    while t.is_alive() and time.monotonic() < deadline:
        qapp.processEvents()
        t.join(0.01)
    assert out.get("v") == "z" and where == [qapp.thread()]       # dialog ran on the GUI thread


# --------------------------------------------------------------------------- enable / disable
def test_enable_disable_persists(tmp_path):
    marker = tmp_path / "ran.txt"
    root = tmp_path / "pl"
    mk(root, "p", f"open(r'{marker}', 'a').write('x')\n" + PLAN_OK)
    s = DictSettings()
    m = mgr(root, s)
    m.scan()
    assert status(m, "p") == "ok" and marker.read_text() == "x" and len(m.planners()) == 1
    assert m.set_enabled("p", False)
    assert status(m, "p") == "disabled" and m.planners() == [] and "settings" in m.result("p").message
    assert s.get("plugins/enabled/p") == "0"
    m2 = mgr(root, s)                                   # "restart"
    m2.scan()
    assert status(m2, "p") == "disabled" and m2.planners() == []
    assert marker.read_text() == "x"                    # disabled plugin code never ran again
    assert m2.set_enabled("p", True)
    assert status(m2, "p") == "ok" and len(m2.planners()) == 1
    assert not m2.set_enabled("unknown", True)


def test_plugin_never_blocks_app_on_unreadable_state(tmp_path):
    # a file where a plugin dir is expected, and a dir without manifest: ignored silently
    (tmp_path / "file.txt").write_text("x")
    (tmp_path / "emptydir").mkdir()
    m = mgr(tmp_path)
    assert m.scan() == []


def test_settings_default_folder(monkeypatch):
    from PySide6.QtCore import QSettings
    from app.settings import AppSettings
    org = "RoboticsAnalyzer_test_r82"
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", org)
    monkeypatch.setattr(AppSettings, "_APPLICATION", org)
    monkeypatch.setenv("APPDATA", r"C:\fake_appdata")
    s = AppSettings()
    qs = QSettings(org, org)
    qs.clear()
    assert s.plugins_folder().replace("/", "\\") == "C:\\fake_appdata\\" + org + "\\plugins"
    s.set_plugins_folder("D:/x/plugins")
    assert s.plugins_folder() == "D:/x/plugins"
    qs.clear()
    qs.sync()


# --------------------------------------------------------------------------- A1.3 (audit M9)
def test_slow_permission_dialog_does_not_time_out_register(tmp_path, monkeypatch):
    """Audit M9: time inside a permission dialog does not count against the load timeout."""
    import app.plugins.permissions as pm
    offset = [0.0]
    fake = lambda: time.monotonic() + offset[0]  # noqa: E731
    monkeypatch.setattr(pm, "clock", fake, raising=False)
    import app.plugins.host as ph
    monkeypatch.setattr(ph, "time", SimpleNamespace(monotonic=fake, sleep=time.sleep))

    def slow_ask(plugin, perm, text):
        offset[0] += 15.0                    # simulated 15 s user think time
        return True

    code = ("def register(host):\n    host.read_text(__file__)\n"
            "    host.register_planner('P', lambda *a, **k: None)\n")
    mk(tmp_path / "pl", "p", code, perms=["file_read"])
    m = mgr(tmp_path / "pl", ask=slow_ask, load_timeout=2.0)
    m.scan()
    assert status(m, "p") == "ok", m.result("p").message


def test_unanswered_ask_is_not_saved_as_denied(tmp_path, qapp, monkeypatch):
    """Audit M9: an unanswered dialog denies this call only; nothing is stored."""
    import app.plugins.permissions as pm
    monkeypatch.setattr(pm, "ASK_TIMEOUT", 0.2, raising=False)
    s = DictSettings()
    mgr_ = pm.PermissionManager(s)
    out = {}

    def work():
        try:
            mgr_.check("p", ["file_read"], "file_read")
        except PermissionError as exc:
            out["e"] = exc

    t = threading.Thread(target=work, daemon=True)
    t.start()
    t.join(3)                                  # GUI thread never answers
    assert "e" in out
    assert s.d == {} and mgr_.state("p", "file_read") == "ask"
    mgr_.shutdown()


def test_empty_plugins_folder_setting_uses_default(tmp_path, monkeypatch):
    """Audit M9 / plugins_folder: empty stored value -> default folder, not the cwd."""
    from PySide6.QtCore import QSettings
    from app.settings import AppSettings
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test")
    monkeypatch.setenv("APPDATA", str(tmp_path))
    raw = QSettings("RoboticsAnalyzer_test", "RoboticsAnalyzer_test")
    raw.clear()
    try:
        s = AppSettings()
        default = s.plugins_folder()
        for bad in ("", "   "):
            s.set_plugins_folder(bad)
            assert s.plugins_folder() == default and default.startswith(str(tmp_path))
        s.set(AppSettings.KEY_RECENT_MAX, "abc")
        assert s.recent_max() == 10
        m = PluginManager(s)
        assert m.folder == default
    finally:
        raw.clear()
        raw.sync()
