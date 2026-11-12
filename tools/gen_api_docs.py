"""
tools/gen_api_docs.py (R8.6): generate docs/api-reference.md for robotics_api.

Output = tools/api_docs/intro.md (with the {API_VERSION} token replaced)
       + one section per public name, rendered from the docstrings and signatures
       + tools/api_docs/tail.md (script runner, examples, cancel, error notes).

Usage (any folder):
    python tools/gen_api_docs.py            write docs/api-reference.md
    python tools/gen_api_docs.py --check    exit 1 when the checked-in file is stale

Rules enforced at generation time (exit 1 with a message):
  * every name in robotics_api.__all__ is listed in GROUPS exactly once;
  * every public name and every public member has a docstring.
"""
from __future__ import annotations

import inspect
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "api-reference.md"
INTRO = ROOT / "tools" / "api_docs" / "intro.md"
TAIL = ROOT / "tools" / "api_docs" / "tail.md"

sys.path.insert(0, str(ROOT / "bindings"))
sys.path.insert(0, str(ROOT))

# Section order and membership. Every __all__ name must appear exactly once.
GROUPS: list[tuple[str, list[str]]] = [
    ("Project and files", ["new_project", "open_project", "save_project", "import_urdf", "Project"]),
    ("Robot", ["Robot"]),
    ("Motion", ["Trajectory", "time_parametrize", "plan", "simulate"]),
    ("Analysis and export", ["workspace", "export_csv", "export_npz", "export_urdf", "generate_report"]),
    ("Cancel", ["check_cancel", "is_cancelled", "CancelToken", "cancel_scope"]),
    ("Errors", ["RoboticsApiError", "InvalidArgument", "ProjectError", "ImportFailed",
                "ComputationError", "SimulationError", "PlanningError", "AnalysisError",
                "ExportError", "ScriptCancelled", "ScriptTimeLimit"]),
]

# Short examples. tests/python/test_docs.py checks that the example files run.
EXAMPLES: dict[str, str] = {
    "new_project": 'project = api.new_project("Demo")',
    "open_project": 'project = api.open_project("cell.robotproj")\nprint(project.name, project.robot.n_dof)',
    "save_project": 'api.save_project(project, "cell.robotproj")',
    "import_urdf": 'robot = api.import_urdf("examples/planar_2r.urdf")\nprint(robot.n_dof, robot.joint_names)',
    "Robot": 'T = robot.fk([0.3, -0.4])          # 4x4 TCP pose\nJ = robot.jacobian([0.3, -0.4])  # (6, n_dof)',
    "Trajectory": 'traj = api.Trajectory([0.0, 1.0], [[0.0, 0.0], [0.5, -0.5]], interpolation="cubic")\nprint(traj.q(0.5))',
    "time_parametrize": 'traj = api.time_parametrize(path, robot)\nprint(traj.duration, traj.ok)',
    "plan": 'path = api.plan(robot, [0.0, 0.0], [2.2, 0.3], planner="rrt_connect", seed=1)\nprint(len(path))',
    "simulate": ('res = api.simulate(robot, None, duration=1.0, dt=0.001, q0=[0.8, 0.0],\n'
                 '                   gravity=[0.0, -9.81, 0.0], record_every=10)\nprint(res["total"][-1])'),
    "workspace": 'ws = api.workspace(robot, n_samples=100000, voxel=0.05)\nprint(ws["volume"])',
    "export_csv": 'api.export_csv(res, "swing.csv", names=["q", "tcp_pos"])',
    "export_npz": 'api.export_npz(res, "swing.npz")',
    "export_urdf": 'info = api.export_urdf(robot, "out/robot.urdf")\nprint(info["warnings"])',
    "generate_report": 'info = api.generate_report(robot, "report.pdf", result=res)\nprint(info["pages"])',
    "check_cancel": 'for i in range(10**6):\n    api.check_cancel()   # raises ScriptCancelled when asked to stop',
}

ERROR_BASE_NOTE = {"ScriptTimeLimit": "ScriptCancelled", "ScriptCancelled": "RoboticsApiError"}


def _ann(a) -> str:
    if a is inspect.Parameter.empty:
        return ""
    return a.strip("'\"") if isinstance(a, str) else getattr(a, "__name__", repr(a))


def _clean_sig(obj, skip_first: bool = False) -> str:
    """Signature as written in the code: string annotations unquoted, defaults via repr."""
    sig = inspect.signature(obj)
    params = list(sig.parameters.values())
    if skip_first and params:
        params = params[1:]
    parts = []
    for p in params:
        text = p.name
        if p.annotation is not inspect.Parameter.empty:
            text += f": {_ann(p.annotation)}"
        if p.default is not inspect.Parameter.empty:
            text += f" = {p.default!r}"
        parts.append(text)
    out = f"({', '.join(parts)})"
    if sig.return_annotation is not inspect.Signature.empty:
        out += f" -> {_ann(sig.return_annotation)}"
    return out


def _doc(obj, where: str) -> str:
    doc = inspect.getdoc(obj)
    if not doc:
        raise SystemExit(f"gen_api_docs: no docstring for {where}")
    return doc


def _example(name: str) -> list[str]:
    if name not in EXAMPLES:
        return []
    return ["Example:", "", "```python", EXAMPLES[name], "```", ""]


def _render_function(name: str, obj) -> list[str]:
    return ([f"### `{name}`", "", "```python", f"{name}{_clean_sig(obj)}", "```", "",
             _doc(obj, name), ""] + _example(name))


def _render_class(name: str, cls) -> list[str]:
    lines = [f"### `{name}`", "", _doc(cls, name), ""] + _example(name)
    members = [(m, v) for m, v in vars(cls).items() if not m.startswith("_")]
    if members:
        lines += ["#### Members", ""]
    for mname, member in members:
        if isinstance(member, property):
            lines += [f"#### `{name}.{mname}`", "", "*(property)*", "",
                      _doc(member, f"{name}.{mname}"), ""]
        elif isinstance(member, (classmethod, staticmethod)):
            func = member.__func__
            skip = isinstance(member, classmethod)
            lines += [f"#### `{name}.{mname}`", "", "```python",
                      f"{mname}{_clean_sig(func, skip_first=skip)}", "```", "",
                      _doc(func, f"{name}.{mname}"), ""]
        elif callable(member):
            lines += [f"#### `{name}.{mname}`", "", "```python",
                      f"{mname}{_clean_sig(member, skip_first=True)}", "```", "",
                      _doc(member, f"{name}.{mname}"), ""]
    return lines


def _render_error(name: str, cls) -> list[str]:
    base = ERROR_BASE_NOTE.get(name) or ", ".join(b.__name__ for b in cls.__bases__)
    return [f"### `{name}`", "", f"Subclass of `{base}`.", "", _doc(cls, name), ""]


def render() -> str:
    import robotics_api as api

    listed = [n for _, names in GROUPS for n in names]
    missing = [n for n in api.__all__ if n not in listed and n != "__api_version__"]
    extra = [n for n in listed if n not in api.__all__]
    dupes = sorted({n for n in listed if listed.count(n) > 1})
    if missing or extra or dupes:
        raise SystemExit(f"gen_api_docs: GROUPS mismatch: missing={missing} extra={extra} dupes={dupes}")

    intro = INTRO.read_text(encoding="utf-8").replace("{API_VERSION}", str(api.__api_version__))
    out: list[str] = [intro.rstrip("\n"), ""]
    for title, names in GROUPS:
        out += [f"## {title}", ""]
        for name in names:
            obj = getattr(api, name)
            if inspect.isclass(obj) and issubclass(obj, BaseException):
                out += _render_error(name, obj)
            elif inspect.isclass(obj):
                out += _render_class(name, obj)
            else:
                out += _render_function(name, obj)
    out += [TAIL.read_text(encoding="utf-8").rstrip("\n"), ""]
    return "\n".join(out)


def main(argv: list[str]) -> int:
    text = render()
    if "--check" in argv:
        current = OUT.read_text(encoding="utf-8") if OUT.exists() else ""
        if current.replace("\r\n", "\n") != text:
            print(f"gen_api_docs: {OUT} is stale; run: python tools/gen_api_docs.py")
            return 1
        print("gen_api_docs: up to date")
        return 0
    OUT.write_text(text, encoding="utf-8", newline="\n")
    print(f"gen_api_docs: wrote {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
