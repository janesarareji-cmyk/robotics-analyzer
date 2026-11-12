"""
tests/python/test_docs.py (R8.6): documentation checks.

- docs/**/*.md: no broken relative links, anchors or unclosed code fences (tools/check_docs.py).
- The checker itself reports a broken link (so a clean run means something).
- Every public name in robotics_api.__all__ (and every public member of a class) has a docstring.
- docs/api-reference.md is the current output of tools/gen_api_docs.py.
- docs/index.md links every docs/*.md and docs/schema/*.md file.
- The three examples run from a shell (the API examples in the reference stay true).
"""
import importlib.util
import inspect
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent.parent
DOCS = ROOT / "docs"
TOOLS = ROOT / "tools"
LINK_RE = re.compile(r"\[[^\]]*\]\(\s*([^)\s]+)")


def _run(args, cwd=ROOT, env=None):
    return subprocess.run([sys.executable, *args], cwd=str(cwd), capture_output=True,
                          text=True, timeout=600, env=env)


def _load_check_docs():
    spec = importlib.util.spec_from_file_location("check_docs", TOOLS / "check_docs.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_no_broken_links_or_fences():
    res = _run([str(TOOLS / "check_docs.py")])
    assert res.returncode == 0, res.stdout + res.stderr


def test_checker_reports_broken_link_and_anchor(tmp_path):
    check_docs = _load_check_docs()
    docs = tmp_path / "docs"
    docs.mkdir()
    (docs / "a.md").write_text(
        "# Title\n\n[missing](nope.md)\n\n[bad anchor](b.md#no-such-heading)\n\n"
        "[ok](b.md#real-heading)\n\n```\nunclosed fence\n", encoding="utf-8")
    (docs / "b.md").write_text("## Real heading\n", encoding="utf-8")
    problems = check_docs.check(docs)
    text = "\n".join(problems)
    assert "nope.md" in text
    assert "no-such-heading" in text
    assert "never closed" in text
    assert "real-heading" not in text


def test_checker_cli_exit_codes(tmp_path):
    bad = tmp_path / "docs"
    bad.mkdir()
    (bad / "x.md").write_text("[l](gone.md)\n", encoding="utf-8")
    assert _run([str(TOOLS / "check_docs.py"), str(bad)]).returncode == 1
    good = tmp_path / "good"
    good.mkdir()
    (good / "x.md").write_text("# Hi\n", encoding="utf-8")
    assert _run([str(TOOLS / "check_docs.py"), str(good)]).returncode == 0


def _api():
    sys.path.insert(0, str(ROOT / "bindings"))
    sys.path.insert(0, str(ROOT))
    import robotics_api
    return robotics_api


def test_every_public_name_has_docstring():
    api = _api()
    missing = []
    for name in api.__all__:
        if name == "__api_version__":
            continue
        obj = getattr(api, name)
        if not inspect.getdoc(obj):
            missing.append(name)
        if inspect.isclass(obj) and not issubclass(obj, BaseException):
            for mname, member in vars(obj).items():
                if mname.startswith("_"):
                    continue
                func = member.fget if isinstance(member, property) else getattr(member, "__func__", member)
                if callable(func) or isinstance(member, property):
                    if not inspect.getdoc(func):
                        missing.append(f"{name}.{mname}")
    assert not missing, f"no docstring: {missing}"


def test_api_reference_is_generated_and_current():
    res = _run([str(TOOLS / "gen_api_docs.py"), "--check"])
    assert res.returncode == 0, res.stdout + res.stderr


def test_api_reference_names_every_public_name():
    api = _api()
    text = (DOCS / "api-reference.md").read_text(encoding="utf-8")
    for name in api.__all__:
        if name == "__api_version__":
            continue
        assert f"`{name}`" in text, name


def _index_targets():
    index = DOCS / "index.md"
    targets = set()
    for raw in LINK_RE.findall(index.read_text(encoding="utf-8")):
        path = raw.partition("#")[0]
        if path:
            targets.add((index.parent / path).resolve())
    return targets


def test_index_lists_every_doc():
    targets = _index_targets()
    required = [p for p in DOCS.glob("*.md") if p.name != "index.md"]
    required += list((DOCS / "schema").glob("*.md"))
    missing = [p.relative_to(ROOT).as_posix() for p in required if p.resolve() not in targets]
    assert not missing, f"docs/index.md does not list: {missing}"


def test_index_lists_every_spec():
    targets = _index_targets()
    specs = list((DOCS / "specs").glob("*.md"))
    missing = [p.name for p in specs if p.resolve() not in targets]
    assert not missing, f"docs/index.md does not list specs: {missing}"


def test_examples_run():
    env = dict(os.environ)
    sep = os.pathsep
    env["PYTHONPATH"] = sep.join([str(ROOT), str(ROOT / "bindings")])
    env["QT_QPA_PLATFORM"] = "offscreen"
    for example in ("fk_sweep.py", "plan_and_simulate.py", "export_data.py"):
        res = _run([str(ROOT / "examples" / example)], cwd=ROOT / "examples", env=env)
        assert res.returncode == 0, f"{example}: {res.stdout}\n{res.stderr}"
