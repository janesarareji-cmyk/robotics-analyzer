"""
tools/check_docs.py (R8.6): check the Markdown docs under docs/.

Checks, for every ``docs/**/*.md`` file:
  * every relative link ``[text](target)`` / ``![alt](target)`` / ``[ref]: target``
    points to an existing file (directories are not accepted as link targets);
  * every ``#anchor`` (same file or another ``.md``) matches a heading slug in the
    target file, using GitHub-style slugs (lower case, punctuation removed, spaces
    become ``-``, duplicates get ``-1``, ``-2``, ...);
  * every code fence (``` or ~~~) is closed.

External links (``scheme:...``) are not fetched; they are skipped.
Links inside code fences and inline code spans are ignored.

Exit code: 0 when clean, 1 when at least one problem was found.
Run from anywhere:  python tools/check_docs.py [docs_root]
"""
from __future__ import annotations

import re
import sys
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DOCS = ROOT / "docs"

FENCE_RE = re.compile(r"^\s{0,3}(`{3,}|~{3,})")
HEADING_RE = re.compile(r"^\s{0,3}(#{1,6})\s+(.*?)\s*#*\s*$")
INLINE_CODE_RE = re.compile(r"`+[^`]*`+")
LINK_RE = re.compile(r"!?\[[^\]]*\]\(\s*<?([^)\s>]+)>?(?:\s+\"[^\"]*\")?\s*\)")
REF_DEF_RE = re.compile(r"^\s{0,3}\[[^\]]+\]:\s*<?(\S+?)>?\s*$")
SCHEME_RE = re.compile(r"^[a-zA-Z][a-zA-Z0-9+.-]*:")


def slugify(text: str) -> str:
    """GitHub-style heading slug (without the duplicate counter)."""
    text = INLINE_CODE_RE.sub(lambda m: m.group(0).strip("`"), text)
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)  # keep link text only
    text = text.strip().lower()
    text = re.sub(r"[^\w\- ]", "", text)  # drop punctuation (unicode word chars kept)
    return text.replace(" ", "-")


def parse_file(path: Path) -> tuple[list[tuple[int, str]], list[str], list[str]]:
    """
    Return (links, anchors, errors) for one file.
    links: (line_no, target) outside code fences.
    anchors: heading slugs with GitHub duplicate suffixes.
    errors: unclosed-fence messages.
    """
    links: list[tuple[int, str]] = []
    anchors: list[str] = []
    seen: dict[str, int] = {}
    errors: list[str] = []
    fence: str | None = None
    fence_line = 0
    lines = path.read_text(encoding="utf-8").splitlines()
    for no, line in enumerate(lines, start=1):
        m = FENCE_RE.match(line)
        if m:
            marker = m.group(1)
            if fence is None:
                fence, fence_line = marker, no
            elif marker[0] == fence[0] and len(marker) >= len(fence) and line.strip() == marker:
                fence = None
            continue
        if fence is not None:
            continue
        h = HEADING_RE.match(line)
        if h:
            slug = slugify(h.group(2))
            n = seen.get(slug, 0)
            anchors.append(slug if n == 0 else f"{slug}-{n}")
            seen[slug] = n + 1
        stripped = INLINE_CODE_RE.sub("", line)
        for lm in LINK_RE.finditer(stripped):
            links.append((no, lm.group(1)))
        rm = REF_DEF_RE.match(line)
        if rm:
            links.append((no, rm.group(1)))
    if fence is not None:
        errors.append(f"{path}:{fence_line}: code fence opened here is never closed")
    return links, anchors, errors


def check(docs_root: Path = DEFAULT_DOCS) -> list[str]:
    """Return a list of problem strings (empty when the docs are clean)."""
    problems: list[str] = []
    files = sorted(p for p in docs_root.rglob("*.md") if p.is_file())
    parsed: dict[Path, tuple[list[tuple[int, str]], list[str]]] = {}
    for f in files:
        links, anchors, errs = parse_file(f)
        problems.extend(errs)
        parsed[f.resolve()] = (links, anchors)

    for f in files:
        links, _ = parsed[f.resolve()]
        for line_no, raw in links:
            target = unquote(raw)
            if not target or SCHEME_RE.match(target) or target.startswith("//"):
                continue  # external or empty
            path_part, _, anchor = target.partition("#")
            if path_part:
                dest = (f.parent / path_part).resolve()
                if not dest.exists():
                    problems.append(f"{f}:{line_no}: broken link {raw!r} (no such file)")
                    continue
                if dest.is_dir():
                    problems.append(f"{f}:{line_no}: link {raw!r} points to a directory")
                    continue
            else:
                dest = f.resolve()
            if anchor:
                if dest.suffix.lower() != ".md":
                    continue  # anchors into non-markdown files are not checked
                if dest not in parsed:
                    _, anchors_other, _ = parse_file(dest)
                    parsed[dest] = ([], anchors_other)
                anchors = parsed[dest][1]
                if anchor not in anchors:
                    problems.append(f"{f}:{line_no}: broken anchor {raw!r} (no heading '{anchor}')")
    return problems


def main(argv: list[str]) -> int:
    docs_root = Path(argv[1]).resolve() if len(argv) > 1 else DEFAULT_DOCS
    if not docs_root.is_dir():
        print(f"check_docs: not a directory: {docs_root}", file=sys.stderr)
        return 1
    problems = check(docs_root)
    for p in problems:
        print(p)
    count = len(list(docs_root.rglob("*.md")))
    if problems:
        print(f"check_docs: {len(problems)} problem(s) in {count} file(s)")
        return 1
    print(f"check_docs: OK ({count} file(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
