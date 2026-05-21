#!/usr/bin/env python3
"""Move block/line comments from src into readme/comments.md; keep brief option hints in code."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
OUT_MD = ROOT / "readme" / "comments.md"
SKIP_PARTS = ("test_", "__pycache__")
EXTS = {".py", ".yaml", ".yml", ".cpp"}


def rel(p: Path) -> str:
    return str(p.relative_to(ROOT))


def keep_inline(text: str) -> bool:
    t = text.strip()
    if not t or len(t) > 72:
        return False
    if t.endswith(".") and len(t) > 35:
        return False
    return bool(
        re.search(r"\|", t)
        or re.match(r"^(true|false|auto|null|on|off|\d)", t, re.I)
        or re.match(r"^[-\d]", t)
        or "→" in t
        or re.match(r"^(uses?|see |e\.g\.)", t, re.I)
    )


def add(sections: list, path: Path, anchor: str, text: str) -> None:
    text = text.strip()
    if text:
        sections.append((rel(path), anchor, text))


def strip_yaml(path: Path, sections: list) -> str:
    out: list[str] = []
    pending: list[str] = []
    key = "header"
    in_doc = False

    def flush():
        nonlocal pending, key
        if pending:
            add(sections, path, key, "\n".join(pending))
            pending = []

    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.rstrip()
        if re.match(r"^\s*#", line):
            pending.append(re.sub(r"^\s*#\s?", "", line).strip())
            continue
        if "#" in line:
            val, _, com = line.partition("#")
            com = com.strip()
            if com and not keep_inline(com):
                pending.append(com)
                line = val.rstrip()
        flush()
        m = re.match(r"^(\s*)([a-zA-Z_][\w]*)\s*:", line)
        if m:
            key = m.group(2)
        out.append(line)
    flush()
    return "\n".join(out).rstrip() + "\n"


def _extract_triple_quoted(text: str, path: Path, sections: list) -> str:
    q = '"""'
    while True:
        start = text.find(q)
        if start == -1:
            start = text.find("'''")
            q = "'''" if start != -1 else None
        if q is None or start == -1:
            break
        end = text.find(q, start + 3)
        if end == -1:
            break
        inner = text[start + 3 : end].strip()
        before = text[:start]
        anchor = "module docstring" if start < 80 and "def " not in before[-40:] else "docstring"
        m = re.search(r"(?:async def|def|class) (\w+)", before[-120:])
        if m:
            anchor = m.group(1)
        if inner:
            add(sections, path, anchor, inner)
        text = text[:start] + text[end + 3 :]
        q = '"""'
    return text


def strip_python(path: Path, sections: list) -> str:
    text = _extract_triple_quoted(path.read_text(encoding="utf-8"), path, sections)

    out: list[str] = []
    anchor = "top"
    for raw in text.splitlines():
        line = raw.rstrip()
        m = re.match(r"^(\s*)(?:async def|def|class) (\w+)", line)
        if m:
            anchor = m.group(2)

        if re.match(r"^\s*#", line):
            if line.lstrip().startswith("#!"):
                out.append(line)
                continue
            body = re.sub(r"^\s*#\s?", "", line).strip()
            if body and not body.startswith("type:") and "noqa" not in body:
                add(sections, path, anchor, body)
            continue

        if "#" in line:
            code, _, com = line.partition("#")
            com = com.strip()
            if com and not com.startswith("type:") and "noqa" not in com:
                if keep_inline(com):
                    out.append(line)
                else:
                    add(sections, path, anchor, com)
                    stripped_code = code.rstrip()
                    if stripped_code:
                        out.append(stripped_code)
                continue

        out.append(line)

    return "\n".join(out).rstrip() + "\n"


def strip_cpp(path: Path, sections: list) -> str:
    text = path.read_text(encoding="utf-8")
    anchor = "file"
    while "/*" in text:
        start = text.find("/*")
        end = text.find("*/", start + 2)
        if end == -1:
            break
        block = text[start + 2 : end].strip()
        if block:
            add(sections, path, anchor, block)
        text = text[:start] + text[end + 2 :]

    out: list[str] = []
    for raw in text.splitlines():
        line = raw.rstrip()
        if re.match(r"^\s*//", line):
            body = re.sub(r"^\s*//\s?", "", line).strip()
            if body:
                add(sections, path, anchor, body)
            continue
        if "//" in line:
            code, _, com = line.partition("//")
            com = com.strip()
            if com and keep_inline(com):
                out.append(line)
            elif com:
                add(sections, path, anchor, com)
                c = code.rstrip()
                if c:
                    out.append(c)
            else:
                out.append(line)
            continue
        out.append(line)
    return "\n".join(out).rstrip() + "\n"


def main() -> None:
    sections: list[tuple[str, str, str]] = []
    files = sorted(
        p
        for p in SRC.rglob("*")
        if p.suffix in EXTS and not any(s in p.parts for s in SKIP_PARTS)
    )
    for path in files:
        if path.suffix in (".yaml", ".yml"):
            new = strip_yaml(path, sections)
        elif path.suffix == ".py":
            new = strip_python(path, sections)
        elif path.suffix == ".cpp":
            new = strip_cpp(path, sections)
        else:
            continue
        path.write_text(new, encoding="utf-8")

    lines = [
        "# Comments archive",
        "",
        "Notes removed from `src/` to keep configs and scripts minimal.",
        "See `readme/TUNING.md` for tuning knobs; this file is the former inline prose.",
        "",
    ]
    cur = None
    for fpath, anchor, body in sections:
        if fpath != cur:
            cur = fpath
            lines.extend([f"## `{fpath}`", ""])
        lines.extend([f"### {anchor}", "", body, ""])
    OUT_MD.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
    print(f"Wrote {OUT_MD} ({len(sections)} sections, {len(files)} files)")


if __name__ == "__main__":
    main()
