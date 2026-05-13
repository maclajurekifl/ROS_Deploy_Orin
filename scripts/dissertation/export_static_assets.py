#!/usr/bin/env python3
"""Generate static dissertation figure assets from workspace source files.

Outputs are written under:
  <workspace>/docs/dissertation_assets/static/
"""

from __future__ import annotations

import os
import pathlib
import xml.etree.ElementTree as ET
from datetime import datetime


WORKSPACE = pathlib.Path(__file__).resolve().parents[2]
SRC = WORKSPACE / "src"
OUT = WORKSPACE / "docs" / "dissertation_assets" / "static"


def _iter_ros_packages(src_dir: pathlib.Path) -> list[pathlib.Path]:
    return sorted(p.parent for p in src_dir.rglob("package.xml"))


def _workspace_tree(max_depth: int = 3) -> str:
    lines: list[str] = [f"{WORKSPACE.name}/"]

    def walk(path: pathlib.Path, prefix: str, depth: int) -> None:
        if depth > max_depth:
            return
        children = sorted(
            [c for c in path.iterdir() if c.name not in {".git", "build", "install", "log", "__pycache__"}],
            key=lambda p: (not p.is_dir(), p.name.lower()),
        )
        for idx, child in enumerate(children):
            last = idx == len(children) - 1
            branch = "└── " if last else "├── "
            lines.append(f"{prefix}{branch}{child.name}{'/' if child.is_dir() else ''}")
            if child.is_dir():
                walk(child, prefix + ("    " if last else "│   "), depth + 1)

    walk(WORKSPACE, "", 1)
    return "\n".join(lines) + "\n"


def _read_package_info(pkg_xml: pathlib.Path) -> tuple[str, set[str]]:
    root = ET.parse(pkg_xml).getroot()
    name = root.findtext("name", "").strip()
    deps: set[str] = set()
    dep_tags = (
        "depend",
        "build_depend",
        "build_export_depend",
        "exec_depend",
        "test_depend",
    )
    for tag in dep_tags:
        for elem in root.findall(tag):
            if elem.text:
                deps.add(elem.text.strip())
    return name, deps


def _package_dependency_dot() -> str:
    pkg_dirs = _iter_ros_packages(SRC)
    local_pkgs: dict[str, set[str]] = {}
    for pkg_dir in pkg_dirs:
        name, deps = _read_package_info(pkg_dir / "package.xml")
        if name:
            local_pkgs[name] = deps

    lines: list[str] = [
        "digraph ros_workspace_packages {",
        '  rankdir=LR;',
        '  graph [fontname="Helvetica"];',
        '  node [shape=box, style="rounded,filled", color="#2D3748", fillcolor="#EBF8FF", fontname="Helvetica"];',
        '  edge [color="#4A5568"];',
    ]

    for pkg_name in sorted(local_pkgs):
        lines.append(f'  "{pkg_name}";')

    for pkg_name, deps in sorted(local_pkgs.items()):
        for dep in sorted(deps):
            if dep in local_pkgs:
                lines.append(f'  "{pkg_name}" -> "{dep}";')

    lines.append("}")
    return "\n".join(lines) + "\n"


def _config_merge_mermaid() -> str:
    return """flowchart TD
  A[slam_bringup.yaml defaults] --> B[bringup_config overlay YAML]
  B --> C[launch arguments]
  C --> D[launch_slam.launch.py merged dict]
  D --> E1[ekf_node parameters]
  D --> E2[lidar_odometry parameters]
  D --> E3[keyframe_map/pose_graph parameters]
  D --> E4[driver + static TF parameters]
"""


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    now = datetime.now().isoformat(timespec="seconds")

    (OUT / "workspace_tree.txt").write_text(_workspace_tree(), encoding="utf-8")
    (OUT / "package_dependency_graph.dot").write_text(_package_dependency_dot(), encoding="utf-8")
    (OUT / "launch_config_merge.mmd").write_text(_config_merge_mermaid(), encoding="utf-8")
    (OUT / "README.txt").write_text(
        "\n".join(
            [
                "Static dissertation assets generated successfully.",
                f"Generated at: {now}",
                "",
                "Files:",
                "- workspace_tree.txt",
                "- package_dependency_graph.dot",
                "- launch_config_merge.mmd",
                "",
                "Optional renders:",
                "  dot -Tpng package_dependency_graph.dot -o package_dependency_graph.png",
                "  dot -Tpdf package_dependency_graph.dot -o package_dependency_graph.pdf",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"[ok] Static assets written to: {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
