#!/usr/bin/env python3
"""Build dissertation-ready assets under docs/dissertation_assets/report/.

Expected inputs:
  docs/dissertation_assets/static/
  docs/dissertation_assets/runtime/stationary/
  docs/dissertation_assets/runtime/moving/
"""

from __future__ import annotations

import argparse
import glob
import os
import pathlib
import shutil
import subprocess
from dataclasses import dataclass
import re


def _run(cmd: list[str]) -> tuple[int, str]:
    try:
        cp = subprocess.run(cmd, check=False, capture_output=True, text=True)
        return cp.returncode, cp.stdout + cp.stderr
    except FileNotFoundError:
        return 127, f"command not found: {cmd[0]}"


@dataclass
class Paths:
    ws: pathlib.Path
    assets: pathlib.Path
    static: pathlib.Path
    runtime: pathlib.Path
    stationary: pathlib.Path
    moving: pathlib.Path
    report: pathlib.Path
    figures: pathlib.Path
    evidence: pathlib.Path
    logs: pathlib.Path


def _mk_paths(workspace: pathlib.Path) -> Paths:
    assets = workspace / "docs" / "dissertation_assets"
    return Paths(
        ws=workspace,
        assets=assets,
        static=assets / "static",
        runtime=assets / "runtime",
        stationary=assets / "runtime" / "stationary",
        moving=assets / "runtime" / "moving",
        report=assets / "report",
        figures=assets / "report" / "figures",
        evidence=assets / "report" / "evidence",
        logs=assets / "report" / "logs",
    )


def _ensure_dirs(p: Paths) -> None:
    for d in (p.report, p.figures, p.evidence, p.logs):
        d.mkdir(parents=True, exist_ok=True)


def _copy_if_exists(src: pathlib.Path, dst: pathlib.Path, missing: list[str]) -> None:
    if src.exists():
        shutil.copy2(src, dst)
    else:
        missing.append(str(src))


def _first_match(pattern: str) -> pathlib.Path | None:
    matches = sorted(glob.glob(pattern))
    return pathlib.Path(matches[0]) if matches else None


def _render_tree_txt_to_png(tree_txt: pathlib.Path, out_png: pathlib.Path, logf: pathlib.Path) -> bool:
    # Render text file to PNG with ImageMagick if available.
    rc, out = _run(["convert", "-background", "white", "-fill", "black", "-font", "DejaVu-Sans-Mono", "-pointsize", "14", f"label:@{tree_txt}", str(out_png)])
    logf.write_text(out, encoding="utf-8")
    return rc == 0


def _extract_hz_summary(path: pathlib.Path) -> str:
    if not path.exists():
        return "missing"
    txt = path.read_text(encoding="utf-8", errors="ignore")
    for line in reversed(txt.splitlines()):
        l = line.strip()
        if "average rate:" in l.lower():
            return l
    return "no average rate line"


def _render_mermaid(mmd: pathlib.Path, png: pathlib.Path, pdf: pathlib.Path, logf: pathlib.Path) -> bool:
    rc1, out1 = _run(["mmdc", "-i", str(mmd), "-o", str(png), "-b", "white"])
    rc2, out2 = _run(["mmdc", "-i", str(mmd), "-o", str(pdf), "-b", "white"])
    logf.write_text(out1 + "\n" + out2, encoding="utf-8")
    return rc1 == 0 and rc2 == 0


def _topic_matrix_from_list(topic_list_types: pathlib.Path, out_md: pathlib.Path) -> None:
    rows: list[tuple[str, str]] = []
    if topic_list_types.exists():
        for line in topic_list_types.read_text(encoding="utf-8", errors="ignore").splitlines():
            line = line.strip()
            if not line or "[" not in line or "]" not in line:
                continue
            topic = line.split("[", 1)[0].strip()
            typ = line.split("[", 1)[1].split("]", 1)[0].strip()
            rows.append((topic, typ))
    rows.sort(key=lambda x: x[0])
    md = ["# Topic matrix (moving capture)", "", "| Topic | Type |", "|---|---|"]
    md.extend([f"| `{t}` | `{y}` |" for t, y in rows])
    out_md.write_text("\n".join(md) + "\n", encoding="utf-8")


def _qos_summary_from_topic_info(moving_dir: pathlib.Path, out_md: pathlib.Path) -> None:
    targets = [
        "topic_info__livox_lidar.txt",
        "topic_info__livox_imu.txt",
        "topic_info__imu_data.txt",
        "topic_info__lidar_odom_raw.txt",
        "topic_info__lidar_odom.txt",
        "topic_info__ekf_odom.txt",
        "topic_info__tf.txt",
        "topic_info__tf_static.txt",
    ]
    md = ["# QoS summary (moving capture)", "", "| Topic info file | Reliability | Durability |", "|---|---|---|"]
    for name in targets:
        f = moving_dir / name
        reliability = "n/a"
        durability = "n/a"
        if f.exists():
            txt = f.read_text(encoding="utf-8", errors="ignore")
            m_rel = re.search(r"Reliability:\s*([A-Za-z_]+)", txt)
            m_dur = re.search(r"Durability:\s*([A-Za-z_]+)", txt)
            if m_rel:
                reliability = m_rel.group(1)
            if m_dur:
                durability = m_dur.group(1)
        md.append(f"| `{name}` | `{reliability}` | `{durability}` |")
    out_md.write_text("\n".join(md) + "\n", encoding="utf-8")


def _make_zip_bundle(report_dir: pathlib.Path) -> pathlib.Path:
    base = report_dir.parent / "report_assets_bundle"
    archive = shutil.make_archive(str(base), "zip", root_dir=str(report_dir))
    return pathlib.Path(archive)


def build(workspace: pathlib.Path) -> int:
    p = _mk_paths(workspace)
    _ensure_dirs(p)
    missing: list[str] = []

    # Core static assets
    _copy_if_exists(
        p.static / "package_dependency_graph.pdf",
        p.figures / "fig_5_4_3_package_dependency_graph.pdf",
        missing,
    )
    _copy_if_exists(
        p.static / "package_dependency_graph.png",
        p.figures / "fig_5_4_3_package_dependency_graph.png",
        missing,
    )
    _copy_if_exists(
        p.static / "launch_config_merge.mmd",
        p.figures / "fig_5_4_8_launch_config_merge.mmd",
        missing,
    )
    mmd_file = p.figures / "fig_5_4_8_launch_config_merge.mmd"
    if mmd_file.exists():
        _render_mermaid(
            mmd_file,
            p.figures / "fig_5_4_8_launch_config_merge.png",
            p.figures / "fig_5_4_8_launch_config_merge.pdf",
            p.logs / "mermaid_render.log",
        )
    _copy_if_exists(
        p.static / "workspace_tree.txt",
        p.evidence / "workspace_tree.txt",
        missing,
    )

    # Try to create a PNG version of workspace tree text.
    tree_txt = p.static / "workspace_tree.txt"
    if tree_txt.exists():
        _render_tree_txt_to_png(
            tree_txt,
            p.figures / "fig_5_4_7_workspace_tree.png",
            p.logs / "workspace_tree_render.log",
        )

    # Runtime figures from stationary and moving
    st_tf = _first_match(str(p.stationary / "frames_*.pdf"))
    mv_tf = _first_match(str(p.moving / "frames_*.pdf"))
    if st_tf:
        shutil.copy2(st_tf, p.figures / "fig_5_4_6_tf_tree_stationary.pdf")
    else:
        missing.append(str(p.stationary / "frames_*.pdf"))
    if mv_tf:
        shutil.copy2(mv_tf, p.figures / "fig_5_4_6_tf_tree_moving.pdf")
    else:
        missing.append(str(p.moving / "frames_*.pdf"))

    # Evidence files: topics, nodes, QoS
    for src, dst in [
        (p.stationary / "node_list.txt", p.evidence / "stationary_node_list.txt"),
        (p.moving / "node_list.txt", p.evidence / "moving_node_list.txt"),
        (p.stationary / "topic_list_types.txt", p.evidence / "stationary_topic_list_types.txt"),
        (p.moving / "topic_list_types.txt", p.evidence / "moving_topic_list_types.txt"),
        (p.moving / "topic_info__ekf_odom.txt", p.evidence / "moving_topic_info_ekf_odom.txt"),
        (p.moving / "topic_info__tf.txt", p.evidence / "moving_topic_info_tf.txt"),
        (p.moving / "topic_info__tf_static.txt", p.evidence / "moving_topic_info_tf_static.txt"),
        (p.stationary / "INDEX.md", p.evidence / "stationary_INDEX.md"),
        (p.moving / "INDEX.md", p.evidence / "moving_INDEX.md"),
    ]:
        _copy_if_exists(src, dst, missing)

    # Topic-rate summaries
    hz_targets = [
        "_livox_lidar",
        "_livox_imu",
        "_imu_data",
        "_lidar_odom_raw",
        "_lidar_odom",
        "_ekf_odom",
        "_tf",
    ]
    lines = ["# Topic rate comparison (stationary vs moving)", ""]
    lines.append("| Topic file suffix | Stationary | Moving |")
    lines.append("|---|---|---|")
    for suffix in hz_targets:
        st = p.stationary / f"hz_{suffix}.txt"
        mv = p.moving / f"hz_{suffix}.txt"
        lines.append(f"| `{suffix}` | {_extract_hz_summary(st)} | {_extract_hz_summary(mv)} |")
        if st.exists():
            shutil.copy2(st, p.evidence / f"stationary_hz_{suffix}.txt")
        if mv.exists():
            shutil.copy2(mv, p.evidence / f"moving_hz_{suffix}.txt")
    (p.report / "rate_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    _topic_matrix_from_list(p.moving / "topic_list_types.txt", p.report / "topic_matrix_moving.md")
    _qos_summary_from_topic_info(p.moving, p.report / "qos_summary_moving.md")

    bundle = _make_zip_bundle(p.report)

    # Report index + suggested captions
    index = [
        "# Dissertation report asset pack",
        "",
        "## Figures",
        "- `fig_5_4_3_package_dependency_graph.pdf/png`",
        "- `fig_5_4_6_tf_tree_stationary.pdf`",
        "- `fig_5_4_6_tf_tree_moving.pdf`",
        "- `fig_5_4_7_workspace_tree.png` (if ImageMagick available)",
        "- `fig_5_4_8_launch_config_merge.mmd`",
        "- `fig_5_4_8_launch_config_merge.png/.pdf` (if Mermaid CLI `mmdc` available)",
        "",
        "## Evidence files",
        "- `evidence/stationary_*`, `evidence/moving_*`",
        "- `rate_summary.md`",
        "- `topic_matrix_moving.md`",
        "- `qos_summary_moving.md`",
        "",
        "## Transfer bundle",
        f"- `{bundle}`",
        "",
        "## Suggested captions",
        "1. Figure 5.4.3: ROS workspace package dependency graph generated from package manifests.",
        "2. Figure 5.4.6a: TF tree during stationary run.",
        "3. Figure 5.4.6b: TF tree during moving run.",
        "4. Figure 5.4.7: Workspace and package directory structure used in deployment.",
        "5. Figure 5.4.8: Launch-time configuration merge flow (defaults, overlays, CLI overrides).",
        "6. Table 5.4.2: Moving-run topic matrix (topic names and message types).",
        "7. Table 5.4.5: QoS summary for critical topics in the moving run.",
        "",
        "## Missing inputs",
    ]
    if missing:
        index.extend([f"- `{m}`" for m in missing])
    else:
        index.append("- none")
    (p.report / "README.md").write_text("\n".join(index) + "\n", encoding="utf-8")

    print(f"[ok] Report assets prepared in: {p.report}")
    if missing:
        print("[warn] Some expected inputs were missing; see report/README.md")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", default="~/ROS_Deployment", help="Workspace root")
    args = ap.parse_args()
    return build(pathlib.Path(os.path.expanduser(args.workspace)).resolve())


if __name__ == "__main__":
    raise SystemExit(main())
