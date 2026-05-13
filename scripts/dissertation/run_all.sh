#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/ROS_Deployment}"
cd "$WS_DIR"

echo "[step] Generating static assets..."
python3 "$WS_DIR/scripts/dissertation/export_static_assets.py"

STATIC_DIR="$WS_DIR/docs/dissertation_assets/static"
if command -v dot >/dev/null 2>&1; then
  echo "[step] Rendering package dependency graph..."
  dot -Tpng "$STATIC_DIR/package_dependency_graph.dot" -o "$STATIC_DIR/package_dependency_graph.png"
  dot -Tpdf "$STATIC_DIR/package_dependency_graph.dot" -o "$STATIC_DIR/package_dependency_graph.pdf"
else
  echo "[warn] Graphviz 'dot' not installed; skipping PNG/PDF render."
fi

echo "[step] Capturing runtime assets (requires active ROS graph)..."
"$WS_DIR/scripts/dissertation/capture_runtime_assets.sh"

echo "[step] Building report asset pack (expects runtime/stationary and runtime/moving)..."
python3 "$WS_DIR/scripts/dissertation/build_report_assets.py" --workspace "$WS_DIR" || true

echo "[done] Dissertation asset generation complete."
