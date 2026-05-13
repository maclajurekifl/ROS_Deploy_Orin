#!/usr/bin/env python3
"""
Map SLAM / replay evidence to tuning blocks A–D (see script docstring output).

  A — Time, TF, extrinsics, LiDAR validity, IMU wiring, single odom publisher
  B — Stamp offsets, deskew (gyro frame, sign, IMU age, interpolation)
  C — Front-end odom: NDT health vs LIO, EMA, EKF variances / NIS / delta fusion
  D — Keyframe / merged map (density, voxel), global drift (loop / pose graph off)

Usage:
  python3 scripts/slam_symptom_classifier.py ~/bags/session_01
  python3 scripts/slam_symptom_classifier.py ~/bags/session_01 --what smeared,drift
  python3 scripts/slam_symptom_classifier.py --live   # same shell: source ROS + install first

Optional ``--what`` tokens: smeared, zigzag, drift, stutter, curl, startup, rotate
  (``rotate`` ≈ robot pose barely translates, mainly spins — often C then A.)
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field


@dataclass
class BagSummary:
    path: str
    duration_sec: float = 0.0
    topics: dict[str, dict] = field(default_factory=dict)  # topic -> {type, count}


def _run(cmd: list[str]) -> str:
    p = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return p.stdout


def parse_bag_info(path: str) -> BagSummary:
    out = _run(['ros2', 'bag', 'info', os.path.expanduser(path)])
    s = BagSummary(path=os.path.expanduser(path))
    dm = re.search(r'Duration:\s+([\d.]+)', out)
    if dm:
        s.duration_sec = float(dm.group(1))
    for line in out.splitlines():
        if 'Topic:' not in line or '|' not in line:
            continue
        # Topic: /foo | Type: bar | Count: 123 | ...
        m = re.search(
            r'Topic:\s+(\S+)\s+\|\s+Type:\s+([^|]+)\|\s+Count:\s+(\d+)',
            line,
        )
        if m:
            s.topics[m.group(1)] = {
                'type': m.group(2).strip(),
                'count': int(m.group(3)),
            }
    return s


def hz(count: int, duration: float) -> float:
    if duration <= 0:
        return 0.0
    return count / duration


def live_ros_graph() -> dict[str, object]:
    """Lightweight checks when launch + bag (or robot) are running."""
    out: dict[str, object] = {
        'topics': [],
        'clock': False,
        'errors': [],
        'graph_ok': False,
    }
    try:
        p = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=30,
        )
    except FileNotFoundError as e:
        out['errors'].append(f"ros2 not in PATH: {e}")
        return out
    except subprocess.TimeoutExpired:
        out['errors'].append('ros2 topic list timed out')
        return out

    if p.returncode != 0:
        err = (p.stderr or p.stdout or '').strip() or f'exit code {p.returncode}'
        out['errors'].append(f"ros2 topic list failed: {err}")
        out['errors'].append(
            'Fix: in this terminal run '
            '`source /opt/ros/humble/setup.bash && source ~/ROS_Deployment/install/setup.bash` '
            'before this script (same ROS_DOMAIN_ID as launch + bag).'
        )
        if 'rclpy.ok' in err or '!rclpy' in err:
            out['errors'].append(
                'If sourcing is already correct: `ros2 daemon stop && ros2 daemon start`, then retry '
                '(daemon can fault after crashes). Or run --live only while launch+bag are up and stable.'
            )
        return out

    tl = [ln for ln in p.stdout.strip().splitlines() if ln.strip()]
    out['topics'] = tl
    out['clock'] = '/clock' in tl
    out['graph_ok'] = True
    return out


SYMPTOM_HINTS: dict[str, tuple[str, ...]] = {
    'smeared': ('B', 'A', 'D'),
    'zigzag': ('C', 'A', 'B'),
    'drift': ('D', 'C', 'A'),
    'stutter': ('A', 'C'),
    'curl': ('D', 'C', 'A'),
    'startup': ('A', 'C'),
    # Little translation, lots of yaw / “spinning in place” in RViz or odom
    'rotate': ('C', 'A', 'B'),
}


def score_categories(
    bag: BagSummary | None,
    symptoms: list[str],
    live: dict[str, object] | None,
) -> dict[str, list[str]]:
    """Category -> list of evidence strings."""
    cat: dict[str, list[str]] = {'A': [], 'B': [], 'C': [], 'D': []}

    if bag and bag.duration_sec > 0:
        d = bag.duration_sec
        tf_c = bag.topics.get('/tf', {}).get('count', 0)
        tfs_c = bag.topics.get('/tf_static', {}).get('count', 0)
        if tf_c > 0:
            cat['A'].append(
                f"Bag has /tf with {tf_c} msgs — likely conflicts with EKF/bringup "
                f"(strip TF: scripts/bag_play_no_recorded_tf.py)."
            )
        if tfs_c > 0:
            cat['A'].append(
                f"Bag has /tf_static ({tfs_c} msgs) — if bringup also publishes the same "
                f"frames, fix duplicates or rely on one source only."
            )
        lidar = bag.topics.get('/livox/lidar', {}).get('count', 0)
        imu_gx5 = bag.topics.get('/imu/data', {}).get('count', 0)
        imu_lv = bag.topics.get('/livox/imu', {}).get('count', 0)
        rh = hz(lidar, d)
        ih = hz(imu_gx5, d)
        liv = hz(imu_lv, d)
        if lidar < 100:
            cat['A'].append(
                f"Low /livox/lidar message count ({lidar}) or short bag — check recording."
            )
        if imu_gx5 < 200:
            cat['A'].append(
                f"/imu/data ~{ih:.1f} Hz — EKF prediction needs steady IMU "
                f"(GX5 topic + slam_bringup use_microstrain_imu)."
            )
        if imu_lv < 200:
            cat['B'].append(
                f"/livox/imu ~{liv:.1f} Hz — deskew wants dense gyro; if smeared, tune B."
            )
        if liv > 0 and rh > 0 and liv / rh < 5.0:
            cat['B'].append(
                f"Livox IMU/cloud rate ratio low (imu {liv:.1f} Hz vs cloud {rh:.1f} Hz) — "
                f"check deskew buffer / max_imu_age."
            )
        if '/imu/data' not in bag.topics and '/livox/imu' in bag.topics:
            cat['A'].append(
                "Bag has /livox/imu but no /imu/data — default EKF uses GX5; "
                "set use_microstrain_imu false or record /imu/data."
            )

    if live and live.get('graph_ok'):
        topics = live.get('topics') or []
        if not live.get('clock'):
            cat['A'].append(
                "Live graph: no /clock — bag replay should use `ros2 bag play ... --clock` "
                "with use_sim_time true on stack."
            )
        need = (
            '/livox/lidar',
            '/lidar/odom',
            '/ekf/odom',
        )
        for t in need:
            if t not in topics:
                cat['A'].append(f"Live graph: missing {t} — stack not fully up or wrong namespace.")

    for sym in symptoms:
        chain = SYMPTOM_HINTS.get(sym, ())
        if not chain:
            continue
        order = ' → '.join(chain)
        cat[chain[0]].append(f"Symptom '{sym}' — check blocks in order: {order}.")

    return cat


def print_report(
    bag: BagSummary | None,
    symptoms: list[str],
    live: dict[str, object] | None,
) -> None:
    scores = score_categories(bag, symptoms, live)

    print("=== SLAM symptom → block (A–D) ===\n")

    if live and live.get('errors'):
        print("--- Live mode ---")
        for e in live['errors']:
            print(f"  ! {e}")
        if not live.get('graph_ok'):
            print(
                "  (Live topic checks skipped — fix sourcing above, then re-run with --live.)\n"
            )
        else:
            print()

    print(
        "A: time + TF + extrinsics + LiDAR validity + IMU topic/frames + single /lidar/odom\n"
        "B: stamp offsets + deskew (gyro frame/sign, IMU age, Livox vs cloud frame)\n"
        "C: NDT vs LIO choice, fitness skips, EMA, EKF variances/NIS, lidar_delta fusion\n"
        "D: keyframe spacing/voxel, merged map follows odom; loop/pose graph for global\n"
    )

    if bag:
        print(f"Bag: {bag.path}")
        print(f"Duration: {bag.duration_sec:.2f} s\n")
        for t in sorted(bag.topics):
            c = bag.topics[t]['count']
            h = hz(c, bag.duration_sec)
            print(f"  {t:22s}  count={c:6d}  ~{h:6.1f} Hz")
        print()

    if live and live.get('graph_ok') and live.get('topics'):
        print(f"Live topics: {len(live['topics'])} ( /clock: {live.get('clock')} )\n")

    if symptoms:
        print(f"Your symptoms: {', '.join(symptoms)}\n")

    # Order categories by evidence count
    ranked = sorted(scores.items(), key=lambda kv: len(kv[1]), reverse=True)
    print("--- Focus order (evidence count) ---")
    for letter, items in ranked:
        if not items:
            continue
        print(f"\n[{letter}]")
        for line in items:
            print(f"  • {line}")

    print("\n--- Default next actions (merge with above) ---")
    print(
        "  1. Bag replay: launch SLAM first, then `ros2 bag play BAG --clock` (`use_sim_time` default true).\n"
        "     Bag-first can flip/mirror the map (first NDT/keyframe vs TF at mid-bag sim time).\n"
        "  2. If TF wars: `python3 scripts/bag_play_no_recorded_tf.py BAG -- --clock`.\n"
        "  3. Verify `ros2 topic echo /livox/lidar --once` header.frame_id + stamp jump.\n"
        "  4. Deskew: slam_bringup `keyframe_deskew_*` + `livox_cloud_frame_id` alignment.\n"
        "  5. Odom like FAST-LIO: `use_lio:=true use_lidar_fusion:=false` or tune NDT skips/EMA/EKF.\n"
        "  6. Map curl over long runs: enable loop closure / pose graph when front-end is trusted.\n"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description='Classify SLAM issues into blocks A–D.')
    ap.add_argument(
        'bag',
        nargs='?',
        default='',
        help='Path to ros2 bag directory (optional if --live only)',
    )
    ap.add_argument(
        '--what',
        default='',
        help='Comma symptoms: smeared,zigzag,drift,stutter,curl,startup,rotate',
    )
    ap.add_argument(
        '--live',
        action='store_true',
        help='Inspect current ROS 2 graph (requires sourced ROS + workspace in this shell)',
    )
    args = ap.parse_args()

    symptoms = [x.strip().lower() for x in args.what.split(',') if x.strip()]
    for s in symptoms:
        if s not in SYMPTOM_HINTS:
            print(f"warning: unknown symptom '{s}' (known: {', '.join(SYMPTOM_HINTS)})", file=sys.stderr)

    bag: BagSummary | None = None
    if args.bag:
        p = os.path.expanduser(args.bag)
        if not os.path.exists(p):
            print(f"error: bag not found: {p}", file=sys.stderr)
            sys.exit(1)
        bag = parse_bag_info(p)

    live = live_ros_graph() if args.live else None

    if not bag and not args.live:
        ap.print_help()
        print("\nProvide a bag path and/or --live.", file=sys.stderr)
        sys.exit(1)

    print_report(bag, symptoms, live)


if __name__ == '__main__':
    main()
