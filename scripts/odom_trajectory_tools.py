#!/usr/bin/env python3
"""
Record nav_msgs/Odometry into CSV and compare FAST-LIO vs SLAM runs.

Recommended flow (same bag, two passes):
  1) Run FAST-LIO + bag replay.
  2a) FAST-LIO /Odometry (camera_init->body): record **base_link in odom** when TF connects
      camera_init->odom and body->base_link (typical: bag /tf + robot static tree + FAST-LIO).
      If `tf2_echo` shows no path, run (sim time) ``odom``→``camera_init``; with **EKF**
      ``odom``→``base_link`` use ``bridge_body_to_base_link:=false`` and load FAST-LIO
      ``ros_project_bringup/config/fastlio_bag_replay_overlay.yaml`` **last** (``publish_tf`` +
      ``lidar_type`` for bags — does not change robot ``lio_bringup`` overlay). Example::
        ros2 launch ros_project_bringup tf_bridge_fastlio_odom_compare.launch.py \\
          use_sim_time:=true bridge_body_to_base_link:=false
      (``bridge_odom_to_map:=true`` only if that launch's docstring says you need it.)
     python3 scripts/odom_trajectory_tools.py record --use-sim-time --topic /Odometry \
       --start-sec 30 --end-sec 80 -o fastlio_odom.csv \
       --output-parent-frame odom --compose-output-child base_link
  2b) Same without TF (legacy raw camera_init XY — not comparable to /ekf/odom XY):
     python3 scripts/odom_trajectory_tools.py record --use-sim-time --topic /Odometry \
       --start-sec 30 --end-sec 80 -o fastlio.csv
  3) Run your SLAM + same bag replay.
  4) record command:
     python3 scripts/odom_trajectory_tools.py record --use-sim-time --topic /ekf/odom \
       --start-sec 30 --end-sec 80 -o slam.csv
  5) Plot + text report:
     python3 scripts/odom_trajectory_tools.py compare \
       --ref fastlio.csv:FAST-LIO --test slam.csv:SLAM --out-prefix run_compare

  6) XY rigid fit (SE(2)) — separates “wrong heading / offset in plane” from “path shape wrong”:
     python3 scripts/odom_trajectory_tools.py align_xy \
       --ref fastlio.csv:FAST-LIO --test slam_lidar7.csv:LiDAR --out-prefix xy_align

Outputs:
  - run_compare_xy_yaw.png
  - run_compare_report.txt
  - xy_align_report.txt, xy_align_overlay.png (from align_xy)

Tuning workflow (NDT vs FAST-LIO, same stamp window):

  ros2 … + bag + FAST-LIO + SLAM → then::
    python3 scripts/odom_trajectory_tools.py auto_fastlio_vs_slam --use-sim-time \\
      --out-dir /tmp/tune_RUN --warmup-sec 5 --duration-sec 50 --wait-timeout-sec 240 \\
      --record-ndt-raw

  Reuse an **absolute** header-stamp window from a prior run (orchestrator log / ``*.record_meta.json``)::
    python3 scripts/odom_trajectory_tools.py auto_fastlio_vs_slam --use-sim-time \\
      --out-dir /tmp/tune_same_window --rec-start-stamp-sec 123.45 --rec-end-stamp-sec 173.45 \\
      --record-ndt-raw

  Plots/reports appear under ``--out-dir`` (``*_xy_yaw.png``, ``*_overlay.png``, ``*_report.txt``).
  Aggregate several runs::
    python3 scripts/odom_trajectory_tools.py tune_rollup /tmp/tune_a /tmp/tune_b
  
"""
from __future__ import annotations

import argparse
import csv
import re
import json
import math
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped for tf2_ros.Buffer.transform
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def angle_wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _pose_to_T44(pose) -> 'numpy.ndarray':
    from tf_transformations import quaternion_matrix

    q = [
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    ]
    T = quaternion_matrix(q)
    T[0, 3] = float(pose.position.x)
    T[1, 3] = float(pose.position.y)
    T[2, 3] = float(pose.position.z)
    T[3, 3] = 1.0
    return T


def _T44_to_xy_yaw(T44: 'numpy.ndarray') -> Tuple[float, float, float]:
    x = float(T44[0, 3])
    y = float(T44[1, 3])
    yaw = math.atan2(float(T44[1, 0]), float(T44[0, 0]))
    return x, y, yaw


def _T_from_tf_msg(tf_msg) -> 'numpy.ndarray':
    from tf_transformations import quaternion_matrix

    r = tf_msg.transform.rotation
    t = tf_msg.transform.translation
    T = quaternion_matrix([float(r.x), float(r.y), float(r.z), float(r.w)])
    T[0, 3] = float(t.x)
    T[1, 3] = float(t.y)
    T[2, 3] = float(t.z)
    T[3, 3] = 1.0
    return T


class OdomRecorder(Node):
    def __init__(
        self,
        node_name: str,
        topic: str,
        out_path: Path,
        start_sec: float,
        end_sec: float,
        start_stamp_sec: float | None,
        end_stamp_sec: float | None,
        use_sim_time: bool,
        best_effort: bool,
        output_parent_frame: str = '',
        compose_output_child: str = '',
        tf_timeout_sec: float = 0.25,
    ):
        overrides = []
        if use_sim_time:
            from rclpy.parameter import Parameter

            overrides.append(Parameter('use_sim_time', value=True))
        super().__init__(node_name, parameter_overrides=overrides)

        self._topic = topic
        self._out_path = out_path
        self._start_sec = max(0.0, float(start_sec))
        self._end_sec = max(self._start_sec + 0.5, float(end_sec))
        self._start_stamp_sec = None if start_stamp_sec is None else float(start_stamp_sec)
        self._end_stamp_sec = None if end_stamp_sec is None else float(end_stamp_sec)
        if (self._start_stamp_sec is None) ^ (self._end_stamp_sec is None):
            raise ValueError('--start-stamp-sec and --end-stamp-sec must be provided together')
        if self._start_stamp_sec is not None:
            if not math.isfinite(self._start_stamp_sec) or not math.isfinite(self._end_stamp_sec):  # type: ignore[arg-type]
                raise ValueError('--start-stamp-sec/--end-stamp-sec must be finite numbers')
            if self._end_stamp_sec <= self._start_stamp_sec:  # type: ignore[operator]
                raise ValueError('--end-stamp-sec must be > --start-stamp-sec')

        self._rows: List[Tuple[float, float, float, float, float, float, float]] = []
        self._first_stamp: float | None = None

        self._out_parent = str(output_parent_frame or '').strip()
        self._compose_child = str(compose_output_child or '').strip()
        self._tf_timeout = Duration(seconds=float(tf_timeout_sec))
        self._tf_buffer: Optional[tf2_ros.Buffer] = None
        self._tf_listener: Optional[tf2_ros.TransformListener] = None
        self._tf_warned = False
        self._tf_fail_count = 0
        self._tf_ok_count = 0
        self._spatial_parent_fallback_info = False
        self._effective_spatial_parent = self._out_parent
        if self._out_parent:
            import tf2_ros

            try:
                import tf_transformations  # noqa: F401
            except ImportError as exc:
                raise SystemExit(
                    'TF projection needs tf_transformations (ros-humble-tf-transformations). '
                    'Install apt package or pip install tf-transformations.'
                ) from exc
            self._tf_buffer = tf2_ros.Buffer(
                cache_time=rclpy.duration.Duration(seconds=600.0)
            )
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
            self.get_logger().info(
                f'TF projection: pose -> {self._out_parent!r} '
                f"then child -> {self._compose_child or '(msg child)'} for CSV x,y,yaw"
            )

        rel = QoSReliabilityPolicy.BEST_EFFORT if best_effort else QoSReliabilityPolicy.RELIABLE
        qos = QoSProfile(depth=2000, reliability=rel, history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, topic, self._cb, qos)

        if self._start_stamp_sec is not None:
            self.get_logger().info(
                f"Recording {topic} in stamp window [{self._start_stamp_sec:.3f}, {self._end_stamp_sec:.3f}] s "
                f"(message header stamp) -> {self._out_path}"
            )
        else:
            self.get_logger().info(
                f"Recording {topic} in window [{self._start_sec:g}, {self._end_sec:g}] s "
                f"from first sample -> {self._out_path}"
            )

    def _project_xy_yaw_vxvy(self, msg: Odometry) -> Tuple[float, float, float, float, float]:
        """Return x, y, yaw in output plane, vx, vy (planar linear in output frame when TF ok)."""
        import numpy as np

        p0 = msg.pose.pose.position
        o0 = msg.pose.pose.orientation
        if not self._out_parent:
            yaw0 = yaw_from_quat(o0.x, o0.y, o0.z, o0.w)
            return (
                float(p0.x),
                float(p0.y),
                yaw0,
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
            )

        assert self._tf_buffer is not None
        msg_parent = str(msg.header.frame_id or '').strip()
        child = str(msg.child_frame_id or '').strip()

        # Step 1: pose of `child` in spatial parent (requested --output-parent-frame, or odometry header).
        if msg_parent == self._out_parent:
            T_parent_body = _pose_to_T44(msg.pose.pose)
            self._effective_spatial_parent = msg_parent
        else:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            try:
                ps_out = self._tf_buffer.transform(
                    ps, self._out_parent, timeout=self._tf_timeout
                )
                T_parent_body = _pose_to_T44(ps_out.pose)
                self._effective_spatial_parent = self._out_parent
            except Exception as exc:
                err_s = str(exc).lower()
                # FAST-LIO-only replay: /tf often has no `odom`. Message pose is already child in msg_parent.
                if (
                    self._out_parent.lower() == 'odom'
                    and msg_parent
                    and msg_parent != self._out_parent
                    and ('does not exist' in err_s or 'could not find' in err_s)
                ):
                    if not self._spatial_parent_fallback_info:
                        self.get_logger().info(
                            f"TF has no frame {self._out_parent!r}; using odometry header "
                            f"{msg_parent!r} as spatial parent (pose of {child!r} in {msg_parent!r}). "
                            f'CSV x,y,yaw will match /lidar/odom only up to a fixed world offset unless you '
                            f'publish static TF {self._out_parent!r}->{msg_parent!r} (identity if origins align).'
                        )
                        self._spatial_parent_fallback_info = True
                    self._effective_spatial_parent = msg_parent
                    T_parent_body = _pose_to_T44(msg.pose.pose)
                else:
                    self._tf_fail_count += 1
                    if not self._tf_warned:
                        self.get_logger().warn(
                            f'TF transform {msg_parent!r} -> {self._out_parent!r} failed ({exc}); '
                            'using raw odometry pose. Fix /tf or use --output-parent-frame matching the bag.'
                        )
                        self._tf_warned = True
                    yaw0 = yaw_from_quat(o0.x, o0.y, o0.z, o0.w)
                    return (
                        float(p0.x),
                        float(p0.y),
                        yaw0,
                        float(msg.twist.twist.linear.x),
                        float(msg.twist.twist.linear.y),
                    )

        if self._compose_child and child and self._compose_child != child:
            t_msg = Time.from_msg(msg.header.stamp)
            tb = None
            compose_err: Optional[Exception] = None
            try:
                tb = self._tf_buffer.lookup_transform(
                    child,
                    self._compose_child,
                    t_msg,
                    timeout=self._tf_timeout,
                )
            except Exception as exc:
                compose_err = exc
                err_s = str(exc).lower()
                # /Odometry stamp can be slightly newer than the newest TF sample (bag ordering,
                # different publishers). body->base_link is fixed; tf2 "time 0" = latest is fine here.
                if 'extrapolation' in err_s or 'into the future' in err_s:
                    try:
                        tb = self._tf_buffer.lookup_transform(
                            child,
                            self._compose_child,
                            Time(
                                nanoseconds=0,
                                clock_type=self.get_clock().clock_type,
                            ),
                            timeout=self._tf_timeout,
                        )
                        compose_err = None
                    except Exception as exc2:
                        compose_err = exc2
            if tb is not None:
                T_body_base = _T_from_tf_msg(tb)
                T_out = T_parent_body @ T_body_base
            else:
                self._tf_fail_count += 1
                if not self._tf_warned and compose_err is not None:
                    self.get_logger().warn(
                        f'TF compose {child!r}->{self._compose_child!r} failed ({compose_err}); '
                        f'using pose of {child!r} in {self._effective_spatial_parent!r} '
                        f'(not {self._compose_child!r}).'
                    )
                    self._tf_warned = True
                x, y, yaw = _T44_to_xy_yaw(T_parent_body)
                R = T_parent_body[:3, :3]
                vb = np.array(
                    [
                        float(msg.twist.twist.linear.x),
                        float(msg.twist.twist.linear.y),
                        float(msg.twist.twist.linear.z),
                    ]
                )
                vo = R @ vb
                self._tf_ok_count += 1
                return x, y, yaw, float(vo[0]), float(vo[1])
        else:
            T_out = T_parent_body

        x, y, yaw = _T44_to_xy_yaw(T_out)
        R = T_out[:3, :3]
        vb = np.array(
            [
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
                float(msg.twist.twist.linear.z),
            ]
        )
        vo = R @ vb
        self._tf_ok_count += 1
        return x, y, yaw, float(vo[0]), float(vo[1])

    def _cb(self, msg: Odometry):
        t = stamp_to_sec(msg.header.stamp)
        if self._first_stamp is None:
            self._first_stamp = t
        t_rel = t - self._first_stamp
        if self._start_stamp_sec is not None:
            if t < self._start_stamp_sec or t > self._end_stamp_sec:  # type: ignore[operator]
                return
        else:
            if t_rel < self._start_sec or t_rel > self._end_sec:
                return

        x, y, yaw, vx, vy = self._project_xy_yaw_vxvy(msg)
        self._rows.append((t, t_rel, x, y, yaw, vx, vy))

    def done(self) -> bool:
        if self._first_stamp is None:
            return False
        if not self._rows:
            return False
        if self._start_stamp_sec is not None:
            return self._rows[-1][0] >= float(self._end_stamp_sec) - 1e-6  # type: ignore[arg-type]
        return self._rows[-1][1] >= self._end_sec - 1e-6

    def save(self):
        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._out_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['stamp_sec', 't_rel_sec', 'x', 'y', 'yaw_rad', 'vx', 'vy'])
            w.writerows(self._rows)
        self.get_logger().info(f"Wrote {len(self._rows)} rows to {self._out_path}")

        if self._out_parent:
            meta_path = self._out_path.parent / (self._out_path.stem + '.record_meta.json')
            meta = {
                'topic': self._topic,
                'tf_projection': True,
                'start_stamp_sec': self._start_stamp_sec,
                'end_stamp_sec': self._end_stamp_sec,
                'output_parent_frame_requested': self._out_parent,
                'effective_spatial_parent': self._effective_spatial_parent,
                'output_child_frame': self._compose_child or '(same as msg child_frame_id)',
                'used_odom_missing_fallback': self._effective_spatial_parent != self._out_parent,
                'tf_timeout_sec': float(self._tf_timeout.nanoseconds) / 1e9,
                'tf_rows_pose_projected_ok': int(self._tf_ok_count),
                'tf_rows_fallback_raw': int(self._tf_fail_count),
                'csv_rows': len(self._rows),
                'note': (
                    'When used_odom_missing_fallback is true, CSV xy is in camera_init (same as FAST-LIO '
                    'header), not odom. Publish static odom->camera_init for true odom CSV. '
                    'When tf_rows_fallback_raw is high, body->base_link or TF chain failed.'
                ),
            }
            meta_path.write_text(json.dumps(meta, indent=2) + '\n')
            self.get_logger().info(f'Wrote {meta_path}')


def load_csv(path: Path):
    ts, tr, xs, ys, yaws = [], [], [], [], []
    with open(path, newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            ts.append(float(row['stamp_sec']))
            tr.append(float(row['t_rel_sec']))
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            yaws.append(float(row['yaw_rad']))
    return ts, tr, xs, ys, yaws


def path_len(xs: List[float], ys: List[float]) -> float:
    d = 0.0
    for i in range(1, len(xs)):
        d += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
    return d


def _read_record_meta(csv_path: Path) -> dict | None:
    meta_path = csv_path.parent / (csv_path.stem + '.record_meta.json')
    if not meta_path.is_file():
        return None
    try:
        return json.loads(meta_path.read_text())
    except (json.JSONDecodeError, OSError):
        return None


def cmd_record(args):
    rclpy.init()
    node = OdomRecorder(
        node_name='odom_trajectory_recorder',
        topic=args.topic,
        out_path=Path(args.out),
        start_sec=args.start_sec,
        end_sec=args.end_sec,
        start_stamp_sec=getattr(args, 'start_stamp_sec', None),
        end_stamp_sec=getattr(args, 'end_stamp_sec', None),
        use_sim_time=bool(args.use_sim_time),
        best_effort=bool(args.best_effort),
        output_parent_frame=str(getattr(args, 'output_parent_frame', '') or ''),
        compose_output_child=str(getattr(args, 'compose_output_child', '') or ''),
        tf_timeout_sec=float(getattr(args, 'tf_timeout_sec', 0.25)),
    )

    wall_start = time.monotonic()
    if getattr(args, 'end_stamp_sec', None) is not None:
        # Stamp-window mode: allow a generous wall timeout since bag rate / playback can vary.
        wall_timeout = wall_start + max(240.0, (float(args.end_stamp_sec) - float(args.start_stamp_sec) + 180.0))
    else:
        wall_timeout = wall_start + max(180.0, (args.end_sec + 90.0))
    try:
        while time.monotonic() < wall_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.done():
                break
            if node._first_stamp is None and time.monotonic() - wall_start > 120.0:
                node.get_logger().error('No odometry messages in 120s. Check topic / sim-time / QoS.')
                break
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # Common during Ctrl+C: executor wakes while context is shutting down.
        msg = str(exc)
        if 'context is invalid' not in msg and 'Unable to convert call argument' not in msg:
            raise
    finally:
        node.save()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def _topic_first_stamp_sec(
    node: Node,
    topic: str,
    wait_timeout_sec: float,
    best_effort: bool,
) -> float:
    """Wait until one Odometry message arrives, return its header stamp as float seconds."""
    got = {'t': None}

    rel = QoSReliabilityPolicy.BEST_EFFORT if best_effort else QoSReliabilityPolicy.RELIABLE
    qos = QoSProfile(depth=10, reliability=rel, history=QoSHistoryPolicy.KEEP_LAST)

    def cb(msg: Odometry):
        got['t'] = stamp_to_sec(msg.header.stamp)

    sub = node.create_subscription(Odometry, topic, cb, qos)
    start = time.monotonic()
    while time.monotonic() - start < wait_timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
        if got['t'] is not None:
            break
    node.destroy_subscription(sub)
    if got['t'] is None:
        raise TimeoutError(f'No messages on {topic!r} within {wait_timeout_sec:.1f}s')
    return float(got['t'])


def cmd_auto_fastlio_vs_slam(args):
    """
    Auto-pick a shared header-stamp window, record FAST-LIO/NDT/EKF, then compare + align.

    This avoids the common pitfall where --start-sec/--end-sec are interpreted as seconds since
    the *first* message on each topic (t_rel), which can differ across passes.
    """
    rclpy.init()
    exec = None
    orchestrator = None
    rec_fastlio = rec_ndt = rec_ekf = rec_ndt_raw = None
    try:
        overrides = []
        if bool(args.use_sim_time):
            from rclpy.parameter import Parameter

            overrides.append(Parameter('use_sim_time', value=True))
        orchestrator = Node('odom_trajectory_auto', parameter_overrides=overrides)

        out_dir = Path(str(args.out_dir)).expanduser().resolve()
        out_dir.mkdir(parents=True, exist_ok=True)

        fastlio_topic = str(args.fastlio_topic)
        ndt_topic = str(args.ndt_topic)
        ekf_topic = str(args.ekf_topic)

        wait_timeout = float(args.wait_timeout_sec)
        warmup = max(0.0, float(args.warmup_sec))
        dur = max(1.0, float(args.duration_sec))

        orchestrator.get_logger().info('Waiting for first message on each topic...')
        t_fastlio0 = _topic_first_stamp_sec(orchestrator, fastlio_topic, wait_timeout, bool(args.best_effort))
        t_ndt0 = _topic_first_stamp_sec(orchestrator, ndt_topic, wait_timeout, bool(args.best_effort))
        t_ekf0 = _topic_first_stamp_sec(orchestrator, ekf_topic, wait_timeout, bool(args.best_effort))

        ndt_raw_topic = str(getattr(args, 'ndt_raw_topic', '') or '').strip()
        record_raw = bool(getattr(args, 'record_ndt_raw', False)) and bool(ndt_raw_topic)
        if record_raw and ndt_raw_topic == ndt_topic:
            orchestrator.get_logger().warn(
                f'--ndt-raw-topic matches --ndt-topic; skipping duplicate raw recorder.'
            )
            record_raw = False

        t_raw0: float | None = None
        if record_raw:
            raw_wait = min(20.0, wait_timeout)
            try:
                t_raw0 = _topic_first_stamp_sec(
                    orchestrator, ndt_raw_topic, raw_wait, bool(args.best_effort)
                )
            except TimeoutError:
                orchestrator.get_logger().warn(
                    f'No messages on {ndt_raw_topic!r} within {raw_wait:.0f}s; '
                    'skipping raw NDT recording (use without --record-ndt-raw if you have no raw topic).'
                )
                record_raw = False

        firsts = [t_fastlio0, t_ndt0, t_ekf0]
        if t_raw0 is not None:
            firsts.append(t_raw0)
        rs = getattr(args, 'rec_start_stamp_sec', None)
        re = getattr(args, 'rec_end_stamp_sec', None)
        if (rs is not None) ^ (re is not None):
            raise ValueError(
                '--rec-start-stamp-sec and --rec-end-stamp-sec must be given together (or neither).'
            )
        if rs is not None:
            start_stamp = float(rs)
            end_stamp = float(re)
            if end_stamp <= start_stamp:
                raise ValueError('--rec-end-stamp-sec must be > --rec-start-stamp-sec')
            t_first = max(firsts)
            if start_stamp + 1e-6 < t_first:
                orchestrator.get_logger().warn(
                    f'--rec-start-stamp-sec ({start_stamp:.3f}) is before latest topic first stamp '
                    f'({t_first:.3f}); CSVs may be empty or short.'
                )
            orchestrator.get_logger().info(
                f'Using **fixed** shared stamp window [{start_stamp:.3f}, {end_stamp:.3f}] sec '
                f'(ignores --warmup-sec / --duration-sec for window bounds)'
            )
        else:
            start_stamp = max(firsts) + warmup
            end_stamp = start_stamp + dur
            orchestrator.get_logger().info(
                f'Using shared stamp window [{start_stamp:.3f}, {end_stamp:.3f}] sec '
                f'(warmup={warmup:g}s, duration={dur:g}s)'
            )

        fastlio_csv = out_dir / 'fastlio_ref.csv'
        ndt_csv = out_dir / 'ndt.csv'
        ndt_raw_csv = out_dir / 'ndt_raw.csv'
        ekf_csv = out_dir / 'ekf.csv'

        tf_timeout_sec = float(args.tf_timeout_sec)

        rec_fastlio = OdomRecorder(
            node_name='odom_trajectory_recorder_fastlio',
            topic=fastlio_topic,
            out_path=fastlio_csv,
            start_sec=0.0,
            end_sec=1.0,
            start_stamp_sec=start_stamp,
            end_stamp_sec=end_stamp,
            use_sim_time=bool(args.use_sim_time),
            best_effort=bool(args.best_effort),
            output_parent_frame=str(args.fastlio_output_parent_frame),
            compose_output_child=str(args.fastlio_compose_output_child),
            tf_timeout_sec=tf_timeout_sec,
        )
        rec_ndt = OdomRecorder(
            node_name='odom_trajectory_recorder_ndt',
            topic=ndt_topic,
            out_path=ndt_csv,
            start_sec=0.0,
            end_sec=1.0,
            start_stamp_sec=start_stamp,
            end_stamp_sec=end_stamp,
            use_sim_time=bool(args.use_sim_time),
            best_effort=bool(args.best_effort),
            output_parent_frame='',
            compose_output_child='',
            tf_timeout_sec=tf_timeout_sec,
        )
        rec_ekf = OdomRecorder(
            node_name='odom_trajectory_recorder_ekf',
            topic=ekf_topic,
            out_path=ekf_csv,
            start_sec=0.0,
            end_sec=1.0,
            start_stamp_sec=start_stamp,
            end_stamp_sec=end_stamp,
            use_sim_time=bool(args.use_sim_time),
            best_effort=bool(args.best_effort),
            output_parent_frame='',
            compose_output_child='',
            tf_timeout_sec=tf_timeout_sec,
        )

        if record_raw:
            rec_ndt_raw = OdomRecorder(
                node_name='odom_trajectory_recorder_ndt_raw',
                topic=ndt_raw_topic,
                out_path=ndt_raw_csv,
                start_sec=0.0,
                end_sec=1.0,
                start_stamp_sec=start_stamp,
                end_stamp_sec=end_stamp,
                use_sim_time=bool(args.use_sim_time),
                best_effort=bool(args.best_effort),
                output_parent_frame='',
                compose_output_child='',
                tf_timeout_sec=tf_timeout_sec,
            )

        exec = rclpy.executors.MultiThreadedExecutor(num_threads=8)
        exec.add_node(orchestrator)
        exec.add_node(rec_fastlio)
        exec.add_node(rec_ndt)
        exec.add_node(rec_ekf)
        if rec_ndt_raw is not None:
            exec.add_node(rec_ndt_raw)

        orchestrator.get_logger().info('Recording (shared stamp window)...')
        wall_start = time.monotonic()
        win_dur = float(end_stamp) - float(start_stamp)
        wall_timeout = wall_start + max(240.0, win_dur + 180.0)
        interrupted = False
        try:
            while time.monotonic() < wall_timeout:
                exec.spin_once(timeout_sec=0.05)
                all_done = rec_fastlio.done() and rec_ndt.done() and rec_ekf.done()
                if rec_ndt_raw is not None:
                    all_done = all_done and rec_ndt_raw.done()
                if all_done:
                    break
        except KeyboardInterrupt:
            interrupted = True
            orchestrator.get_logger().warn('Interrupted by user; saving partial recordings.')

        rec_fastlio.save()
        rec_ndt.save()
        rec_ekf.save()
        if rec_ndt_raw is not None:
            rec_ndt_raw.save()
        if interrupted:
            # User asked to stop early; keep CSVs/meta and skip compare/align unless enough overlap exists.
            nr = len(rec_ndt_raw._rows) if rec_ndt_raw else -1
            if len(rec_fastlio._rows) < 3 or len(rec_ndt._rows) < 3 or len(rec_ekf._rows) < 3:
                orchestrator.get_logger().warn(
                    'Not enough rows after interruption for compare/align. '
                    f'rows: fastlio={len(rec_fastlio._rows)}, ndt={len(rec_ndt._rows)}, ekf={len(rec_ekf._rows)}, '
                    f'ndt_raw={nr}'
                )
                return

        class _Args:
            pass

        ndt_lbl_sm = 'NDT-smoothed' if rec_ndt_raw is not None else 'NDT'

        c1 = _Args()
        c1.ref = f'{fastlio_csv}:FAST-LIO'
        c1.test = f'{ndt_csv}:{ndt_lbl_sm}'
        c1.out_prefix = str(out_dir / 'fastlio_vs_ndt')
        cmd_compare(c1)

        c2 = _Args()
        c2.ref = f'{fastlio_csv}:FAST-LIO'
        c2.test = f'{ekf_csv}:EKF'
        c2.out_prefix = str(out_dir / 'fastlio_vs_ekf')
        cmd_compare(c2)

        a1 = _Args()
        a1.ref = f'{fastlio_csv}:FAST-LIO'
        a1.test = f'{ndt_csv}:{ndt_lbl_sm}'
        a1.out_prefix = str(out_dir / 'align_ndt')
        a1.samples = 600
        cmd_align_xy(a1)

        a2 = _Args()
        a2.ref = f'{fastlio_csv}:FAST-LIO'
        a2.test = f'{ekf_csv}:EKF'
        a2.out_prefix = str(out_dir / 'align_ekf')
        a2.samples = 600
        cmd_align_xy(a2)

        if rec_ndt_raw is not None and len(rec_ndt_raw._rows) >= 3:
            c_raw = _Args()
            c_raw.ref = f'{fastlio_csv}:FAST-LIO'
            c_raw.test = f'{ndt_raw_csv}:NDT-raw'
            c_raw.out_prefix = str(out_dir / 'fastlio_vs_ndt_raw')
            cmd_compare(c_raw)

            a_raw = _Args()
            a_raw.ref = f'{fastlio_csv}:FAST-LIO'
            a_raw.test = f'{ndt_raw_csv}:NDT-raw'
            a_raw.out_prefix = str(out_dir / 'align_ndt_raw')
            a_raw.samples = 600
            cmd_align_xy(a_raw)

            orch = orchestrator.get_logger()
            orch.info(f'Cross-check overlays: smoothed={out_dir / "align_ndt_overlay.png"} '
                      f'raw={out_dir / "align_ndt_raw_overlay.png"}')

        win_meta = {
            'rec_start_stamp_sec': float(start_stamp),
            'rec_end_stamp_sec': float(end_stamp),
            'used_fixed_stamp_window': rs is not None,
            'topic_first_stamp_sec': {
                'fastlio': float(t_fastlio0),
                'ndt': float(t_ndt0),
                'ekf': float(t_ekf0),
                'ndt_raw': float(t_raw0) if t_raw0 is not None else None,
            },
        }
        (out_dir / 'auto_fastlio_window.json').write_text(
            json.dumps(win_meta, indent=2) + '\n', encoding='utf-8'
        )
        orchestrator.get_logger().info(
            f'Done. Outputs in {out_dir} (stamp window saved to auto_fastlio_window.json)'
        )
    finally:
        for n in (rec_fastlio, rec_ndt, rec_ekf, rec_ndt_raw, orchestrator):
            try:
                if n is not None:
                    n.destroy_node()
            except Exception:
                pass
        try:
            if exec is not None:
                exec.shutdown(timeout_sec=0.5)
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def _parse_compare_header(text: str) -> dict:
    d = {}
    m = re.search(
        r'Path length \(m\):\s+ref=([\d.]+),\s+test=([\d.]+),\s+ratio test/ref=([\d.]+)',
        text,
    )
    if m:
        d['plen_ratio'] = float(m.group(3))
    m = re.search(
        r'Displacement \(m\):\s+ref=([\d.]+),\s+test=([\d.]+),\s+ratio test/ref=([\d.]+)',
        text,
    )
    if m:
        d['disp_ratio'] = float(m.group(3))
    m = re.search(
        r'Yaw change \(deg\):\s+ref=([-\d.]+),\s+test=([-\d.]+),\s+diff=([+-]?[\d.]+)',
        text,
    )
    if m:
        d['yaw_diff_deg'] = float(m.group(3))
    return d


def _parse_align_header(text: str) -> dict:
    d = {}
    m = re.search(r'RMSE_xy \(after SE\(2\) fit\):\s+([\d.]+)\s+m', text)
    if m:
        d['rmse_aligned_m'] = float(m.group(1))
    m = re.search(r'RMSE_xy \(same t_rel, no fit\):\s+([\d.]+)\s+m', text)
    if m:
        d['rmse_raw_m'] = float(m.group(1))
    m = re.search(r'Fit: rotate test by theta=([+-]?[\d.]+)\s+deg', text)
    if m:
        d['theta_deg'] = float(m.group(1))
    return d


def cmd_tune_rollup(args):
    """Print a compact table from several ``auto_fastlio_vs_slam`` output directories."""
    rows = []
    for d in args.dirs:
        p = Path(d).expanduser().resolve()
        row = {'dir': str(p)}
        cmp_sm = p / 'fastlio_vs_ndt_report.txt'
        aln_sm = p / 'align_ndt_report.txt'
        cmp_raw = p / 'fastlio_vs_ndt_raw_report.txt'
        aln_raw = p / 'align_ndt_raw_report.txt'
        ekf_cmp = p / 'fastlio_vs_ekf_report.txt'
        aln_ekf = p / 'align_ekf_report.txt'

        if cmp_sm.is_file():
            row.update({f'ndt_{k}': v for k, v in _parse_compare_header(cmp_sm.read_text()).items()})
        if aln_sm.is_file():
            row.update({f'ndt_{k}': v for k, v in _parse_align_header(aln_sm.read_text()).items()})
        if cmp_raw.is_file():
            row.update({f'raw_{k}': v for k, v in _parse_compare_header(cmp_raw.read_text()).items()})
        if aln_raw.is_file():
            row.update({f'raw_{k}': v for k, v in _parse_align_header(aln_raw.read_text()).items()})
        if ekf_cmp.is_file():
            row.update({f'ekf_{k}': v for k, v in _parse_compare_header(ekf_cmp.read_text()).items()})
        if aln_ekf.is_file():
            row.update({f'ekf_{k}': v for k, v in _parse_align_header(aln_ekf.read_text()).items()})
        rows.append(row)

    # Header: union of keys except dir
    keys = sorted({k for r in rows for k in r if k != 'dir'})
    hdr = ['dir'] + keys
    lines = ['\t'.join(hdr)]
    for r in rows:
        lines.append(
            '\t'.join(
                str(r.get(k, '')) for k in hdr
            )
        )
    out_txt = '\n'.join(lines) + '\n'
    print(out_txt)
    if getattr(args, 'out', None):
        Path(args.out).write_text(out_txt)
        print(f'Wrote {args.out}')


def cmd_plot(args):
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError as exc:
        raise SystemExit('Install dependencies: pip install matplotlib numpy') from exc

    items = []
    for spec in args.inputs:
        if ':' in spec:
            p, lbl = spec.rsplit(':', 1)
        else:
            p, lbl = spec, Path(spec).stem
        ts, tr, xs, ys, y = load_csv(Path(p))
        if not xs:
            raise SystemExit(f'No data in {p}')
        items.append((lbl, tr, xs, ys, y))

    n = len(items)
    colors = plt.cm.tab10(np.linspace(0, 0.9, max(1, n)))
    fig = plt.figure(figsize=(5 * n, 7))
    gs = fig.add_gridspec(2, n, height_ratios=[2.2, 1.0], hspace=0.35)

    for i, (lbl, tr, xs, ys, y) in enumerate(items):
        ax = fig.add_subplot(gs[0, i])
        ax.plot(xs, ys, color=colors[i], linewidth=1.2)
        ax.scatter([xs[0]], [ys[0]], c='green', s=35)
        ax.scatter([xs[-1]], [ys[-1]], c='red', s=35)
        ax.set_title(f'XY - {lbl}')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='datalim')

    ay = fig.add_subplot(gs[1, :])
    for i, (lbl, tr, xs, ys, y) in enumerate(items):
        ay.plot(tr, np.unwrap(np.array(y)), color=colors[i], label=lbl)
    ay.set_title('Yaw vs relative time')
    ay.set_xlabel('t_rel (s)')
    ay.set_ylabel('yaw (rad, unwrapped)')
    ay.grid(True, alpha=0.3)
    ay.legend(loc='upper right')

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f'Saved plot: {out.resolve()}')


def cmd_compare(args):
    ref_path, ref_label = (args.ref.rsplit(':', 1) if ':' in args.ref else (args.ref, 'REF'))
    test_path, test_label = (args.test.rsplit(':', 1) if ':' in args.test else (args.test, 'TEST'))

    _, tr_r, xr, yr, yrad_r = load_csv(Path(ref_path))
    _, tr_t, xt, yt, yrad_t = load_csv(Path(test_path))
    if len(xr) < 2 or len(xt) < 2:
        raise SystemExit('Not enough samples in one of the files for compare.')

    disp_r = math.hypot(xr[-1] - xr[0], yr[-1] - yr[0])
    disp_t = math.hypot(xt[-1] - xt[0], yt[-1] - yt[0])
    plen_r = path_len(xr, yr)
    plen_t = path_len(xt, yt)
    yawchg_r = angle_wrap(yrad_r[-1] - yrad_r[0])
    yawchg_t = angle_wrap(yrad_t[-1] - yrad_t[0])

    disp_ratio = disp_t / disp_r if disp_r > 1e-6 else float('nan')
    path_ratio = plen_t / plen_r if plen_r > 1e-6 else float('nan')
    yaw_err_deg = math.degrees(angle_wrap(yawchg_t - yawchg_r))

    out_prefix = Path(args.out_prefix)
    png = out_prefix.with_name(out_prefix.name + '_xy_yaw.png')
    txt = out_prefix.with_name(out_prefix.name + '_report.txt')

    # Save plot using existing plot path
    ns = argparse.Namespace(inputs=[f'{ref_path}:{ref_label}', f'{test_path}:{test_label}'], out=str(png))
    cmd_plot(ns)

    report = [
        f'Reference: {ref_label} ({ref_path})',
        f'Test:      {test_label} ({test_path})',
        '',
        f'Window: ref [{tr_r[0]:.2f}, {tr_r[-1]:.2f}] s, test [{tr_t[0]:.2f}, {tr_t[-1]:.2f}] s',
        f'Displacement (m): ref={disp_r:.3f}, test={disp_t:.3f}, ratio test/ref={disp_ratio:.3f}',
        f'Path length (m):  ref={plen_r:.3f}, test={plen_t:.3f}, ratio test/ref={path_ratio:.3f}',
        f'Yaw change (deg): ref={math.degrees(yawchg_r):.2f}, test={math.degrees(yawchg_t):.2f}, diff={yaw_err_deg:+.2f}',
        '',
        'Heuristic flags:',
    ]
    if disp_ratio < 0.6:
        report.append('- Test displacement much smaller than reference (possible rotation-without-translation behavior).')
    if path_ratio > 1.6:
        report.append('- Test path significantly longer than reference (possible jitter / drift / smear).')
    if abs(yaw_err_deg) > 20.0:
        report.append('- Yaw change diverges from reference by >20 deg (possible frame/extrinsic/deskew issue).')
    if len(report) == 10:
        report.append('- No strong red flags by these simple metrics; inspect plot shape for local failures.')

    ref_meta = _read_record_meta(Path(ref_path))
    if ref_meta and ref_meta.get('tf_projection'):
        report.append('')
        report.append('Reference recording metadata (.record_meta.json):')
        req_p = ref_meta.get('output_parent_frame_requested', ref_meta.get('output_parent_frame'))
        eff_p = ref_meta.get('effective_spatial_parent', req_p)
        report.append(
            f"- TF projection: requested_parent={req_p!r}, effective_spatial_parent={eff_p!r}, "
            f"child={ref_meta.get('output_child_frame')!r}"
        )
        if ref_meta.get('used_odom_missing_fallback'):
            report.append(
                '- Reference XY is in **camera_init** (no `odom` on /tf during record). '
                'XY vs /lidar/odom still differs by a fixed world transform unless you add static odom->camera_init.'
            )
        report.append(
            f"- Rows pose-projected ok: {ref_meta.get('tf_rows_pose_projected_ok')}, "
            f"fallback raw: {ref_meta.get('tf_rows_fallback_raw')}, csv_rows: {ref_meta.get('csv_rows')}"
        )
        if int(ref_meta.get('tf_rows_fallback_raw', 0) or 0) > int(ref_meta.get('tf_rows_pose_projected_ok', 1) or 1):
            report.append(
                '- Warning: many TF fallbacks on reference — raw odometry frame may still leak into CSV; '
                'fix /tf before trusting XY vs test.'
            )

    txt.parent.mkdir(parents=True, exist_ok=True)
    txt.write_text('\n'.join(report) + '\n')
    print('\n'.join(report))
    print(f'\nSaved report: {txt.resolve()}')


def _sorted_tr_xy(tr: List[float], xs: List[float], ys: List[float]):
    try:
        import numpy as np
    except ImportError as exc:
        raise SystemExit('Install numpy for align_xy: pip install numpy') from exc
    tr_a = np.asarray(tr, dtype=np.float64)
    x_a = np.asarray(xs, dtype=np.float64)
    y_a = np.asarray(ys, dtype=np.float64)
    order = np.argsort(tr_a)
    return tr_a[order], x_a[order], y_a[order]


def _interp_common_time(
    tr_r, xr, yr, tr_t, xt, yt, n_samples: int
):
    import numpy as np

    tr_r, xr, yr = _sorted_tr_xy(tr_r, xr, yr)
    tr_t, xt, yt = _sorted_tr_xy(tr_t, xt, yt)
    t0 = max(float(tr_r[0]), float(tr_t[0]))
    t1 = min(float(tr_r[-1]), float(tr_t[-1]))
    if t1 - t0 < 0.25:
        raise SystemExit(
            f'Overlapping t_rel window too small: [{t0:.3f}, {t1:.3f}] s. '
            'Re-record with matching --start-sec/--end-sec on both topics.'
        )
    tr = np.linspace(t0, t1, int(n_samples))
    xr_i = np.interp(tr, tr_r, xr)
    yr_i = np.interp(tr, tr_r, yr)
    xt_i = np.interp(tr, tr_t, xt)
    yt_i = np.interp(tr, tr_t, yt)
    return tr, xr_i, yr_i, xt_i, yt_i


def _se2_align(px: 'np.ndarray', py: 'np.ndarray', qx: 'np.ndarray', qy: 'np.ndarray'):
    """Find R (2x2), t (2,) minimizing || [qx;qy] - R @ [px;py] - t ||_F (column vectors)."""
    import numpy as np

    P = np.vstack([px, py])  # 2 x N
    Q = np.vstack([qx, qy])
    pm = P.mean(axis=1)
    qm = Q.mean(axis=1)
    P0 = P - pm.reshape(2, 1)
    Q0 = Q - qm.reshape(2, 1)
    M = Q0 @ P0.T
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0.0:
        U[:, -1] *= -1.0
        R = U @ Vt
    t = qm - R @ pm
    theta = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return R, t, theta


def cmd_align_xy(args):
    """Time-sync ref vs test, fit SE(2) test→ref, report RMSE before/after, save overlay plot."""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError as exc:
        raise SystemExit('Install dependencies: pip install matplotlib numpy') from exc

    ref_path, ref_label = (args.ref.rsplit(':', 1) if ':' in args.ref else (args.ref, 'REF'))
    test_path, test_label = (args.test.rsplit(':', 1) if ':' in args.test else (args.test, 'TEST'))

    _, tr_r, xr, yr, _ = load_csv(Path(ref_path))
    _, tr_t, xt, yt, _ = load_csv(Path(test_path))
    if len(xr) < 3 or len(xt) < 3:
        raise SystemExit('Not enough samples for align_xy.')

    tr, qx, qy, px, py = _interp_common_time(tr_r, xr, yr, tr_t, xt, yt, int(args.samples))
    R, tvec, theta = _se2_align(px, py, qx, qy)
    P = np.vstack([px, py])
    Pal = R @ P + tvec.reshape(2, 1)
    err_raw = np.hypot(qx - px, qy - py)
    err_al = np.hypot(qx - Pal[0, :], qy - Pal[1, :])
    rmse_raw = float(np.sqrt(np.mean(err_raw**2)))
    rmse_al = float(np.sqrt(np.mean(err_al**2)))
    theta_deg = math.degrees(theta)
    tx, ty = float(tvec[0]), float(tvec[1])

    out_prefix = Path(args.out_prefix)
    txt = out_prefix.with_name(out_prefix.name + '_report.txt')
    png = out_prefix.with_name(out_prefix.name + '_overlay.png')

    lines = [
        f'align_xy: rigid SE(2) fit of TEST -> REF in the horizontal plane',
        f'Reference: {ref_label} ({ref_path})',
        f'Test:      {test_label} ({test_path})',
        '',
        f'Overlap t_rel: [{float(tr[0]):.3f}, {float(tr[-1]):.3f}] s, samples={len(tr)}',
        f'Fit: rotate test by theta={theta_deg:+.2f} deg, then translate by t=({tx:+.4f}, {ty:+.4f}) m',
        '(theta is applied about the centroid of the interpolated TEST path, then shift matches REF centroid)',
        '',
        f'RMSE_xy (same t_rel, no fit):     {rmse_raw:.4f} m',
        f'RMSE_xy (after SE(2) fit):       {rmse_al:.4f} m',
        f'Improvement factor (raw/aligned): {(rmse_raw / rmse_al) if rmse_al > 1e-9 else float("inf"):.2f}x',
        '',
        'How to read this:',
        '- If RMSE drops a lot after fit → mostly a **constant plane offset** (origin choice + heading bias).',
        '- If RMSE stays large → **path shape** differs (NDT/EKF dynamics, smear, wrong motion).',
        '- |theta| near 90/180 with big raw RMSE → check **base_link / lidar extrinsics / odom frame**.',
    ]
    txt.parent.mkdir(parents=True, exist_ok=True)
    txt.write_text('\n'.join(lines) + '\n')

    fig, ax = plt.subplots(1, 1, figsize=(7.5, 7))
    ax.plot(qx, qy, color='navy', linewidth=1.4, label=f'{ref_label} (ref)')
    ax.plot(px, py, color='tab:cyan', linewidth=1.1, alpha=0.85, label=f'{test_label} (raw)')
    ax.plot(Pal[0, :], Pal[1, :], color='tab:orange', linewidth=1.1, linestyle='--', label=f'{test_label} (SE(2) aligned)')
    ax.scatter([qx[0]], [qy[0]], c='green', s=40, zorder=5)
    ax.scatter([qx[-1]], [qy[-1]], c='red', s=40, zorder=5)
    ax.set_title(f'XY overlay  |  RMSE raw {rmse_raw:.3f} m → aligned {rmse_al:.3f} m  |  θ={theta_deg:+.1f}°')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(loc='best', fontsize=9)
    fig.tight_layout()
    png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png, dpi=150)
    plt.close(fig)

    print('\n'.join(lines))
    print(f'\nSaved: {txt.resolve()}')
    print(f'Saved: {png.resolve()}')


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    pr = sub.add_parser('record', help='Record one odometry topic into CSV')
    pr.add_argument('--topic', required=True, help='Odometry topic, e.g. /Odometry or /ekf/odom')
    pr.add_argument('--start-sec', type=float, default=30.0, help='Start window at t_rel >= this value')
    pr.add_argument('--end-sec', type=float, default=80.0, help='End window at t_rel <= this value')
    pr.add_argument(
        '--start-stamp-sec',
        type=float,
        default=None,
        help=(
            'Optional absolute filter: only accept messages with header stamp_sec >= this value. '
            'Use together with --end-stamp-sec. When set, --start-sec/--end-sec are ignored for filtering.'
        ),
    )
    pr.add_argument(
        '--end-stamp-sec',
        type=float,
        default=None,
        help=(
            'Optional absolute filter: only accept messages with header stamp_sec <= this value. '
            'Use together with --start-stamp-sec.'
        ),
    )
    pr.add_argument('-o', '--out', required=True, help='Output CSV path')
    pr.add_argument('--use-sim-time', action='store_true', help='Use when replaying bags with --clock')
    pr.add_argument('--best-effort', action='store_true', help='Try sensor QoS if no data with reliable')
    pr.add_argument(
        '--output-parent-frame',
        default='',
        help=(
            'If set (e.g. odom), TF-project each odometry pose from msg.header.frame_id into this frame '
            'before writing x,y,yaw. Use with FAST-LIO /Odometry + bag /tf (camera_init must connect to odom).'
        ),
    )
    pr.add_argument(
        '--compose-output-child',
        default='',
        help=(
            'If set (e.g. base_link), multiply projected pose by TF(msg.child_frame_id -> this frame). '
            'Typical: --output-parent-frame odom --compose-output-child base_link for /Odometry.'
        ),
    )
    pr.add_argument(
        '--tf-timeout-sec',
        type=float,
        default=0.25,
        help='Per-lookup timeout for TF projection (seconds).',
    )
    pr.set_defaults(func=cmd_record)

    pa0 = sub.add_parser(
        'auto_fastlio_vs_slam',
        help='Auto-record FAST-LIO/NDT/EKF on same stamp window, then compare + align',
    )
    pa0.add_argument('--use-sim-time', action='store_true', help='Use when replaying bags with --clock')
    pa0.add_argument('--out-dir', default='/tmp/odom_auto', help='Output directory for CSVs/plots/reports')
    pa0.add_argument('--warmup-sec', type=float, default=5.0, help='Wait after all topics start before recording')
    pa0.add_argument('--duration-sec', type=float, default=50.0, help='Recording duration (seconds in message stamp)')
    pa0.add_argument(
        '--rec-start-stamp-sec',
        type=float,
        default=None,
        help=(
            'Optional absolute header-stamp (seconds) for recording start. '
            'Use with --rec-end-stamp-sec to reproduce the same window as a prior run; '
            'when set, --warmup-sec and --duration-sec are ignored for the window.'
        ),
    )
    pa0.add_argument(
        '--rec-end-stamp-sec',
        type=float,
        default=None,
        help='Absolute header-stamp (seconds) for recording end (must be > --rec-start-stamp-sec).',
    )
    pa0.add_argument('--wait-timeout-sec', type=float, default=120.0, help='Wall timeout waiting for topics')
    pa0.add_argument('--tf-timeout-sec', type=float, default=0.35, help='Per-lookup TF timeout for FAST-LIO projection')
    pa0.add_argument('--fastlio-topic', default='/Odometry', help='FAST-LIO odom topic')
    pa0.add_argument('--ndt-topic', default='/lidar/odom', help='NDT/LiDAR odom topic')
    pa0.add_argument('--ekf-topic', default='/ekf/odom', help='EKF odom topic')
    pa0.add_argument('--fastlio-output-parent-frame', default='odom', help='Project FAST-LIO pose into this parent frame')
    pa0.add_argument('--fastlio-compose-output-child', default='base_link', help='Compose FAST-LIO child into this frame')
    pa0.add_argument('--best-effort', action='store_true', help='Try sensor QoS if no data with reliable')
    pa0.add_argument(
        '--record-ndt-raw',
        action='store_true',
        help=(
            'Also record /lidar/odom_raw (pre-EMA) if present and generate fastlio_vs_ndt_raw + align_ndt_raw plots.'
        ),
    )
    pa0.add_argument(
        '--ndt-raw-topic',
        default='/lidar/odom_raw',
        help='Raw NDT odometry topic (only used with --record-ndt-raw)',
    )
    pa0.set_defaults(func=cmd_auto_fastlio_vs_slam)

    pru = sub.add_parser(
        'tune_rollup',
        help='Tabulate metrics from several auto_fastlio_vs_slam output directories',
    )
    pru.add_argument(
        'dirs',
        nargs='+',
        help='Paths like /tmp/ndt_matrix_A /tmp/ndt_matrix_B',
    )
    pru.add_argument('-o', '--out', default='', help='Optional TSV output path')
    pru.set_defaults(func=cmd_tune_rollup)

    pp = sub.add_parser('plot', help='Plot XY side-by-side + yaw overlay from CSV files')
    pp.add_argument('-i', '--inputs', action='append', required=True, help='FILE:LABEL (repeat)')
    pp.add_argument('-o', '--out', default='odom_compare.png', help='Output plot path')
    pp.set_defaults(func=cmd_plot)

    pc = sub.add_parser('compare', help='Plot + numeric compare for 2 runs')
    pc.add_argument('--ref', required=True, help='Reference CSV (usually FAST-LIO), format FILE[:LABEL]')
    pc.add_argument('--test', required=True, help='Test CSV (your SLAM), format FILE[:LABEL]')
    pc.add_argument('--out-prefix', default='compare', help='Output prefix for png/txt')
    pc.set_defaults(func=cmd_compare)

    pa = sub.add_parser(
        'align_xy',
        help='SE(2) fit of test XY to ref (same t_rel); RMSE before/after + overlay plot',
    )
    pa.add_argument('--ref', required=True, help='Reference CSV, format FILE[:LABEL]')
    pa.add_argument('--test', required=True, help='Test CSV, format FILE[:LABEL]')
    pa.add_argument(
        '--out-prefix',
        default='xy_align',
        help='Output prefix for *_report.txt and *_overlay.png',
    )
    pa.add_argument(
        '--samples',
        type=int,
        default=600,
        help='Number of time samples in overlap window (linear interp each series)',
    )
    pa.set_defaults(func=cmd_align_xy)

    return p


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
