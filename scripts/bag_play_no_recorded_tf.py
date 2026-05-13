#!/usr/bin/env python3
"""
Run ``ros2 bag play`` on all topics **except** ``/tf`` and ``/tf_static``.

When replaying with your own ``ekf_node`` (and static TFs from ``launch_slam``), recorded TF
usually conflicts and produces ``TF_OLD_DATA ignoring data from the past for frame base_link``.

Usage:
  python3 scripts/bag_play_no_recorded_tf.py /path/to/bag --clock
  python3 scripts/bag_play_no_recorded_tf.py /path/to/bag -- --clock -r 1.2
"""
from __future__ import annotations

import os
import subprocess
import sys


def _topics_from_bag_info(bag: str) -> list[str]:
    p = subprocess.run(
        ['ros2', 'bag', 'info', bag],
        check=True,
        capture_output=True,
        text=True,
    )
    topics: list[str] = []
    for line in p.stdout.splitlines():
        if 'Topic:' not in line:
            continue
        rest = line.split('Topic:', 1)[1].strip()
        if not rest:
            continue
        topic = rest.split()[0]
        if topic.startswith('/'):
            topics.append(topic)
    return topics


def main() -> None:
    argv = sys.argv[1:]
    if not argv:
        print(
            'usage: python3 bag_play_no_recorded_tf.py BAG [ros2 bag play args...]\n'
            '  Example: python3 bag_play_no_recorded_tf.py ~/bags/session_01 --clock',
            file=sys.stderr,
        )
        sys.exit(1)
    bag = os.path.expanduser(str(argv[0]).strip())
    if not bag:
        print(
            'error: bag path is empty (e.g. `$BAG` was unset in this terminal).\n'
            '  export BAG=/path/to/your_bag_dir_or_file\n'
            '  python3 bag_play_no_recorded_tf.py "$BAG" --clock',
            file=sys.stderr,
        )
        sys.exit(1)
    if not os.path.exists(bag):
        print(f'error: bag path does not exist: {bag!r}', file=sys.stderr)
        sys.exit(1)
    forward = argv[1:]
    if forward[:1] == ['--']:
        forward = forward[1:]

    skip = {'/tf', '/tf_static'}
    all_topics = _topics_from_bag_info(bag)
    play = [t for t in all_topics if t not in skip]
    if not play:
        print('No topics left after removing /tf and /tf_static.', file=sys.stderr)
        sys.exit(1)

    cmd = ['ros2', 'bag', 'play', bag, *forward, '--topics', *play]
    print('Running:', ' '.join(cmd), file=sys.stderr)
    os.execvp('ros2', cmd)


if __name__ == '__main__':
    main()
