#!/usr/bin/env python3
"""Keep the complete simulation alive for manual ROS navigation commands."""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import threading

from robis_semantic_navigation.stop import cancel_actions
from .fixed_camera import scene_session, start
from .scene import prepare


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--owner')
    parser.add_argument('--foxglove-port', type=int, default=8765)
    args = parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    root = args.directory/f'session-{stamp}-{os.getpid()}'
    ready = Path('/tmp/robis-semantic-ready')
    ready.unlink(missing_ok=True)
    stopped = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    # During mapping, normal SIGINT still interrupts the ROS wait immediately.
    report = {'ready': False, 'session': str(root)}
    try:
        prepared = prepare(root)
        if stopped.is_set():
            return
        with scene_session(root, Path(prepared['world']), Path(prepared['bridge']),
                           Path(prepared['configuration']), Path('/opt/robis-models/labsea-yolo26s-seg.pt'),
                           prepared['mapping']['checksum'], report) as (node, _, processes):
            signal.signal(signal.SIGINT, lambda *_: stopped.set())
            with ExitStack() as stack:
                bridge = start(stack, ['ros2', 'launch', 'robis_bringup', 'foxglove.launch.py',
                                       'use_sim_time:=true', 'port:='+str(args.foxglove_port)], root/'foxglove.log')
                report.update(ready=True, environment='labsea_fixed_fixture', prepared=prepared)
                (args.directory/'current.json').write_text(json.dumps(report, indent=2)+'\n')
                ready.touch()
                print('Simulação pronta. Use goto ou /navigate_to_pose; stop cancela as ações.', flush=True)
                try:
                    while not stopped.is_set():
                        for process in [*processes, bridge]:
                            if process.process.poll() is not None:
                                raise RuntimeError('Um processo da simulação encerrou; consulte os logs da sessão')
                        node.spin(.1)
                finally:
                    ready.unlink(missing_ok=True)
                    cancel_actions(node)
                    node.settle()
    finally:
        ready.unlink(missing_ok=True)
        if args.owner:
            uid, gid = map(int, args.owner.split(':'))
            for path in [args.directory, *args.directory.rglob('*')]:
                os.chown(path, uid, gid)


if __name__ == '__main__':
    main()
