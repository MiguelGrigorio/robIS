"""Observe the native Gazebo pose for simulation checks."""
import json
import math
import subprocess


def gazebo_pose(world, robot):
    result = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "--json-output",
         "-t", f"/world/{world}/dynamic_pose/info"],
        check=True, capture_output=True, text=True, timeout=10,
    )
    # gz topic can flush an extra queued sample even with -n 1. Decode the
    # complete JSON stream; selecting its latest robot pose also handles
    # pretty-printed packets without dropping malformed trailing data.
    decoder = json.JSONDecoder()
    remaining, packets = result.stdout.lstrip(), []
    while remaining:
        packet, end = decoder.raw_decode(remaining)
        packets.append(packet)
        remaining = remaining[end:].lstrip()
    pose = next((item for packet in reversed(packets) for item in packet['pose']
                 if item.get('name') == robot), None)
    if pose is None:
        raise RuntimeError(f'Robô {robot!r} ausente da telemetria do Gazebo')
    q = pose["orientation"]
    x, y, z, w = (q.get(key, 0.0) for key in ("x", "y", "z", "w"))
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = math.asin(max(-1, min(1, 2 * (w*y - z*x))))
    position = pose["position"]
    if not all(math.isfinite(value) for value in (roll, pitch, *position.values())):
        raise RuntimeError("Posição ou inclinação inválida no Gazebo")
    # A smoke-test bound against falling/tipping, not a physical calibration.
    if max(abs(roll), abs(pitch)) > math.radians(15):
        raise RuntimeError("Inclinação excessiva do robô no Gazebo")
    if abs(position.get("z", 0)) > 0.05:
        raise RuntimeError("Robô não assentou próximo ao chão")
    return {"position": position, "roll_rad": roll, "pitch_rad": pitch}
