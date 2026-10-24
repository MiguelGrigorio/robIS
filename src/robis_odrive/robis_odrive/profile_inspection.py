"""Inspect the commissioned ODrive through USB without issuing any writes."""

import argparse
import json
from pathlib import Path

from .drive_safety import SafetyProfile, property_value


def inspect(device, profile):
    profile.verify(device)
    return {
        'serial': format(int(device.serial_number), 'X'),
        'firmware': '.'.join(str(v) for v in profile.firmware),
        'firmware_unreleased': int(device.fw_version_unreleased),
        'profile_matches': True,
        'host_forward_motor_signs': profile.motor_signs,
        'axes': [{
            'state': int(axis.current_state),
            'properties': {key: property_value(axis, key) for key in profile.properties},
            'position_turns': float(axis.encoder.pos_estimate),
        } for axis in (device.axis0, device.axis1)],
        'vbus_voltage': float(device.vbus_voltage),
        'writes_issued': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()
    path = args.profile
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('robis_odrive')) / 'params/ground_05mps.json'
    profile = SafetyProfile.load(path)
    import odrive
    device = odrive.find_any(serial_number=profile.serial, timeout=args.timeout)
    print(json.dumps(inspect(device, profile), indent=2))


if __name__ == '__main__':
    main()
