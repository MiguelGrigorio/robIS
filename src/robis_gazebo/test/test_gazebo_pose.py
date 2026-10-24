"""Exercise native-pose observation against actual CLI output variations."""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest


path = Path(__file__).resolve().parents[1] / 'robis_gazebo/runtime/gazebo.py'
spec = importlib.util.spec_from_file_location('native_pose', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def packet(x, roll=0.):
    return {'pose': [{'name': 'robis', 'position': {'x': x, 'y': 0., 'z': 0.},
                     'orientation': {'x': math.sin(roll/2), 'w': math.cos(roll/2)}}]}


@pytest.mark.parametrize('indent', [None, 2])
def test_extra_queued_cli_sample_uses_latest_pose(indent):
    output = '\n'.join(json.dumps(packet(x), indent=indent) for x in [0., .5]) + '\n'
    with patch.object(module.subprocess, 'run', return_value=SimpleNamespace(stdout=output)):
        assert module.gazebo_pose('labsea', 'robis')['position']['x'] == .5


def test_malformed_extra_output_is_not_silently_ignored():
    output = json.dumps(packet(0.)) + '\n{broken'
    with patch.object(module.subprocess, 'run', return_value=SimpleNamespace(stdout=output)):
        with pytest.raises(json.JSONDecodeError):
            module.gazebo_pose('labsea', 'robis')


def test_latest_pose_still_checks_for_tipping():
    output = json.dumps(packet(0.)) + '\n' + json.dumps(packet(.5, math.pi/2))
    with patch.object(module.subprocess, 'run', return_value=SimpleNamespace(stdout=output)):
        with pytest.raises(RuntimeError, match='Inclinação'):
            module.gazebo_pose('labsea', 'robis')


def test_missing_robot_is_reported_explicitly():
    with patch.object(module.subprocess, 'run', return_value=SimpleNamespace(stdout='{"pose": []}')):
        with pytest.raises(RuntimeError, match='ausente'):
            module.gazebo_pose('labsea', 'robis')
