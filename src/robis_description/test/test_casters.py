"""Mechanical invariants of the expanded caster model, independent of rendering."""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


@pytest.fixture(scope="module")
def model():
    share = Path(get_package_share_directory("robis_description"))
    result = subprocess.run(
        ["xacro", str(share / "urdf" / "robis.urdf.xacro"), "sim_flavor:=gz"],
        check=True, capture_output=True, text=True,
    )
    return ET.fromstring(result.stdout), yaml.safe_load(
        (share / "config" / "robot_parameters.yaml").read_text()
    )["physical"]


@pytest.mark.parametrize("side", ["front", "rear"])
def test_caster_has_passive_swivel_and_orthogonal_rolling_axis(model, side):
    root, _ = model
    swivel = root.find(f"joint[@name='{side}_caster_joint']")
    rolling = root.find(f"joint[@name='{side}_caster_wheel_joint']")
    assert swivel.attrib["type"] == rolling.attrib["type"] == "continuous"
    assert swivel.find("axis").attrib["xyz"] == "0 0 1"
    assert rolling.find("axis").attrib["xyz"] == "0 0 1"
    rotation = list(map(float, rolling.find("origin").attrib["rpy"].split()))
    assert math.isclose(rotation[0], -math.pi / 2)
    assert rotation[1:] == [0, 0]
    assert float(rolling.find("origin").attrib["xyz"].split()[0]) < 0
    for plugin in root.findall("gazebo/plugin"):
        for joint in plugin.findall("left_joint") + plugin.findall("right_joint"):
            assert "caster" not in joint.text


@pytest.mark.parametrize("side", ["front", "rear"])
def test_caster_preserves_mass_and_ground_clearance(model, side):
    root, physical = model
    mount = root.find(f"link[@name='{side}_caster_swivel']")
    wheel = root.find(f"link[@name='{side}_caster_wheel']")
    masses = [float(link.find("inertial/mass").attrib["value"]) for link in (mount, wheel)]
    assert all(value > 0 for value in masses)
    assert math.isclose(sum(masses), physical[f"caster_{side}_mass"])
    for link in (mount, wheel):
        inertia = link.find("inertial/inertia").attrib
        assert all(float(inertia[key]) > 0 for key in ("ixx", "iyy", "izz"))
    cylinder = wheel.find("collision/geometry/cylinder")
    assert cylinder is not None
    assert wheel.find("collision/geometry/sphere") is None
    radius = float(cylinder.attrib["radius"])
    joint = root.find(f"joint[@name='{side}_caster_wheel_joint']")
    z = float(joint.find("origin").attrib["xyz"].split()[2])
    assert math.isclose(physical["ground_clearance"] + z - radius, physical["caster_ground_gap"], abs_tol=1e-9)
    assert float(cylinder.attrib["length"]) > 0
