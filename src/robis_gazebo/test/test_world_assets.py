"""Structural tests for self-contained Gazebo worlds and model provenance."""

from pathlib import Path
import hashlib
import json
import importlib.util
import math
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
MODELS = PACKAGE / "models"


def test_all_worlds_and_models_are_valid_xml():
    for filename in list((PACKAGE / "worlds").glob("*.world")) + list(
        MODELS.glob("*/model.sdf")
    ):
        ET.parse(filename)


def test_semantic_hall_has_expected_queryable_classes():
    root = ET.parse(PACKAGE / "worlds" / "semantic_hall.world").getroot()
    labels = {}
    for model in root.findall(".//world/model"):
        plugin = model.find("plugin[@filename='gz-sim-label-system']")
        if plugin is not None:
            labels[model.attrib["name"]] = int(plugin.findtext("label"))
    assert labels == {
        "chair_east": 1,
        "chair_west": 1,
        "couch": 2,
        "dining_table": 3,
        "bed": 4,
        "toilet": 5,
    }


def test_referenced_local_models_exist_and_have_provenance():
    for world in (PACKAGE / "worlds").glob("*.world"):
        root = ET.parse(world).getroot()
        for uri in root.findall(".//include/uri"):
            if not uri.text.startswith("model://robis_"):
                continue
            model = MODELS / uri.text[len("model://"):]
            assert (model / "model.sdf").is_file(), uri.text
            assert (model / "model.config").is_file(), uri.text
            assert (model / "PROVENANCE.md").is_file(), uri.text


def test_cmd_vel_uses_dedicated_typed_bridge():
    bridge = yaml.safe_load((PACKAGE / "config" / "bridge_topics.yaml").read_text())
    assert all(item.get("ros_topic_name") != "/cmd_vel" for item in bridge)
    launch = (PACKAGE / "launch" / "simulation.launch.py").read_text()
    assert "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" in launch


def test_labsea_import_preserves_source_and_separate_object_pivots():
    model = MODELS / 'robis_labsea'
    manifest = json.loads((model / 'import_manifest.json').read_text())
    assert hashlib.sha256((model / 'source/labsea.glb').read_bytes()).hexdigest() == manifest['source_sha256']
    chairs = [item for item in manifest['objects'] if item['kind'] == 'chair']
    assert len(chairs) == 13
    assert len(manifest['objects']) == 75  # structure + 74 groups in the updated GLB
    points = [list(map(float, row.split()[1:])) for row in
              (model / 'meshes/Chair_001.obj').read_text().splitlines() if row.startswith('v ')]
    # A chair stays centred on its own pivot and stands upright at metre scale.
    assert max(abs(point[0]) for point in points) < 0.4
    assert 0 <= min(point[2] for point in points) < 0.05
    assert 0.9 < max(point[2] for point in points) < 1.1


def test_labsea_embedded_textures_are_portable():
    meshes = MODELS / 'robis_labsea/meshes'
    for material in meshes.glob('*.mtl'):
        for row in material.read_text().splitlines():
            if row.startswith(('map_Kd ', 'map_bump ')):
                filename = row.split(maxsplit=1)[1]
                assert not filename.startswith('*')
                assert (meshes / filename).is_file()


def test_labsea_controlled_layout_keeps_structure_and_independent_furniture():
    root = ET.parse(PACKAGE / 'worlds/labsea_semantic.world').getroot()
    models = {item.attrib['name']: item for item in root.findall('world/model')}
    assert set(models) == {'structure', 'Chair_001', 'Chair_006', 'Table_001', 'Cabinet_001'}
    for model in models.values():
        assert model.find('link/collision/geometry/mesh/uri') is not None
    moved = ET.parse(PACKAGE / 'worlds/labsea_rearranged.world').getroot()
    changed = moved.find("world/model[@name='Chair_001']")
    assert changed.findtext('pose') != models['Chair_001'].findtext('pose')


def test_labsea_axis_conversion_preserves_metric_pose_and_heading():
    path = PACKAGE.parents[1] / 'extras/scripts/import_labsea.py'
    if not path.is_file():
        import pytest
        pytest.skip('Private asset conversion tooling is not present in this checkout')
    spec = importlib.util.spec_from_file_location('labsea_import', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    pose = module.gazebo_pose({'translation': [2, 1, -3],
        'rotation': [0, math.sqrt(0.5), 0, math.sqrt(0.5)]})
    assert pose[:3] == [2, 3, 1]
    assert math.isclose(pose[3], 0, abs_tol=1e-7)
    assert math.isclose(pose[4], 0, abs_tol=1e-7)
    assert math.isclose(pose[5], math.pi/2, abs_tol=1e-7)
