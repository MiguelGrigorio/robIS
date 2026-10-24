"""Shared preparation of the headless static-object simulation and its LiDAR map."""
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

from .fixed_camera_setup import add_fixed_cameras, write_fixture_configuration
from .fixed_camera import WORLD, mapping


def prepare(root):
    root.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory('robis_gazebo'))
    tree = ET.parse(share/'worlds/labsea_semantic.world')
    world = tree.getroot().find('world')
    world.set('name', WORLD)
    _, bridges, hashes = add_fixed_cameras(world, Path('/robis_ws/config/calib'), update_rate=2.)
    world_path = root/'world.sdf'
    tree.write(world_path, encoding='utf-8', xml_declaration=True)
    bridge_path = root/'bridge.yaml'
    bridge_path.write_text(yaml.safe_dump(bridges))
    mapped = mapping(root, world_path)
    time.sleep(2)
    config = root/'calibrated'
    config.mkdir()
    write_fixture_configuration(config, Path('/robis_ws/config/calib'), root/'map.pgm')
    return {'mapping': mapped, 'calibration_sha256': hashes, 'world': str(world_path),
            'bridge': str(bridge_path), 'configuration': str(config/'config.yaml')}
