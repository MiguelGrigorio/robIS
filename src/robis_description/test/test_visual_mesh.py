"""Regressions found by rendering the updated CAD in Gazebo."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml


SHARE = Path(get_package_share_directory('robis_description'))


def test_collada_uses_unambiguous_interleaved_inputs_for_gazebo():
    root = ET.parse(SHARE / 'meshes/updated/body.dae')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    primitives = root.findall('.//c:polylist', ns)
    assert primitives
    for primitive in primitives:
        inputs = primitive.findall('c:input', ns)
        offsets = [int(item.attrib['offset']) for item in inputs]
        assert offsets == list(range(len(inputs)))
        counts = list(map(int, primitive.findtext('c:vcount', namespaces=ns).split()))
        indices = list(map(int, primitive.findtext('c:p', namespaces=ns).split()))
        assert len(counts) == int(primitive.attrib['count'])
        assert len(indices) == sum(counts) * len(inputs)
        assert all(index >= 0 for index in indices)


def test_optical_origin_matches_simulated_visual_without_changing_real_extrinsics():
    params = yaml.safe_load((SHARE / 'config/robot_parameters.yaml').read_text())
    physical, camera = params['physical'], params['sensors']['camera']
    for flavor in ('none', 'gz'):
        result = subprocess.run(['xacro', str(SHARE / 'urdf/robis.urdf.xacro'),
                                 'sim_flavor:=' + flavor],
                                check=True, capture_output=True, text=True)
        root = ET.fromstring(result.stdout)
        origin = root.find("joint[@name='camera_joint']/origin")
        xyz = list(map(float, origin.attrib['xyz'].split()))
        expected = ([camera['optical_origin_x'], 0, camera['optical_origin_z']]
                    if flavor == 'gz' else
                    [physical['camera_offset_x'], 0, physical['camera_offset_z']])
        assert xyz == expected
    # The supplied CAD lens centre is x=0.237 m. Its front face must not
    # render in front of the simulated optical origin.
    assert camera['optical_origin_x'] > 0.242
