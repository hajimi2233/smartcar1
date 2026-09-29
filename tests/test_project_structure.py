"""Validate source migration and launch wiring without requiring ROS."""
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = {ET.parse(p).getroot().findtext('name'): p.parent
            for p in (ROOT/'src').glob('*/package.xml')}


class StructureTests(unittest.TestCase):
    def test_local_launch_includes_and_required_arguments_resolve(self):
        for path in (ROOT/'src').rglob('*.launch'):
            document = ET.parse(path).getroot()
            for include in document.iter('include'):
                match = re.match(r'\$\(find ([^)]+)\)/(.*)', include.attrib['file'])
                if not match or match[1] not in PACKAGES:
                    continue
                target = PACKAGES[match[1]]/match[2]
                self.assertTrue(target.is_file(), str(target))
                arguments = {a.attrib['name']: a.attrib for a in ET.parse(target).getroot().findall('arg')}
                supplied = {a.attrib['name'] for a in include.findall('arg')}
                required = {n for n,a in arguments.items() if 'default' not in a and 'value' not in a}
                self.assertTrue(required <= supplied, str(path)+' missing '+str(required-supplied))
                self.assertTrue(supplied <= set(arguments), str(path)+' unknown arguments')

    def test_local_launch_nodes_exist(self):
        for path in (ROOT/'src').rglob('*.launch'):
            for node in ET.parse(path).getroot().iter('node'):
                pkg = node.attrib['pkg']
                if pkg in PACKAGES:
                    self.assertTrue((PACKAGES[pkg]/'scripts'/node.attrib['type']).is_file(), str(path))

    def test_python_build_sources_exist(self):
        for pkg in PACKAGES.values():
            cmake=(pkg/'CMakeLists.txt').read_text(encoding='utf-8')
            for rel in re.findall(r'scripts/[A-Za-z_0-9]+\.py',cmake):
                self.assertTrue((pkg/rel).is_file(),str(pkg/rel))

    def test_tf_stages_do_not_spawn_simulation_sensors(self):
        for name in ('smartcar_mapping','smartcar_localization','smartcar_navigation','smartcar_mission'):
            for path in PACKAGES[name].rglob('*.launch'):
                for node in ET.parse(path).getroot().iter('node'):
                    self.assertNotEqual(node.attrib['pkg'],'smartcar_sim')
                    self.assertNotEqual(node.attrib['pkg'],'gazebo_ros')
        sim=ET.parse(PACKAGES['smartcar_bringup']/'launch/drivers_sim.launch')
        self.assertFalse(any('localization' in n.attrib['file'] for n in sim.iter('include')))

    def test_source_rebuild_does_not_depend_on_workspace_archive(self):
        docker=(ROOT/'docker/Dockerfile').read_text()
        self.assertIn('COPY --chown=hajimi:hajimi src ',docker)
        self.assertNotIn('restore_workspace',docker)
        self.assertNotIn('install_overlay',docker)


if __name__=='__main__': unittest.main()
