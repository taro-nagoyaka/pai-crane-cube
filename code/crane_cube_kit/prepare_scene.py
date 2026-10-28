"""Run with system Python in the user's sourced ROS 2 Humble container."""
import argparse
import ast
from pathlib import Path
import xml.etree.ElementTree as ET


def build_world(source, rtf):
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('SDF has no world')
    physics = world.find('physics')
    if physics is None:
        raise ValueError('SDF has no physics configuration')
    factor = physics.find('real_time_factor')
    if factor is None:
        factor = ET.SubElement(physics, 'real_time_factor')
    factor.text = str(rtf)
    if not any('systems::Sensors' in p.get('name', '') for p in world.findall('plugin')):
        plugin = ET.SubElement(world, 'plugin', {
            'filename': 'ignition-gazebo-sensors-system',
            'name': 'ignition::gazebo::systems::Sensors'})
        ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    for inc in world.findall('include'):
        uri = inc.findtext('uri', '')
        if 'Wood%20cube%205cm' in uri or 'Wood cube 5cm' in uri:
            name = inc.find('name')
            if name is None:
                name = ET.SubElement(inc, 'name')
            name.text = 'woodcube'
    camera = ET.fromstring('''
      <model name="training_camera">
        <static>true</static>
        <pose>0.45 -0.50 1.40 0 0.42 2.30</pose>
        <link name="camera_link">
          <sensor name="front_camera" type="camera">
            <always_on>true</always_on><update_rate>20</update_rate>
            <topic>/camera/front/image</topic>
            <camera><horizontal_fov>1.05</horizontal_fov>
              <image><width>320</width><height>240</height><format>R8G8B8</format></image>
              <clip><near>0.02</near><far>5</far></clip>
            </camera>
          </sensor>
        </link>
      </model>''')
    world.append(camera)
    ET.indent(tree, space='  ')
    return tree


def build_launch(source):
    tree = ast.parse(source)
    world_changed = bridge_changed = False
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'world_file' for t in node.targets):
            node.value = ast.parse("os.path.join(os.path.dirname(__file__), 'table_camera.sdf')", mode='eval').body
            world_changed = True
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'bridge' for t in node.targets):
            for kw in node.value.keywords:
                if kw.arg == 'arguments' and isinstance(kw.value, ast.List):
                    kw.value.elts.append(ast.Constant('/camera/front/image@sensor_msgs/msg/Image[ignition.msgs.Image'))
                    bridge_changed = True
    if not (world_changed and bridge_changed):
        raise ValueError('Launch structure differs: world_file / bridge.arguments not found')
    ast.fix_missing_locations(tree)
    notice = '''# Derived from CRANE+ launch, Copyright 2022 RT Corporation.
# Licensed under the Apache License, Version 2.0.
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an AS IS BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Modified: world file replaced; image bridge added. Original files unchanged.
'''
    return notice + ast.unparse(tree) + '\n'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--rtf', type=float, default=0.25)
    args = p.parse_args()
    if not 0 < args.rtf <= 1:
        p.error('--rtf must be in (0, 1]')
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('crane_plus_gazebo'))
    out = Path(__file__).resolve().parent / 'generated'
    out.mkdir(exist_ok=True)
    build_world(share / 'worlds/table.sdf', args.rtf).write(
        out / 'table_camera.sdf', encoding='utf-8', xml_declaration=True)
    (out / 'cube_camera.launch.py').write_text(build_launch(
        (share / 'launch/crane_plus_with_table.launch.py').read_text()), encoding='utf-8')
    print('Stop the previous Gazebo launch, then run:')
    print(f'ros2 launch {out / "cube_camera.launch.py"}')


if __name__ == '__main__':
    main()
