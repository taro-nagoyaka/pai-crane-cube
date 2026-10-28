"""Generate an isolated ROS package and launch based on the installed examples."""
import ast
from pathlib import Path
import shutil


def patch_launch(source):
    tree = ast.parse(source)
    changed = 0
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name) or node.func.id != 'Node':
            continue
        keywords = {k.arg: k for k in node.keywords}
        package = keywords.get('package')
        if package is None or not isinstance(package.value, ast.Constant) or package.value.value != 'crane_plus_examples':
            continue
        package.value = ast.Constant('crane_cube_teacher')
        keywords['executable'].value = ast.Constant('lift_teacher')
        if 'name' in keywords:
            keywords['name'].value = ast.Constant('cube_lift_teacher')
        changed += 1
    if changed != 1:
        raise ValueError('Installed example.launch.py differs: expected exactly one example Node')
    ast.fix_missing_locations(tree)
    return ('# Derived from CRANE+ example.launch.py, Copyright 2020 RT Corporation.\n'
            '# Apache-2.0: http://www.apache.org/licenses/LICENSE-2.0\n'
            '# AS IS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.\n'
            '# Modified package/executable; other parameters are preserved.\n' + ast.unparse(tree) + '\n')


def main():
    from ament_index_python.packages import get_package_share_directory
    root = Path(__file__).resolve().parent
    share = Path(get_package_share_directory('crane_plus_examples'))
    source = (share / 'launch/example.launch.py').read_text()
    generated = root / 'generated'
    generated.mkdir(exist_ok=True)
    (generated / 'lift_teacher.launch.py').write_text(patch_launch(source))
    package = root / 'teacher_ws/src/crane_cube_teacher'
    package.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(root / 'lift_teacher.cpp', package / 'lift_teacher.cpp')
    (package / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.8)
project(crane_cube_teacher)
set(CMAKE_CXX_STANDARD 14)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_executable(lift_teacher lift_teacher.cpp)
ament_target_dependencies(lift_teacher rclcpp moveit_ros_planning_interface moveit_msgs geometry_msgs trajectory_msgs std_msgs tf2_geometry_msgs)
install(TARGETS lift_teacher DESTINATION lib/${PROJECT_NAME})
ament_package()
''')
    (package / 'package.xml').write_text('''<?xml version="1.0"?>
<package format="3"><name>crane_cube_teacher</name><version>0.1.0</version>
<description>CRANE+ Gazebo lift-only teacher with reference trajectory output</description>
<maintainer email="maintainer@example.invalid">Local workspace maintainer</maintainer>
<license>Apache-2.0</license><buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend><depend>moveit_ros_planning_interface</depend>
<depend>moveit_msgs</depend>
<depend>geometry_msgs</depend><depend>trajectory_msgs</depend><depend>std_msgs</depend>
<depend>tf2_geometry_msgs</depend>
<export><build_type>ament_cmake</build_type></export></package>
''')
    print('Generated teacher package and launch. Original CRANE+ package unchanged.')


if __name__ == '__main__':
    main()
