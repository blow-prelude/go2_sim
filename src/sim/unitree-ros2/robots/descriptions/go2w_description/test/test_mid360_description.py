import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import pytest
from rclpy.utilities import remove_ros_args
import resource_retriever
import xacro


PACKAGE_DIR = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def robot():
    description = xacro.process_file(
        str(PACKAGE_DIR / 'urdf' / 'go2w_description.urdf.xacro'))
    return ET.fromstring(description.toxml())


def test_mid360_is_mounted_above_head(robot):
    assert robot.find("link[@name='livox']") is not None
    joint = robot.find("joint[@name='livox_joint']")
    assert joint is not None
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'mid360_mount'
    assert joint.find('child').get('link') == 'livox'
    origin = joint.find('origin')
    assert [float(v) for v in origin.get('xyz').split()] == pytest.approx([0, 0, 0.049])
    assert [float(v) for v in origin.get('rpy').split()] == pytest.approx([0, 0, 0])
    link = robot.find("link[@name='livox']")
    assert link.find('inertial') is None
    assert link.find('collision') is None


def test_mid360_mount_sits_on_head_boss_without_gaps(robot):
    mount = robot.find("link[@name='mid360_mount']")
    assert mount is not None
    assert mount.find('inertial') is None
    assert mount.find('collision') is None
    size = [float(v) for v in mount.find('visual/geometry/box').get('size').split()]
    assert size == pytest.approx([0.10, 0.08, 0.008])
    joint = robot.find("joint[@name='mid360_mount_joint']")
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'Head_upper'
    assert joint.find('child').get('link') == 'mid360_mount'
    position = [float(v) for v in joint.find('origin').get('xyz').split()]
    assert position == pytest.approx([-0.064, 0, 0.0828])
    head_z = float(robot.find("joint[@name='Head_upper_joint']/origin").get('xyz').split()[2])
    assert head_z + position[2] - size[2] / 2 == pytest.approx(0.0888)
    sensor_z = float(robot.find("joint[@name='livox_joint']/origin").get('xyz').split()[2])
    mesh_z = float(robot.find("link[@name='livox']/visual/origin").get('xyz').split()[2])
    assert sensor_z + mesh_z == pytest.approx(size[2] / 2)


def test_mid360_pointcloud_and_resources(robot):
    sensor = robot.find("gazebo[@reference='livox']/sensor")
    assert sensor is not None
    assert sensor.get('name') == 'livox'
    assert float(sensor.findtext('update_rate')) == 10
    plugin = sensor.find("plugin[@filename='libmid360_plugin.so']")
    assert plugin is not None
    assert plugin.findtext('topic') == '/livox/lidar'
    assert Path(plugin.findtext('csv_file_name')).is_file()
    meshes = robot.findall("link[@name='livox']//mesh")
    assert len(meshes) == 1
    assert all(resource_retriever.get(mesh.get('filename')) for mesh in meshes)


def test_mid360_internal_imu(robot):
    assert robot.find("link[@name='livox_imu']") is not None
    joint = robot.find("joint[@name='livox_imu_joint']")
    assert joint is not None
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'livox'
    assert joint.find('child').get('link') == 'livox_imu'
    plugin = robot.find(
        "gazebo[@reference='livox_imu']/sensor/plugin[@filename='libgazebo_ros_imu_sensor.so']")
    assert plugin is not None
    assert plugin.findtext('frame_name') == 'livox_imu'
    assert plugin.findtext('ros/remapping') == '~/out:=/livox/imu'


@pytest.mark.parametrize('name, xyz, rpy', [
    ('imu', [-0.02557, 0, 0.04232], [0, 0, 0]),
    ('lidar', [0.28945, 0, -0.046825], [3.141592653589793, 2.8782, 0]),
    ('camera', [0.325, 0, 0.043], [0, 0, 0]),
])
def test_original_sensor_mounts_are_unchanged(robot, name, xyz, rpy):
    assert robot.find(f"link[@name='{name}']") is not None
    assert robot.find(f"gazebo[@reference='{name}']/sensor") is not None
    joint = robot.find(f"joint[@name='{name}_joint']")
    assert joint.find('parent').get('link') == 'body'
    origin = joint.find('origin')
    assert [float(v) for v in origin.get('xyz').split()] == pytest.approx(xyz)
    assert [float(v) for v in origin.get('rpy').split()] == pytest.approx(rpy)


def test_link_and_joint_names_are_unique(robot):
    for kind in ['link', 'joint']:
        names = [element.get('name') for element in robot.findall(kind)]
        assert len(names) == len(set(names))


def test_mid360_runtime_dependencies_are_declared():
    package = ET.parse(PACKAGE_DIR / 'package.xml')
    dependencies = {dep.text for dep in package.findall('exec_depend')}
    assert {'mid360_simulation', 'xacro'} <= dependencies


def test_gazebo_control_can_parse_robot_description():
    description = xacro.process_file(
        str(PACKAGE_DIR / 'urdf' / 'go2w_description.urdf.xacro')).toxml()
    assert remove_ros_args([
        'gazebo_ros2_control', '--ros-args', '--param',
        'robot_description:=' + description]) == ['gazebo_ros2_control']


@pytest.mark.parametrize('filename', ['gazebo.launch.py', 'go2w_rviz.launch.py'])
def test_launch_passes_robot_description_as_string(filename, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'go2w_test_launch', PACKAGE_DIR / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    descriptions = []

    def capture_node(**kwargs):
        descriptions.extend(
            param for param in kwargs.get('parameters', [])
            if isinstance(param, dict) and 'robot_description' in param)
        return Node(**kwargs)

    monkeypatch.setattr(module, 'Node', capture_node)
    module.generate_launch_description()
    assert len(descriptions) == 1
    parameters = evaluate_parameters(LaunchContext(), normalize_parameters(descriptions))
    description = parameters[0]['robot_description']
    assert isinstance(description, str)
    assert ET.fromstring(description).find("link[@name='livox']") is not None
