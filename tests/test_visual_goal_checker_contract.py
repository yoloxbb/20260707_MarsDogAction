"""The visual Nav2 request selects the precise checker installed in both maps."""
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import pytest
import yaml

from marsdog_action_executor.adapters.nav2_goal_transport import Ros2Nav2GoalTransport


def test_visual_behavior_tree_is_carried_by_the_real_nav2_goal():
    from builtin_interfaces.msg import Time
    from nav2_msgs.action import NavigateToPose
    transport = object.__new__(Ros2Nav2GoalTransport)
    transport._nav_type = NavigateToPose
    transport._node = NS(get_clock=lambda: NS(now=lambda: NS(
        to_msg=lambda: Time())))
    tree = Path(__file__).parents[1] / 'config/navigate_to_visual_target.xml'
    transport._behavior_tree = str(tree)
    pose = {'header': {'frame_id': 'map'}, 'pose': {
        'position': {'x': 1., 'y': 2., 'z': 0.},
        'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}
    request = transport._make_goal(pose, 0.)
    assert request.behavior_tree == str(tree)
    assert request.pose.pose.position.x == 1.
    assert {e.attrib['goal_checker_id'] for e in ET.parse(tree).iter('FollowPath')} == {
        'visual_goal_checker'}


@pytest.mark.parametrize('name', ['nav.yaml', 'nav_voxel.yaml'])
def test_visual_checker_and_map_freshness_match_localization_requirements(name):
    bringup = Path('/home/cat/robot_ws/slam_ws/src/robot_slam_bringup')
    if not bringup.exists():
        pytest.skip('deployment contract requires the paired navigation workspace')
    config = yaml.safe_load((bringup / 'config' / name).read_text())
    controller = config['controller_server']['ros__parameters']
    assert 'visual_goal_checker' in controller['goal_checker_plugins']
    assert controller['visual_goal_checker']['xy_goal_tolerance'] == .05
    assert controller['general_goal_checker']['xy_goal_tolerance'] == .3
    costmap = config['global_costmap']['global_costmap']['ros__parameters']
    assert costmap['update_frequency'] == costmap['publish_frequency'] == 1.
