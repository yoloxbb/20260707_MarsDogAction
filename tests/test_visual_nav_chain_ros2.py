"""Opt-in ROS transport test with synthetic localization/Nav2 servers."""
import os
from pathlib import Path

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("MARSDOG_RUN_ROS_TESTS") != "1", reason="requires isolated ROS domain",
)


@pytest.mark.parametrize("route", ["owner", "human", "object", "animal"])
def test_visual_point_to_action_nav2_uses_one_correlated_goal(route):
    import threading
    import time
    from types import SimpleNamespace as NS

    import rclpy
    from nav2_msgs.action import NavigateToPose
    from person_3d_localization.srv import LocateFromBbox
    from rclpy.action import ActionServer
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from marsdog_vision_interaction.srv import VisionTask
    from marsdog_vision_interaction.core.target_localization import Ros2TargetLocalizationClient
    from marsdog_vision_interaction.nodes.vision_interaction_node import VisionInteractionNode
    from marsdog_action_executor.adapters.person_nav_approach_adapter import (
        PersonNavApproachAdapter, Ros2PersonApproachTransport,
    )

    rclpy.init()
    node = Node("test_visual_nav_chain")
    executor = MultiThreadedExecutor(num_threads=4)
    kind = "human" if route in {"owner", "human"} else route
    target_id = "test-epoch:" + ("human" if kind == "human" else "object") + ":1"
    from marsdog_action_executor.adapters.visual_nav_approach_adapter import VisualNavApproachAdapter

    calls = []
    behavior_tree = '' if route == 'owner' else str(
        Path(__file__).parents[1] / 'config/navigate_to_visual_target.xml')
    node._state_lock = threading.Lock()
    node._providers = {}
    node._latest_visual_source_image = {"width": 640, "height": 480, "stereo_split": False}
    body = {"vision_epoch": "test-epoch", "target_id": target_id, "target_type": kind,
            "tracking_state": "tracking", "geometry_source": "body", "motion_eligible": True,
            "confidence": .9, "last_seen_age_ms": 0., "bbox": [.25, .1, .5, .8]}
    node._latest_person_localization_snapshot = {"vision_epoch": "test-epoch", "human_candidates": [body]}
    node._target_localization_client = Ros2TargetLocalizationClient(node, timeout_sec=2.)
    node._target_current_timeout_sec = .35
    node._slam_service_type = LocateFromBbox
    node._slam_ready_timeout_sec = .1
    node._slam_response_timeout_sec = 2.
    node._slam_client = node.create_client(
        LocateFromBbox, '/person_3d_localization/locate_from_bbox',
        callback_group=MutuallyExclusiveCallbackGroup())
    node._target_manager = NS(get_localization_candidate=lambda *args, **kwargs: {
        'target': body, 'source': {
            'header': {'stamp': {'sec': node.get_clock().now().to_msg().sec,
                                 'nanosec': 0}, 'frame_id': 'color_optical'},
            'source_width': 640, 'source_height': 480,
            'view_width': 640, 'view_height': 480, 'view_split': False,
            'received_monotonic': time.monotonic(),
        }})
    node._latest_object_source_image = node._latest_visual_source_image
    node._latest_object_localization_snapshot = {"vision_epoch": "test-epoch", "sequence": 1, "objects": [body]}
    node._locate_target_once = lambda params: VisionInteractionNode._locate_target_once(node, params)
    node._locate_person_once = lambda params: VisionInteractionNode._locate_person_once(node, params)
    node._run_task = lambda kind, params: VisionInteractionNode._run_task(node, kind, params)

    def vision_task(request, response):
        node._latest_visual_snapshot_monotonic = time.monotonic()
        node._latest_person_localization_snapshot["header"] = {
            "stamp": node.get_clock().now().nanoseconds * 1e-9, "frame_id": "color_optical"}
        if kind != "human":
            node._latest_object_localization_monotonic = time.monotonic()
            header = node._latest_person_localization_snapshot["header"]
            node._latest_object_localization_snapshot["header"] = header
            body["header"], body["source_sequence"] = header, 1
        return VisionInteractionNode._handle_task(node, request, response)

    def localize(request, response):
        calls.append("locate")
        assert request.image_header.frame_id == "color_optical"
        assert (request.bbox.x_offset, request.bbox.y_offset, request.bbox.width, request.bbox.height) == (160, 48, 320, 384)
        response.success, response.status, response.navigation_required = True, 0, True
        response.person_point.header.frame_id = "map"
        response.person_point.point.x, response.person_point.point.y = 3., 2.
        response.navigation_goal.header.frame_id = "map"
        response.navigation_goal.pose.position.x, response.navigation_goal.pose.position.y = 1., 2.
        response.navigation_goal.pose.orientation.w = 1.
        return response

    def navigate(handle):
        calls.append("navigate")
        assert handle.request.behavior_tree == behavior_tree
        assert handle.request.pose.header.frame_id == "map"
        assert handle.request.pose.pose.position.x == 1.
        assert handle.request.pose.pose.position.y == 2.
        handle.succeed()
        return NavigateToPose.Result()

    services = [
        node.create_service(VisionTask, "/perception/vision/task", vision_task,
                            callback_group=MutuallyExclusiveCallbackGroup()),
        node.create_service(LocateFromBbox, "/person_3d_localization/locate_from_bbox", localize,
                            callback_group=MutuallyExclusiveCallbackGroup()),
    ]
    server = ActionServer(node, NavigateToPose, "/navigate_to_pose", execute_callback=navigate,
                          callback_group=MutuallyExclusiveCallbackGroup())
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        transport = Ros2PersonApproachTransport(
            node, server_timeout_sec=2., service_timeout_sec=4.,
            behavior_tree=behavior_tree)
        backend = NS(prepare_navigation=lambda: calls.append("prepare") or True,
                     finish_navigation=lambda: calls.append("finish") or True)
        if route == "owner":
            adapter = PersonNavApproachAdapter(transport, lambda _: None, chassis_backend=backend)
            adapter.update_visual({"vision_epoch": "test-epoch", "active_target": body})
            ctx = NS(resolved_behavior_name="come_to_owner", target=None, metadata={},
                     params={"strict_target_lock": True, "allow_target_switch": False})
            unit_id = "ACT_INTERACT_APPROACH_OWNER"
        else:
            behavior = "expressCuriosityWithHuman" if kind == "human" else "greetAnimal" if kind == "animal" else "inspectObject"
            adapter = VisualNavApproachAdapter(lambda _: None, transport=transport, chassis_backend=backend,
                behavior_policies={behavior: {"target_type": kind, "desired_distance_m": 1.5 if kind == "human" else 1.2 if kind == "animal" else .9,
                                             "minimum_safe_distance_m": .55}})
            visual = {"schema_version": 1, "header": {"stamp": 1., "frame_id": "color_optical"},
                      "vision_epoch": "test-epoch", "sequence": 1, "snapshot_id": "test-epoch:1",
                      "human_candidates" if kind == "human" else "tracked_objects": [body]}
            adapter.update_visual(visual)
            ctx = NS(resolved_behavior_name=behavior, target=body, metadata={}, params={})
            unit_id = "ACT_APPROACH_VISUAL_TARGET"
        result = adapter.execute_task({"unit_id": unit_id}, ctx, 8.)
        assert result.success, result
        assert calls == ["locate", "prepare", "navigate", "finish"]
        metadata = ctx.metadata["target_approach" if route == "owner" else "visual_target_approach"]
        assert metadata["target_id" if route == "owner" else "bound_target_id"] == target_id
        assert not adapter.recovery_required
    finally:
        server.destroy()
        for service in services:
            node.destroy_service(service)
        executor.shutdown(timeout_sec=2.)
        thread.join(timeout=2.)
        node.destroy_node()
        rclpy.shutdown()
