from marsdog_action_executor.adapters.attention_tracking_controller import (
    AttentionTrackingController,
)


def test_packaged_follow_configuration_uses_safe_faster_defaults():
    from pathlib import Path

    import yaml

    config_path = Path(__file__).parents[1] / "config" / "attention_tracking.yaml"
    parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "action_executor_node"
    ]["ros__parameters"]

    assert parameters["follow_linear_gain"] == 0.75
    assert parameters["follow_max_linear_x"] == 0.25
    assert parameters["follow_max_linear_accel"] == 0.35


def test_session_visual_centering_uses_angular_velocity_only():
    controller = AttentionTrackingController(wake_fallback_sec=0.0)
    controller.update_control({
        "enabled": True,
        "interaction_id": "session-1",
    }, now=1.0)
    controller.update_visual({
        "active_target": {
            "confidence": 0.9,
            "tracking_state": "tracking",
            "last_seen_age_ms": 10.0,
            "face_center": [0.75, 0.3],
            "body_center": [0.7, 0.5],
        }
    }, now=1.1)
    command = controller.command(now=1.2)
    assert command is not None
    assert command.linear_x == 0.0
    assert command.angular_z < 0.0


def test_follow_mode_moves_forward_until_body_reaches_target_size():
    controller = AttentionTrackingController(
        wake_fallback_sec=0.0,
        smoothing_alpha=1.0,
        follow_target_height=0.6,
        follow_activation_deadband=0.1,
        follow_max_linear_accel=10.0,
    )
    controller.update_control({
        "enabled": True,
        "interaction_id": "session-1",
        "mode": "follow_owner",
    }, now=1.0)
    controller.update_visual({
        "active_target": {
            "track_id": 1,
            "confidence": 0.9,
            "tracking_state": "tracking",
            "last_seen_age_ms": 10.0,
            "bbox": [0.35, 0.2, 0.3, 0.25],
            "body_center": [0.5, 0.5],
        }
    }, now=1.1)
    assert controller.command(now=1.2).linear_x > 0.0

    controller.update_visual({
        "active_target": {
            "track_id": 1,
            "confidence": 0.9,
            "tracking_state": "tracking",
            "last_seen_age_ms": 10.0,
            "bbox": [0.2, 0.05, 0.6, 0.58],
            "body_center": [0.5, 0.5],
        }
    }, now=1.3)
    assert controller.command(now=1.4).linear_x == 0.0


def test_follow_mode_rotates_before_driving_when_target_is_at_edge():
    controller = AttentionTrackingController(
        wake_fallback_sec=0.0,
        smoothing_alpha=1.0,
        follow_max_linear_accel=10.0,
    )
    controller.update_control({"enabled": True, "mode": "follow_owner"}, now=1.0)
    controller.update_visual({
        "active_target": {
            "track_id": 1,
            "confidence": 0.9,
            "tracking_state": "tracking",
            "last_seen_age_ms": 10.0,
            "bbox": [0.7, 0.2, 0.2, 0.2],
            "body_center": [0.8, 0.5],
        }
    }, now=1.1)
    command = controller.command(now=1.2)
    assert command.linear_x == 0.0
    assert command.angular_z < 0.0


def test_centered_stale_and_disabled_targets_stop_safely():
    controller = AttentionTrackingController(wake_fallback_sec=0.0)
    controller.update_control({
        "enabled": True,
        "interaction_id": "session-1",
    }, now=1.0)
    controller.update_visual({
        "active_target": {
            "confidence": 0.9,
            "tracking_state": "tracking",
            "last_seen_age_ms": 10.0,
            "face_center": [0.51, 0.3],
        }
    }, now=1.1)
    assert controller.command(now=1.2).is_zero
    assert controller.command(now=2.0).is_zero

    controller.update_control({
        "enabled": False,
        "interaction_id": "session-1",
    }, now=2.1)
    assert controller.command(now=2.2) is None


def test_formal_behavior_suspends_attention_without_publishing_override():
    controller = AttentionTrackingController()
    controller.update_control({
        "enabled": True,
        "interaction_id": "session-1",
        "wake_angle": 30.0,
    }, now=1.0)
    assert controller.command(now=1.1, suspended=True) is None


def test_center_jitter_does_not_restart_rotation_inside_hysteresis():
    controller = AttentionTrackingController(
        wake_fallback_sec=0.0,
        smoothing_alpha=1.0,
        deadband=0.05,
        activation_deadband=0.12,
    )
    controller.update_control({"enabled": True}, now=1.0)
    for index, x in enumerate((0.54, 0.46, 0.55, 0.45), start=1):
        controller.update_visual({
            "active_target": {
                "track_id": 1,
                "confidence": 0.9,
                "tracking_state": "tracking",
                "last_seen_age_ms": 10.0,
                "face_center": [x, 0.3],
            }
        }, now=1.0 + index * 0.1)
        assert controller.command(now=1.01 + index * 0.1).is_zero


def test_abrupt_error_reversal_must_ramp_through_zero():
    controller = AttentionTrackingController(
        wake_fallback_sec=0.0,
        smoothing_alpha=1.0,
        activation_deadband=0.05,
        max_angular_accel=0.5,
    )
    controller.update_control({"enabled": True}, now=1.0)
    controller.update_visual({
        "active_target": {
            "track_id": 1,
            "confidence": 0.9,
            "tracking_state": "tracking",
            "last_seen_age_ms": 10.0,
            "face_center": [0.8, 0.3],
        }
    }, now=1.1)
    right = controller.command(now=1.1)
    assert right.angular_z < 0.0

    controller.update_visual({
        "active_target": {
            "track_id": 1,
            "confidence": 0.9,
            "tracking_state": "tracking",
            "last_seen_age_ms": 10.0,
            "face_center": [0.2, 0.3],
        }
    }, now=1.2)
    reversal = controller.command(now=1.2)
    assert reversal.angular_z <= 0.0


def test_temporary_visual_miss_keeps_filtered_target_until_timeout():
    controller = AttentionTrackingController(
        wake_fallback_sec=0.0,
        visual_timeout_sec=0.8,
        smoothing_alpha=1.0,
    )
    controller.update_control({"enabled": True}, now=1.0)
    controller.update_visual({
        "active_target": {
            "track_id": 1,
            "confidence": 0.9,
            "tracking_state": "tracking",
            "last_seen_age_ms": 10.0,
            "body_center": [0.75, 0.5],
        }
    }, now=1.1)
    assert controller.command(now=1.2).angular_z < 0.0

    controller.update_visual({
        "active_target": {
            "track_id": 1,
            "confidence": 0.9,
            "tracking_state": "temporarily_lost",
            "last_seen_age_ms": 400.0,
        }
    }, now=1.3)
    assert controller.command(now=1.4).angular_z < 0.0
    assert controller.command(now=2.0).is_zero


def test_duplicate_session_control_does_not_reset_visual_tracking():
    controller = AttentionTrackingController(
        wake_fallback_sec=0.0,
        smoothing_alpha=1.0,
    )
    control = {
        "enabled": True,
        "interaction_id": "session-1",
        "mode": "follow_owner",
    }
    controller.update_control(control, now=1.0)
    controller.update_visual({
        "active_target": {
            "track_id": 1,
            "confidence": 0.9,
            "tracking_state": "tracking",
            "last_seen_age_ms": 10.0,
            "bbox": [0.25, 0.2, 0.3, 0.2],
            "body_center": [0.5, 0.5],
        }
    }, now=1.1)
    assert controller.command(now=1.2).linear_x > 0.0

    controller.update_control(control, now=1.25)
    assert controller.command(now=1.3).linear_x > 0.0
