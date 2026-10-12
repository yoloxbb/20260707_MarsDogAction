"""External navigation velocities must obey the Action posture gate."""
import threading

import pytest

from marsdog_action_executor.adapters.go2_sport_backend import Go2ChassisBackend
from marsdog_action_executor.adapters.go2_velocity_bridge import Go2VelocityBridge
from marsdog_action_executor.adapters.velocity import TwistCommand


def setup_bridge():
    now = [0.0]
    requests = []
    backend = Go2ChassisBackend(lambda req: requests.append(req) or True, {},
                               monotonic=lambda: now[0])
    return backend, Go2VelocityBridge(backend, monotonic=lambda: now[0]), requests, now


def test_external_velocity_prepares_once_and_never_replays_old_commands():
    backend, bridge, requests, now = setup_bridge()
    bridge.tick()
    assert requests == []
    bridge.receive(TwistCommand(0.1))
    for _ in range(10):
        now[0] += 0.1
        bridge.receive(TwistCommand(0.1))
    assert [req.api_id for req in requests] == [1003, 1004]
    now[0] = 2.6
    bridge.tick()  # A stale velocity must not complete preparation or move.
    assert not any(req.api_id in (1002, 1008) for req in requests)
    bridge.receive(TwistCommand(0.2))
    bridge.receive(TwistCommand(0.3))
    assert [req.api_id for req in requests][-3:] == [1002, 1008, 1008]
    assert sum(req.api_id == 1004 for req in requests) == 1
    assert sum(req.api_id == 1002 for req in requests) == 1
    bridge.receive(TwistCommand())
    assert backend._balance_prepared


def test_watchdog_stops_once_without_starting_posture_preparation():
    backend, bridge, requests, now = setup_bridge()
    bridge.receive(TwistCommand(0.2))
    now[0] = 2.6
    bridge.receive(TwistCommand(0.2))
    now[0] += 0.5
    bridge.tick()
    assert requests[-1].api_id == 1003
    size = len(requests)
    bridge.tick()
    assert len(requests) == size
    assert backend._balance_prepared


def test_cancel_during_external_preparation_prevents_later_velocity():
    backend, bridge, requests, now = setup_bridge()
    bridge.receive(TwistCommand(0.2))
    backend.cancel_step()
    now[0] = 4.0
    bridge.receive(TwistCommand(0.2))
    assert not any(req.api_id in (1002, 1008) for req in requests)
    assert not backend._standing_prepared and not backend._balance_prepared


@pytest.mark.parametrize("failed_api", [1003, 1004, 1002, 1008])
def test_publication_failure_latches_velocity_until_new_explicit_preparation(failed_api):
    now, requests, failing = [0.0], [], [True]
    def publish(req):
        requests.append(req)
        return not (failing[0] and req.api_id == failed_api)
    backend = Go2ChassisBackend(publish, {}, monotonic=lambda: now[0],
                               sleep=lambda sec: now.__setitem__(0, now[0] + sec))
    bridge = Go2VelocityBridge(backend, monotonic=lambda: now[0])
    bridge.receive(TwistCommand(.2))
    now[0] = 3.
    bridge.receive(TwistCommand(.2))
    assert not backend._standing_prepared and not backend._balance_prepared
    failing[0] = False
    requests.clear()
    for _ in range(4):
        bridge.receive(TwistCommand(.2))
    assert requests and all(req.api_id == 1003 for req in requests)
    assert backend.prepare_navigation()
    bridge.receive(TwistCommand(.2))
    assert requests[-1].api_id == 1008


@pytest.mark.parametrize("timeout", [0, -1, float("inf"), float("nan")])
def test_invalid_watchdog_timeout_is_rejected(timeout):
    with pytest.raises(ValueError):
        Go2VelocityBridge(None, timeout)


def test_spin_accepts_live_velocity_while_sequence_lock_is_owned():
    requests, now = [], [0.0]
    backend = None

    class Spin:
        last_terminal_status = 4

        def spin(self, yaw, deadline, cancel):
            # A ROS subscription runs on another executor thread.
            thread = threading.Thread(target=lambda: backend.publish_external_velocity(TwistCommand(0, 0, 0.3)))
            thread.start()
            thread.join(timeout=1)
            assert not thread.is_alive()
            assert requests[-1].api_id == 1008
            return True, False, "spin_succeeded"

    backend = Go2ChassisBackend(
        lambda req: requests.append(req) or True,
        {"spin": [{"command": "nav2_spin", "turns": 3, "duration_sec": 60}]},
        spin_transport=Spin(), monotonic=lambda: now[0],
        sleep=lambda seconds: now.__setitem__(0, now[0] + seconds),
    )
    backend.begin_behavior()
    assert backend.execute_step({"unit_id": "spin"}, None, 65)
    assert sum(req.api_id == 1008 for req in requests) == 1
    assert sum(req.api_id == 1004 for req in requests) == 1
    assert not backend._spin_velocity_allowed


def test_body_chain_blocks_external_wakeup_between_units_and_during_hold():
    backend, bridge, requests, now = setup_bridge()
    backend._sleep = lambda seconds: now.__setitem__(0, now[0] + seconds)
    backend.begin_behavior()
    bridge.receive(TwistCommand(.2))
    assert requests == []
    assert backend.prepare_navigation()
    bridge.receive(TwistCommand(.2))
    assert requests[-1].api_id == 1008  # Owned navigation is allowed.
    backend.finish_navigation()
    backend._action_sequences["down"] = [{"command": "stand_down", "duration_sec": 2.5}]
    assert backend.execute_step({"unit_id": "down"}, duration=3)
    requests.clear()
    for _ in range(3):
        bridge.receive(TwistCommand(.2))
        now[0] += 1
    assert requests == []  # The sequence lock is free between units.

    def sleep(seconds):
        now[0] += seconds
        bridge.receive(TwistCommand(.2))
    backend._sleep = sleep
    assert backend.hold_position(2)
    assert requests and all(req.api_id == 1003 for req in requests)
    assert not backend._standing_prepared
    backend.end_behavior()
    bridge.receive(TwistCommand(.2))
    assert requests[-1].api_id == 1004  # A fresh external motion can prepare.
