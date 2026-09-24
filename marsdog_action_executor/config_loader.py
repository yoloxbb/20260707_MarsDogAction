"""ConfigLoader — loads and validates YAML configuration files.

Startup validation catches:
  - missing behavior-tree action templates
  - missing referenced actions
  - invalid selection_policy / unit_type / interrupt_policy
  - duplicate stage orders
  - empty required stages
  - weight < 0
  - timeout < 0 or excessively large
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any, Mapping

logger = logging.getLogger(__name__)

# ── Valid enum values ─────────────────────────────────────────────────────────

VALID_SELECTION_POLICIES = {
    "fixed", "sequence", "random_one", "weighted_random",
    "random_n", "loop_random", "condition_first",
}

VALID_UNIT_TYPES = {
    "atomic_action", "composite_action", "task", "policy", "modifier",
}

VALID_INTERRUPT_POLICIES = {
    "immediate", "safe_point", "non_interruptible",
}

VALID_MOTION_STATES = {
    "active", "stationary",
}


class ConfigurationError(Exception):
    """Raised when configuration is invalid — should prevent node startup."""

    def __init__(self, path: str, detail: str) -> None:
        super().__init__(f"Config error in {path}: {detail}")
        self.path = path
        self.detail = detail


class ConfigLoader:
    """Loads and validates YAML configuration from a directory.

    Usage::

        loader = ConfigLoader("/path/to/config")
        loader.load_all()
        templates = loader.behavior_tree_templates
        catalog = loader.action_catalog
    """

    def __init__(self, config_dir: str | Path = "config") -> None:
        self._dir = Path(config_dir)
        self.behavior_tree_templates: dict[str, Any] = {}
        self.action_catalog: dict[str, Any] = {}
        self.controller_routes: dict[str, Any] = {}
        self.safety_policies: dict[str, Any] = {}
        self.navigation_config: dict[str, Any] = {}
        self.sound_config: dict[str, Any] = {}
        self.wake_orientation_config: dict[str, Any] = {}
        self.visual_target_approach_config: dict[str, Any] = {}
        self.uwb_follow_config: dict[str, Any] = {}
        self.go2_sport_config: dict[str, Any] = {}
        self.lite3_action_config: dict[str, Any] = {}
        self._errors: list[str] = []

    # Mapping of filename → (attr, wrapper_key)
    # wrapper_key=None means the YAML top level IS the data (no unwrap needed).
    _CONFIG_FILES: list[tuple[str, str, str | None]] = [
        ("behavior_tree_actions.yaml", "behavior_tree_templates", "behaviors"),
        ("action_catalog.yaml", "action_catalog", "action_units"),
        ("controller_routes.yaml", "controller_routes", None),
        ("safety_policies.yaml", "safety_policies", None),
        ("navigation_waypoints.yaml", "navigation_config", None),
        ("sound_config.yaml", "sound_config", "bark_sound"),
        ("wake_orientation.yaml", "wake_orientation_config", None),
        (
            "visual_target_approach.yaml",
            "visual_target_approach_config",
            None,
        ),
        ("uwb_follow.yaml", "uwb_follow_config", None),
        ("go2_sport.yaml", "go2_sport_config", None),
        ("lite3_actions.yaml", "lite3_action_config", None),
    ]

    _REQUIRED_CONFIGS = {
        "behavior_tree_actions.yaml",
        "action_catalog.yaml",
        "controller_routes.yaml",
        "navigation_waypoints.yaml",
        "wake_orientation.yaml",
        "visual_target_approach.yaml",
        "uwb_follow.yaml",
        "go2_sport.yaml",
        "lite3_actions.yaml",
    }

    def load_all(self) -> None:
        """Load all config files and validate."""
        for filename, attr, wrapper_key in self._CONFIG_FILES:
            required = filename in self._REQUIRED_CONFIGS
            self._load_yaml(filename, attr, required=required, unwrap_key=wrapper_key)

        referenced_actions = {
            candidate.get("unit_id")
            for behavior in self.behavior_tree_templates.values()
            for stage in behavior.get("stages", [])
            for candidate in stage.get("candidates", [])
            if isinstance(candidate, dict) and candidate.get("unit_id")
        }
        catalog_actions = set(self.action_catalog)
        unexpected_actions = catalog_actions - referenced_actions
        if unexpected_actions:
            self._errors.append(
                "action_catalog.yaml contains actions outside "
                "behavior_tree_actions.yaml: "
                + ", ".join(sorted(unexpected_actions))
            )

        self._validate()

        if self._errors:
            msg = "\n  - ".join(self._errors)
            raise ConfigurationError(str(self._dir), f"Validation errors:\n  - {msg}")

    def _load_yaml(
        self, filename: str, attr: str, required: bool = True,
        unwrap_key: str | None = None,
    ) -> None:
        filepath = self._dir / filename
        if not filepath.exists():
            if required:
                self._errors.append(f"Missing required config: {filepath}")
            else:
                logger.debug("Optional config not found: %s", filepath)
            return

        try:
            import yaml
        except ImportError:
            logger.error("PyYAML not installed — cannot load configs")
            return

        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            if data is None:
                if required:
                    self._errors.append(f"Empty config: {filepath}")
                return
            # Unwrap the contract's top-level wrapper key.
            if unwrap_key and isinstance(data, dict) and unwrap_key in data:
                data = data[unwrap_key]
            if not isinstance(data, dict):
                if required:
                    self._errors.append(f"Invalid config structure in {filepath}: expected dict, got {type(data).__name__}")
                return
            setattr(self, attr, data)
            logger.info("Loaded %s (%d entries)", filename, len(data))
        except Exception as exc:
            self._errors.append(f"Failed to load {filepath}: {exc}")

    def _validate(self) -> None:
        """Run all validation checks."""
        self._validate_behavior_templates()
        self._validate_action_catalog()
        self._validate_controller_routes()
        self._validate_navigation_config()
        self._validate_sound_config()
        self._validate_wake_orientation_config()
        self._validate_visual_target_approach_config()
        self._validate_uwb_follow_config()
        self._validate_go2_sport_config()
        self._validate_lite3_action_config()

    def _validate_lite3_action_config(self) -> None:
        """Validate the fail-closed Lite3 simple-command action mapping."""
        config = self.lite3_action_config
        if not config:
            return
        for key in ("enabled", "allow_proxies", "allow_unverified"):
            if not isinstance(config.get(key), bool):
                self._errors.append(
                    f"lite3_actions.yaml: {key} must be boolean"
                )
        for key in ("simple_cmd_topic", "status_topic", "cmd_vel_topic"):
            value = config.get(key)
            if not isinstance(value, str) or not value.startswith("/"):
                self._errors.append(
                    f"lite3_actions.yaml: {key} must be an absolute ROS topic"
                )
        rate = config.get("publish_rate_hz")
        if not self._is_finite_number(rate) or not 20.0 <= float(rate) <= 100.0:
            self._errors.append(
                "lite3_actions.yaml: publish_rate_hz must be within [20, 100]"
            )
        for key in (
            "status_timeout_sec",
            "min_battery_percent",
            "minimum_backward_clearance_m",
        ):
            value = config.get(key)
            if not self._is_finite_number(value) or float(value) <= 0.0:
                self._errors.append(
                    f"lite3_actions.yaml: {key} must be finite and > 0"
                )
        mode_settle = config.get("mode_settle_sec")
        if (
            not self._is_finite_number(mode_settle)
            or not 0.0 <= float(mode_settle) <= 2.0
        ):
            self._errors.append(
                "lite3_actions.yaml: mode_settle_sec must be within [0, 2]"
            )
        navigation_settle_timeout = config.get(
            "navigation_settle_timeout_sec"
        )
        if (
            not self._is_finite_number(navigation_settle_timeout)
            or not 0.0 < float(navigation_settle_timeout) <= 10.0
        ):
            self._errors.append(
                "lite3_actions.yaml: navigation_settle_timeout_sec must be "
                "within (0, 10]"
            )
        navigation_stable_samples = config.get("navigation_stable_samples")
        if (
            not isinstance(navigation_stable_samples, int)
            or isinstance(navigation_stable_samples, bool)
            or not 1 <= navigation_stable_samples <= 100
        ):
            self._errors.append(
                "lite3_actions.yaml: navigation_stable_samples must be "
                "within [1, 100]"
            )
        posture_recovery_timeout = config.get("posture_recovery_timeout_sec")
        if (
            not self._is_finite_number(posture_recovery_timeout)
            or not 0.0 < float(posture_recovery_timeout) <= 30.0
        ):
            self._errors.append(
                "lite3_actions.yaml: posture_recovery_timeout_sec must be "
                "within (0, 30]"
            )
        stop_count = config.get("stop_publish_count")
        if (
            not isinstance(stop_count, int)
            or isinstance(stop_count, bool)
            or not 1 <= stop_count <= 20
        ):
            self._errors.append(
                "lite3_actions.yaml: stop_publish_count must be within [1, 20]"
            )
        repeat = config.get("command_repeat")
        if (
            not isinstance(repeat, int)
            or isinstance(repeat, bool)
            or not 1 <= repeat <= 5
        ):
            self._errors.append(
                "lite3_actions.yaml: command_repeat must be within [1, 5]"
            )
        repeat_interval = config.get("command_repeat_interval_sec")
        if (
            not self._is_finite_number(repeat_interval)
            or not 0.0 <= float(repeat_interval) <= 1.0
        ):
            self._errors.append(
                "lite3_actions.yaml: command_repeat_interval_sec must be "
                "within [0, 1]"
            )

        limits = config.get("limits", {})
        for key in ("max_linear_x", "max_linear_y", "max_angular_z"):
            value = limits.get(key)
            if not self._is_finite_number(value) or float(value) < 0.0:
                self._errors.append(
                    f"lite3_actions.yaml: limits.{key} must be >= 0"
                )

        direct = config.get("action_sequences", {})
        groups = config.get("action_groups", {})
        if not isinstance(direct, dict) or not direct:
            self._errors.append(
                "lite3_actions.yaml: action_sequences must be a non-empty mapping"
            )
            direct = {}
        if not isinstance(groups, dict) or not groups:
            self._errors.append(
                "lite3_actions.yaml: action_groups must be a non-empty mapping"
            )
            groups = {}

        mapped: set[str] = set()
        for unit_id, plan in direct.items():
            self._validate_lite3_plan(
                plan,
                label=f"Lite3 action {unit_id!r}",
                unit_ids=[str(unit_id)],
                limits=limits,
            )
            if unit_id not in self.action_catalog:
                self._errors.append(
                    f"lite3_actions.yaml: unknown action {unit_id!r}"
                )
            mapped.add(str(unit_id))

        for group_name, plan in groups.items():
            label = f"Lite3 action group {group_name!r}"
            if not isinstance(plan, dict):
                self._errors.append(f"{label}: expected mapping")
                continue
            actions = plan.get("actions")
            if not isinstance(actions, list) or not actions:
                self._errors.append(f"{label}: actions must be non-empty")
                actions = []
            valid_actions: list[str] = []
            for unit_id in actions:
                if not isinstance(unit_id, str) or not unit_id:
                    self._errors.append(
                        f"{label}: action IDs must be non-empty strings"
                    )
                    continue
                if unit_id not in self.action_catalog:
                    self._errors.append(f"{label}: unknown action {unit_id!r}")
                if unit_id in mapped:
                    self._errors.append(
                        f"lite3_actions.yaml: action {unit_id!r} appears in "
                        "multiple plans"
                    )
                mapped.add(unit_id)
                valid_actions.append(unit_id)
            self._validate_lite3_plan(
                plan,
                label=label,
                unit_ids=valid_actions,
                limits=limits,
            )

        overrides = config.get("controller_route_overrides", {})
        if not isinstance(overrides, dict):
            self._errors.append(
                "lite3_actions.yaml: controller_route_overrides must be a mapping"
            )
            overrides = {}
        expected_owner_actions = {
            "ACT_INTERACT_APPROACH_OWNER",
            "ACT_INTERACT_APPROACH_OWNER_CLOSER",
            "ACT_INTERACT_RETURN_OWNER",
        }
        if set(overrides) != expected_owner_actions or any(
            route != "person_nav_approach" for route in overrides.values()
        ):
            self._errors.append(
                "lite3_actions.yaml: owner route overrides must map all three "
                "owner approach actions to person_nav_approach"
            )
        if mapped & set(overrides):
            self._errors.append(
                "lite3_actions.yaml: mapped actions cannot also have route overrides"
            )

        accepted = config.get("accepted_unverified_actions", [])
        if not isinstance(accepted, list) or any(
            not isinstance(unit_id, str) or not unit_id
            for unit_id in accepted
        ):
            self._errors.append(
                "lite3_actions.yaml: accepted_unverified_actions must be a "
                "list of action or shared-plan IDs"
            )
        else:
            unknown_accepted = set(accepted) - mapped
            if unknown_accepted:
                self._errors.append(
                    "lite3_actions.yaml: accepted_unverified_actions contains "
                    f"unmapped actions {sorted(unknown_accepted)!r}"
                )
            for label, plan in [
                *[
                    (f"Lite3 action {unit_id!r}", item)
                    for unit_id, item in direct.items()
                    if isinstance(item, dict)
                ],
                *[
                    (f"Lite3 action group {group_name!r}", item)
                    for group_name, item in groups.items()
                    if isinstance(item, dict)
                ],
            ]:
                if "plan_id" not in plan:
                    continue
                plan_id = str(plan.get("plan_id", ""))
                reference = direct.get(plan_id)
                if not isinstance(reference, dict):
                    self._errors.append(
                        f"{label}: shared plan_id {plan_id!r} must reference "
                        "a direct Lite3 action"
                    )
                elif plan.get("sequence") != reference.get("sequence"):
                    self._errors.append(
                        f"{label}: shared plan_id {plan_id!r} must reuse the "
                        "exact same physical sequence"
                    )

            unaccepted_unverified: list[str] = []
            for unit_id, plan in self.get_lite3_action_plans().items():
                if bool(plan.get("verified", False)):
                    continue
                plan_id = str(plan.get("plan_id", unit_id))
                if unit_id not in accepted and plan_id not in accepted:
                    unaccepted_unverified.append(unit_id)
            if unaccepted_unverified:
                self._errors.append(
                    "lite3_actions.yaml: unverified plans have no explicit "
                    "action or shared-plan acceptance "
                    f"{sorted(unaccepted_unverified)!r}"
                )

        policies = self.visual_target_approach_config.get(
            "lite3_owner_approach_policies", {}
        )
        expected_policy_names = {
            "come_to_owner", "approach_owner", "return_to_owner"
        }
        if not isinstance(policies, dict) or set(policies) != expected_policy_names:
            self._errors.append(
                "visual_target_approach.yaml: lite3_owner_approach_policies "
                "must be exactly come_to_owner, approach_owner, return_to_owner"
            )
        else:
            for name, policy in policies.items():
                if (
                    not isinstance(policy, dict)
                    or policy.get("target_type") != "human"
                    or str(policy.get("required_identity", "")).lower() != "owner"
                    or policy.get("distance_mode") != "bbox_height"
                ):
                    self._errors.append(
                        f"Lite3 owner policy {name!r} must require a human "
                        "owner with bbox_height distance mode"
                    )

    def _validate_lite3_plan(
        self,
        plan: Any,
        *,
        label: str,
        unit_ids: list[str],
        limits: dict[str, Any],
    ) -> None:
        if not isinstance(plan, dict):
            self._errors.append(f"{label}: expected mapping")
            return
        if plan.get("fidelity") not in {"exact", "proxy"}:
            self._errors.append(f"{label}: fidelity must be exact or proxy")
        if not isinstance(plan.get("verified"), bool):
            self._errors.append(f"{label}: verified must be boolean")
        if not isinstance(plan.get("description"), str) or not plan.get(
            "description", ""
        ).strip():
            self._errors.append(f"{label}: description must be non-empty")
        sequence = plan.get("sequence")
        if not isinstance(sequence, list) or not sequence:
            self._errors.append(f"{label}: sequence must be non-empty")
            return

        pose_dead_zones = {
            553713968: 6553,
            553713973: 9553,
            553713922: 20000,
        }
        pose_axis_max_abs = 32767
        forbidden_codes = {
            553713969,  # roll axis; forbidden with top-mounted payload
            553716746,  # APP voice channel
            553782286,  # motor-disabling soft E-stop
            553714178,  # stand/lie toggle
            553714189,  # twist jump
            553714181,  # roll over
            553714946,  # backflip
            553714955,  # jump forward
        }
        for index, step in enumerate(sequence):
            step_label = f"{label} step {index}"
            if not isinstance(step, dict):
                self._errors.append(f"{step_label}: expected mapping")
                continue
            kind = step.get("type")
            if kind not in {
                "simple",
                "target_posture",
                "pose",
                "twist",
                "hold",
            }:
                self._errors.append(
                    f"{step_label}: unsupported type {kind!r}"
                )
                continue
            initial_states = step.get("when_initial_basic_states")
            if initial_states is not None and (
                not isinstance(initial_states, list)
                or not initial_states
                or any(
                    not isinstance(state, int)
                    or isinstance(state, bool)
                    or state not in {1, 6}
                    for state in initial_states
                )
                or len(set(initial_states)) != len(initial_states)
            ):
                self._errors.append(
                    f"{step_label}: when_initial_basic_states must be a "
                    "non-empty unique list containing only 1 or 6"
                )
            if kind in {"simple", "pose"}:
                code = step.get("cmd_code")
                if not isinstance(code, int) or isinstance(code, bool):
                    self._errors.append(
                        f"{step_label}: cmd_code must be an integer"
                    )
                elif code in forbidden_codes:
                    self._errors.append(
                        f"{step_label}: forbidden command code {code}"
                    )
                if step.get("require", "none") not in {"none", "stand", "lie"}:
                    self._errors.append(
                        f"{step_label}: require must be none, stand, or lie"
                    )
            if kind == "hold":
                if step.get("require", "none") not in {"none", "stand", "lie"}:
                    self._errors.append(
                        f"{step_label}: require must be none, stand, or lie"
                    )
                if not isinstance(step.get("require_status", True), bool):
                    self._errors.append(
                        f"{step_label}: require_status must be boolean"
                    )
            if kind == "target_posture":
                if step.get("target") not in {"stand", "lie"}:
                    self._errors.append(
                        f"{step_label}: target must be stand or lie"
                    )
                stable_samples = step.get("stable_samples", 5)
                if (
                    not isinstance(stable_samples, int)
                    or isinstance(stable_samples, bool)
                    or not 1 <= stable_samples <= 100
                ):
                    self._errors.append(
                        f"{step_label}: stable_samples must be within [1, 100]"
                    )
            if kind == "pose":
                code = step.get("cmd_code")
                value = step.get("value")
                if code not in pose_dead_zones:
                    self._errors.append(
                        f"{step_label}: command is not a pose-axis code"
                    )
                if (
                    not isinstance(value, int)
                    or isinstance(value, bool)
                    or abs(value) > pose_axis_max_abs
                ):
                    self._errors.append(
                        f"{step_label}: pose value exceeds the documented range"
                    )
                elif (
                    code in pose_dead_zones
                    and (value == 0 or abs(value) <= pose_dead_zones[code])
                ):
                    self._errors.append(
                        f"{step_label}: pose value is inside the documented "
                        "dead zone"
                    )
                if plan.get("verified") is not False:
                    self._errors.append(
                        f"{label}: pose-axis plans must remain unverified "
                        "until hardware acceptance"
                    )
            if kind == "twist":
                for axis, limit_name in (
                    ("linear_x", "max_linear_x"),
                    ("linear_y", "max_linear_y"),
                    ("angular_z", "max_angular_z"),
                ):
                    value = step.get(axis, 0.0)
                    if not self._is_finite_number(value):
                        self._errors.append(
                            f"{step_label}: {axis} must be finite"
                        )
                    elif abs(float(value)) > float(limits.get(limit_name, 0.0)):
                        self._errors.append(
                            f"{step_label}: {axis} exceeds {limit_name}"
                        )
                clearance = step.get("minimum_backward_clearance_m", 0.0)
                if (
                    not self._is_finite_number(clearance)
                    or float(clearance) < 0.0
                ):
                    self._errors.append(
                        f"{step_label}: minimum_backward_clearance_m must be "
                        "finite and >= 0"
                    )
            if kind in {"pose", "twist", "hold"}:
                duration = step.get("duration_sec")
                if not self._is_finite_number(duration) or float(duration) <= 0.0:
                    self._errors.append(
                        f"{step_label}: duration_sec must be finite and > 0"
                    )
            if kind in {"simple", "target_posture"}:
                timeout = step.get("completion_timeout_sec", 0.0)
                minimum_timeout = 0.0 if kind == "simple" else 0.001
                if (
                    not self._is_finite_number(timeout)
                    or float(timeout) < minimum_timeout
                ):
                    self._errors.append(
                        f"{step_label}: completion_timeout_sec must be "
                        f">= {minimum_timeout:g}"
                    )

        for unit_id in unit_ids:
            if unit_id not in self.action_catalog:
                continue
            timeout = self.action_catalog[unit_id].get("timeout_sec", 0.0)
            if not self._is_finite_number(timeout) or float(timeout) <= 0.0:
                continue
            unconditional = 0.0
            branch_costs: dict[int, float] = {}
            for step in sequence:
                if not isinstance(step, dict):
                    continue
                cost = float(
                    step.get(
                        "completion_timeout_sec",
                        step.get("duration_sec", 0.0),
                    )
                )
                initial_states = step.get("when_initial_basic_states")
                if isinstance(initial_states, list):
                    for state in initial_states:
                        if isinstance(state, int) and not isinstance(state, bool):
                            branch_costs[state] = branch_costs.get(state, 0.0) + cost
                else:
                    unconditional += cost
            worst_case = unconditional + max(branch_costs.values(), default=0.0)
            if worst_case > float(timeout):
                self._errors.append(
                    f"{label} for {unit_id!r}: sequence duration "
                    f"{worst_case:.3f}s exceeds action timeout "
                    f"{float(timeout):.3f}s"
                )

    def _validate_go2_sport_config(self) -> None:
        """Validate the isolated Unitree Go2 SportMode action mapping."""
        config = self.go2_sport_config
        if not config:
            return
        if not isinstance(config.get("enabled", False), bool):
            self._errors.append("go2_sport.yaml: enabled must be boolean")
        topic = config.get("request_topic")
        if not isinstance(topic, str) or not topic.startswith("/"):
            self._errors.append(
                "go2_sport.yaml: request_topic must be an absolute ROS topic"
            )
        rate = config.get("publish_rate_hz")
        if not self._is_finite_number(rate) or not 1.0 <= float(rate) <= 100.0:
            self._errors.append(
                "go2_sport.yaml: publish_rate_hz must be within [1, 100]"
            )
        stop_count = config.get("stop_publish_count")
        if (
            not isinstance(stop_count, int)
            or isinstance(stop_count, bool)
            or not 1 <= stop_count <= 20
        ):
            self._errors.append(
                "go2_sport.yaml: stop_publish_count must be within [1, 20]"
            )

        limits = config.get("limits", {})
        for key in ("max_linear_x", "max_linear_y", "max_angular_z"):
            value = limits.get(key)
            if not self._is_finite_number(value) or float(value) < 0.0:
                self._errors.append(
                    f"go2_sport.yaml: limits.{key} must be >= 0"
                )
        min_linear = limits.get("min_linear_x", 0.0)
        if not self._is_finite_number(min_linear) or float(min_linear) < 0.0:
            self._errors.append(
                "go2_sport.yaml: limits.min_linear_x must be >= 0"
            )

        supported_commands = {
            "damp", "balance_stand", "stop_move", "stand_up",
            "stand_down", "recovery_stand", "euler", "move", "sit",
            "rise_sit", "speed_level", "hello", "stretch", "content",
            "dance1", "dance2", "heart",
        }
        sequences = config.get("action_sequences", {})
        if not isinstance(sequences, dict) or not sequences:
            self._errors.append(
                "go2_sport.yaml: action_sequences must be a non-empty mapping"
            )
            sequences = {}
        for unit_id, sequence in sequences.items():
            if unit_id not in self.action_catalog:
                self._errors.append(
                    f"go2_sport.yaml: unknown action {unit_id!r}"
                )
            self._validate_go2_sequence(
                sequence,
                label=f"Go2 action {unit_id!r}",
                supported_commands=supported_commands,
                unit_ids=[str(unit_id)],
            )

        action_groups = config.get("action_groups", {})
        if not isinstance(action_groups, dict) or not action_groups:
            self._errors.append(
                "go2_sport.yaml: action_groups must be a non-empty mapping"
            )
            action_groups = {}
        grouped_actions: set[str] = set()
        for group_name, group_config in action_groups.items():
            label = f"Go2 action group {group_name!r}"
            if not isinstance(group_config, dict):
                self._errors.append(f"{label}: expected mapping")
                continue
            fidelity = group_config.get("fidelity")
            if fidelity not in {"exact", "proxy"}:
                self._errors.append(
                    f"{label}: fidelity must be 'exact' or 'proxy'"
                )
            description = group_config.get("description")
            if not isinstance(description, str) or not description.strip():
                self._errors.append(
                    f"{label}: description must be a non-empty string"
                )
            actions = group_config.get("actions")
            if not isinstance(actions, list) or not actions:
                self._errors.append(f"{label}: actions must be a non-empty list")
                actions = []
            valid_actions: list[str] = []
            for unit_id in actions:
                if not isinstance(unit_id, str) or not unit_id:
                    self._errors.append(
                        f"{label}: action IDs must be non-empty strings"
                    )
                    continue
                if unit_id not in self.action_catalog:
                    self._errors.append(f"{label}: unknown action {unit_id!r}")
                if unit_id in grouped_actions:
                    self._errors.append(
                        f"go2_sport.yaml: action {unit_id!r} appears in "
                        "multiple action_groups"
                    )
                if unit_id in sequences:
                    self._errors.append(
                        f"go2_sport.yaml: action {unit_id!r} cannot have both "
                        "an action_sequence and an action_group"
                    )
                grouped_actions.add(unit_id)
                valid_actions.append(unit_id)
            self._validate_go2_sequence(
                group_config.get("sequence"),
                label=label,
                supported_commands=supported_commands,
                unit_ids=valid_actions,
            )

        overrides = config.get("controller_route_overrides", {})
        if not isinstance(overrides, dict):
            self._errors.append(
                "go2_sport.yaml: controller_route_overrides must be a mapping"
            )
            overrides = {}
        for unit_id, route in overrides.items():
            if unit_id not in self.action_catalog:
                self._errors.append(
                    f"go2_sport.yaml route override: unknown action {unit_id!r}"
                )
            if route not in {"person_nav_approach"}:
                self._errors.append(
                    f"go2_sport.yaml route override {unit_id!r}: unsupported "
                    f"controller {route!r}"
                )
            if unit_id in sequences or unit_id in grouped_actions:
                self._errors.append(
                    f"go2_sport.yaml: {unit_id!r} cannot have both a SportMode "
                    "mapping and a controller route override"
                )

        owner_policies = self.visual_target_approach_config.get(
            "go2_owner_approach_policies", {}
        )
        if not isinstance(owner_policies, dict) or not owner_policies:
            self._errors.append(
                "visual_target_approach.yaml: "
                "go2_owner_approach_policies must be a non-empty mapping"
            )
            owner_policies = {}
        expected_owner_actions = {
            "come_to_owner": "ACT_INTERACT_APPROACH_OWNER",
            "approach_owner": "ACT_INTERACT_APPROACH_OWNER_CLOSER",
            "return_to_owner": "ACT_INTERACT_RETURN_OWNER",
        }
        if set(owner_policies) != set(expected_owner_actions):
            self._errors.append(
                "visual_target_approach.yaml: Go2 owner approach policies "
                "must be exactly come_to_owner, approach_owner, return_to_owner"
            )
        if set(overrides) != set(expected_owner_actions.values()):
            self._errors.append(
                "go2_sport.yaml: controller_route_overrides must match all "
                "three owner-approach actions"
            )
        for behavior_name, policy in owner_policies.items():
            if behavior_name not in self.behavior_tree_templates:
                self._errors.append(
                    "visual_target_approach.yaml: unknown Go2 owner policy "
                    f"{behavior_name!r}"
                )
                continue
            if not isinstance(policy, dict):
                self._errors.append(
                    f"Go2 owner policy {behavior_name!r} must be a mapping"
                )
                continue
            if (
                policy.get("target_type") != "human"
                or str(policy.get("required_identity", "")).strip().lower()
                != "owner"
                or policy.get("distance_mode") != "bbox_height"
            ):
                self._errors.append(
                    f"Go2 owner policy {behavior_name!r} must require a human "
                    "owner with bbox_height distance mode"
                )
            for key in (
                "desired_distance_m", "minimum_safe_distance_m",
                "target_max_age_ms", "target_lost_timeout_sec",
                "max_linear_x", "max_linear_accel", "bbox_target_height",
                "bbox_height_deadband", "arrival_hold_sec",
            ):
                value = policy.get(key)
                if not self._is_finite_number(value) or float(value) <= 0.0:
                    self._errors.append(
                        f"Go2 owner policy {behavior_name!r}.{key} must be > 0"
                    )

        go2_actions = set(sequences) | grouped_actions
        effective_routes = dict(self.controller_routes.get("routes", {}))
        effective_routes.update({unit_id: "go2" for unit_id in go2_actions})
        effective_routes.update(
            {str(unit_id): str(route) for unit_id, route in overrides.items()}
        )
        invalid_go2_routes = {
            "unsupported", "perception_navigation_manipulation"
        }
        for unit_id in sorted(self.action_catalog):
            route = effective_routes.get(
                unit_id,
                effective_routes.get("_default", "unsupported"),
            )
            if route in invalid_go2_routes:
                self._errors.append(
                    f"go2_sport.yaml: action {unit_id!r} remains on "
                    f"unavailable Go2 route {route!r}"
                )

    def _validate_go2_sequence(
        self,
        sequence: Any,
        *,
        label: str,
        supported_commands: set[str],
        unit_ids: list[str],
    ) -> None:
        """Validate one reusable Go2 SportMode sequence and its time budget."""
        if not isinstance(sequence, list) or not sequence:
            self._errors.append(f"{label}: sequence must be non-empty")
            return
        total_duration = 0.0
        for index, item in enumerate(sequence):
            if not isinstance(item, dict):
                self._errors.append(
                    f"{label} item {index}: expected mapping"
                )
                continue
            command = item.get("command")
            if command not in supported_commands:
                self._errors.append(
                    f"{label} item {index}: unsupported command {command!r}"
                )
            duration = item.get("duration_sec")
            if not self._is_finite_number(duration) or float(duration) < 0.0:
                self._errors.append(
                    f"{label} item {index}: duration_sec must be >= 0"
                )
            else:
                total_duration += float(duration)
            if command in {"move", "euler"}:
                for axis in ("x", "y", "z"):
                    if not self._is_finite_number(item.get(axis, 0.0)):
                        self._errors.append(
                            f"{label} item {index}: {axis} must be finite"
                        )
        for unit_id in unit_ids:
            timeout = self.action_catalog.get(unit_id, {}).get("timeout_sec")
            if (
                self._is_finite_number(timeout)
                and float(timeout) > 0.0
                and total_duration > float(timeout)
            ):
                self._errors.append(
                    f"{label} for {unit_id!r}: sequence duration "
                    f"{total_duration:.3f}s exceeds action timeout "
                    f"{float(timeout):.3f}s"
                )

    def _validate_behavior_templates(self) -> None:
        templates = self.get_all_behavior_templates()
        if not templates:
            return
        for name, config in templates.items():
            stages = config.get("stages", [])
            if not stages:
                self._errors.append(f"Behavior {name!r}: no stages defined")
                continue
            seen_orders: set[int] = set()
            for stage in stages:
                sid = stage.get("stage_id", "?")
                order = stage.get("order")
                if order is not None:
                    if order in seen_orders:
                        self._errors.append(f"Behavior {name!r}: duplicate stage order {order}")
                    seen_orders.add(order)
                policy = stage.get("selection_policy", "random_one")
                if policy not in VALID_SELECTION_POLICIES:
                    self._errors.append(f"Behavior {name!r} stage {sid!r}: invalid selection_policy {policy!r}")
                motion_state = stage.get("motion_state", "active")
                if motion_state not in VALID_MOTION_STATES:
                    self._errors.append(
                        f"Behavior {name!r} stage {sid!r}: "
                        f"invalid motion_state {motion_state!r}"
                    )
                if stage.get("required", True) and not stage.get("candidates"):
                    self._errors.append(f"Behavior {name!r} stage {sid!r}: required stage has no candidates")
                for candidate in stage.get("candidates", []):
                    if not isinstance(candidate, dict):
                        continue
                    unit_id = candidate.get("unit_id")
                    if unit_id and unit_id not in self.action_catalog:
                        self._errors.append(
                            f"Behavior {name!r} stage {sid!r}: "
                            f"unknown action {unit_id!r}"
                        )

    def _validate_action_catalog(self) -> None:
        catalog = self.action_catalog
        if not catalog:
            return
        for unit_id, config in catalog.items():
            ut = config.get("unit_type", "")
            if ut and ut not in VALID_UNIT_TYPES:
                self._errors.append(f"Action {unit_id!r}: invalid unit_type {ut!r}")
            ip_ = config.get("interrupt_policy", "")
            if ip_ and ip_ not in VALID_INTERRUPT_POLICIES:
                self._errors.append(f"Action {unit_id!r}: invalid interrupt_policy {ip_!r}")
            to_ = config.get("timeout_sec", 0)
            if isinstance(to_, (int, float)) and to_ < 0:
                self._errors.append(f"Action {unit_id!r}: negative timeout_sec={to_}")
            elif isinstance(to_, (int, float)) and to_ > 3600:
                self._errors.append(f"Action {unit_id!r}: unreasonable timeout_sec={to_}")
            weight = config.get("weight")
            if weight is not None and (not isinstance(weight, (int, float)) or weight < 0):
                self._errors.append(f"Action {unit_id!r}: invalid weight={weight}")

    def _validate_controller_routes(self) -> None:
        routes = self.controller_routes.get("routes", {})
        if not isinstance(routes, dict):
            self._errors.append("controller_routes.yaml: routes must be a mapping")
            return
        if routes.get("_default") != "unsupported":
            self._errors.append(
                "controller_routes.yaml: _default must be 'unsupported' "
                "so unconfigured physical actions cannot mock-success"
            )
        for unit_id in routes:
            if unit_id != "_default" and unit_id not in self.action_catalog:
                self._errors.append(
                    f"controller_routes.yaml: unknown action {unit_id!r}"
                )
            if routes[unit_id] == "agv":
                self._errors.append(
                    f"controller_routes.yaml: AGV route removed for {unit_id!r}"
                )
        for unit_id, config in self.action_catalog.items():
            if not bool(config.get("requires_controller", False)):
                continue
            platform_routes = (
                self.get_controller_routes("go2"),
                self.get_controller_routes("lite3"),
            )
            if all(
                effective.get(unit_id, effective.get("_default"))
                in (None, "", "mock", "unsupported")
                for effective in platform_routes
            ):
                self._errors.append(
                    f"Action {unit_id!r}: requires_controller=true needs "
                    "a Go2 or Lite3 controller route"
                )

    def _validate_navigation_config(self) -> None:
        config = self.navigation_config
        if not config:
            return

        waypoints = config.get("waypoints", {})
        behavior_routes = config.get("behavior_routes", {})
        stage_actions = config.get("stage_actions", [])

        if not isinstance(config.get("enabled", False), bool):
            self._errors.append(
                "navigation_waypoints.yaml: enabled must be boolean"
            )
        action_name = config.get("action_name")
        if not isinstance(action_name, str) or not action_name.startswith("/"):
            self._errors.append(
                "navigation_waypoints.yaml: action_name must be an absolute "
                "ROS action name"
            )
        frame_id = config.get("frame_id")
        if not isinstance(frame_id, str) or not frame_id.strip():
            self._errors.append(
                "navigation_waypoints.yaml: frame_id must be non-empty"
            )
        for key in ("server_timeout_sec", "result_timeout_sec"):
            value = config.get(key)
            if not self._is_finite_number(value) or float(value) <= 0.0:
                self._errors.append(
                    f"navigation_waypoints.yaml: {key} must be > 0"
                )

        waypoint_nav = config.get("waypoint_nav")
        if not isinstance(waypoint_nav, dict):
            self._errors.append(
                "navigation_waypoints.yaml: waypoint_nav must be a mapping"
            )
            waypoint_nav = {}
        for key in ("service_name", "status_topic"):
            value = waypoint_nav.get(key)
            if not isinstance(value, str) or not value.startswith("/"):
                self._errors.append(
                    "navigation_waypoints.yaml: waypoint_nav."
                    f"{key} must be an absolute ROS name"
                )
        for key in ("protocol_version", "client_id"):
            value = waypoint_nav.get(key)
            if not isinstance(value, str) or not value.strip():
                self._errors.append(
                    "navigation_waypoints.yaml: waypoint_nav."
                    f"{key} must be non-empty"
                )
        for key in (
            "service_timeout_sec",
            "query_timeout_sec",
            "cancel_confirmation_timeout_sec",
            "terminal_retention_sec",
            "preempt_lock_wait_sec",
        ):
            value = waypoint_nav.get(key)
            if not self._is_finite_number(value) or float(value) <= 0.0:
                self._errors.append(
                    "navigation_waypoints.yaml: waypoint_nav."
                    f"{key} must be > 0"
                )

        if not isinstance(waypoints, dict) or not waypoints:
            self._errors.append(
                "navigation_waypoints.yaml: waypoints must be a non-empty "
                "mapping"
            )
            return
        if not isinstance(behavior_routes, dict) or not behavior_routes:
            self._errors.append(
                "navigation_waypoints.yaml: behavior_routes must be a "
                "non-empty mapping"
            )
            return
        if not isinstance(stage_actions, list) or not stage_actions:
            self._errors.append(
                "navigation_waypoints.yaml: stage_actions must be a "
                "non-empty list"
            )
            return

        places = waypoint_nav.get("places")
        if not isinstance(places, dict):
            self._errors.append(
                "navigation_waypoints.yaml: waypoint_nav.places must be "
                "a mapping"
            )
            places = {}
        for waypoint_name in waypoints:
            place = places.get(waypoint_name)
            if not isinstance(place, str) or not place.strip():
                self._errors.append(
                    "navigation_waypoints.yaml: waypoint_nav.places missing "
                    f"non-empty ID/name for waypoint {waypoint_name!r}"
                )
        extra_places = set(places) - set(waypoints)
        if extra_places:
            self._errors.append(
                "navigation_waypoints.yaml: waypoint_nav.places contains "
                "unknown waypoints: " + ", ".join(sorted(extra_places))
            )
        if "place_ids" in waypoint_nav:
            self._errors.append(
                "navigation_waypoints.yaml: waypoint_nav.place_ids is no "
                "longer supported; use places with exact YAML IDs/names"
            )

        required_pose_fields = (
            "x",
            "y",
            "orientation_z",
            "orientation_w",
        )
        for waypoint_name, waypoint in waypoints.items():
            if not isinstance(waypoint, dict):
                self._errors.append(
                    f"Navigation waypoint {waypoint_name!r}: expected mapping"
                )
                continue
            for field_name in required_pose_fields:
                if not self._is_finite_number(waypoint.get(field_name)):
                    self._errors.append(
                        f"Navigation waypoint {waypoint_name!r}: "
                        f"{field_name} must be finite"
                    )
            orientation_z = waypoint.get("orientation_z")
            orientation_w = waypoint.get("orientation_w")
            if (
                self._is_finite_number(orientation_z)
                and self._is_finite_number(orientation_w)
                and math.hypot(
                    float(orientation_z),
                    float(orientation_w),
                ) < 1e-6
            ):
                self._errors.append(
                    f"Navigation waypoint {waypoint_name!r}: "
                    "orientation quaternion cannot be zero"
                )

        routed_actions: set[str] = set()
        for behavior_name, route in behavior_routes.items():
            if behavior_name not in self.behavior_tree_templates:
                self._errors.append(
                    "navigation_waypoints.yaml: unknown behavior "
                    f"{behavior_name!r}"
                )
                continue
            if not isinstance(route, dict):
                self._errors.append(
                    f"Navigation route {behavior_name!r}: expected mapping"
                )
                continue
            waypoint_name = route.get("waypoint")
            if waypoint_name not in waypoints:
                self._errors.append(
                    f"Navigation route {behavior_name!r}: unknown waypoint "
                    f"{waypoint_name!r}"
                )

            routed_stages = route.get("stages", [])
            if not isinstance(routed_stages, list) or not routed_stages:
                self._errors.append(
                    f"Navigation route {behavior_name!r}: "
                    "stages must be a non-empty list"
                )
                continue
            if any(not isinstance(stage_id, str) for stage_id in routed_stages):
                self._errors.append(
                    f"Navigation route {behavior_name!r}: "
                    "every stage name must be a string"
                )
                continue
            if len(routed_stages) != len(set(routed_stages)):
                self._errors.append(
                    f"Navigation route {behavior_name!r}: duplicate stages"
                )
            stage_configs = {
                str(stage.get("stage_id")): stage
                for stage in self.behavior_tree_templates[
                    behavior_name
                ].get("stages", [])
            }
            for stage_id in routed_stages:
                if stage_id not in stage_configs:
                    self._errors.append(
                        f"Navigation route {behavior_name!r}: unknown stage "
                        f"{stage_id!r}"
                    )
                    continue
                routed_actions.update(
                    str(candidate.get("unit_id"))
                    for candidate in stage_configs[stage_id].get(
                        "candidates",
                        [],
                    )
                    if (
                        isinstance(candidate, dict)
                        and candidate.get("unit_id")
                    )
                )

        for unit_id in stage_actions:
            if unit_id not in self.action_catalog:
                self._errors.append(
                    "navigation_waypoints.yaml: unknown action "
                    f"{unit_id!r}"
                )

        if len(stage_actions) != len(set(stage_actions)):
            self._errors.append(
                "navigation_waypoints.yaml: stage_actions contains duplicates"
            )
        missing_actions = routed_actions - set(stage_actions)
        if missing_actions:
            self._errors.append(
                "navigation_waypoints.yaml: routed stage actions without "
                "platform action mapping: "
                + ", ".join(sorted(missing_actions))
            )
        extra_actions = set(stage_actions) - routed_actions
        if extra_actions:
            self._errors.append(
                "navigation_waypoints.yaml: stage actions outside "
                "routed stages: "
                + ", ".join(sorted(extra_actions))
            )

        # ── Validate random navigation config ──────────────────────────
        self._validate_random_navigation_config(config, behavior_routes)

    def _validate_random_navigation_config(
        self,
        config: dict,
        behavior_routes: dict,
    ) -> None:
        """Validate behaviors using waypoint_nav's reserved random target."""
        random_behaviors = config.get("random_navigation_behaviors")
        random_place = config.get("random_navigation_place")
        random_timeout = config.get("random_navigation_result_timeout_sec")
        random_fixed_pool = config.get("random_navigation_fixed_pool")

        # All are optional together; skip if none present.
        if (
            random_behaviors is None
            and random_place is None
            and random_timeout is None
            and random_fixed_pool is None
        ):
            return

        # random_navigation_behaviors is always required when any config present.
        if random_behaviors is None:
            self._errors.append(
                "navigation_waypoints.yaml: "
                "random_navigation_behaviors is required when random "
                "navigation config is present"
            )
            return

        if random_place is None:
            self._errors.append(
                "navigation_waypoints.yaml: "
                "random_navigation_place is required"
            )

        # Validate random_navigation_behaviors.
        if not isinstance(random_behaviors, list):
            self._errors.append(
                "navigation_waypoints.yaml: "
                "random_navigation_behaviors must be a list"
            )
        else:
            for name in random_behaviors:
                if not isinstance(name, str):
                    self._errors.append(
                        "navigation_waypoints.yaml: "
                        f"random_navigation_behaviors contains non-string: "
                        f"{name!r}"
                    )
                elif name not in self.behavior_tree_templates:
                    self._errors.append(
                        "navigation_waypoints.yaml: unknown behavior in "
                        f"random_navigation_behaviors: {name!r}"
                    )
                elif name in behavior_routes:
                    self._errors.append(
                        "navigation_waypoints.yaml: behavior "
                        f"{name!r} in both random_navigation_behaviors "
                        "and behavior_routes"
                    )

        if not isinstance(random_place, str) or not random_place.strip():
            self._errors.append(
                "navigation_waypoints.yaml: "
                "random_navigation_place must be a non-empty string"
            )
        elif random_place.strip() not in {"K", "11"}:
            self._errors.append(
                "navigation_waypoints.yaml: random_navigation_place must be "
                "the reserved waypoint_nav target 'K' or '11'"
            )

        if (
            not self._is_finite_number(random_timeout)
            or float(random_timeout) <= 0.0
        ):
            self._errors.append(
                "navigation_waypoints.yaml: "
                "random_navigation_result_timeout_sec must be > 0"
            )

        for legacy_key in (
            "random_navigation_region",
            "random_navigation_bounds",
            "random_navigation_exclude_waypoints",
            "random_navigation_waypoints",
        ):
            if legacy_key in config:
                self._errors.append(
                    "navigation_waypoints.yaml: "
                    f"{legacy_key} is no longer supported; random navigation "
                    "must use random_navigation_place=K"
                )

        # Validate random_navigation_in_place_probability.
        in_place_probs = config.get("random_navigation_in_place_probability")
        if in_place_probs is not None:
            if not isinstance(in_place_probs, dict):
                self._errors.append(
                    "navigation_waypoints.yaml: "
                    "random_navigation_in_place_probability must be a mapping"
                )
            else:
                for name, prob in in_place_probs.items():
                    if not isinstance(name, str):
                        self._errors.append(
                            "navigation_waypoints.yaml: "
                            "random_navigation_in_place_probability "
                            f"contains non-string: {name!r}"
                        )
                        continue
                    if name not in self.behavior_tree_templates:
                        self._errors.append(
                            "navigation_waypoints.yaml: unknown behavior in "
                            "random_navigation_in_place_probability: "
                            f"{name!r}"
                        )
                    elif (
                        random_behaviors is not None
                        and name not in random_behaviors
                    ):
                        self._errors.append(
                            "navigation_waypoints.yaml: behavior "
                            f"{name!r} in "
                            "random_navigation_in_place_probability must "
                            "also be in random_navigation_behaviors"
                        )
                    if (
                        not isinstance(prob, (int, float))
                        or isinstance(prob, bool)
                        or not 0.0 <= float(prob) <= 1.0
                    ):
                        self._errors.append(
                            "navigation_waypoints.yaml: probability for "
                            f"{name!r} must be within [0, 1]"
                        )

        # Validate random_navigation_fixed_pool: an optional temporary
        # override which replaces waypoint_nav's server-generated random
        # target with one randomly picked fixed waypoint per navigation.
        # An empty list is valid and means "use the reserved K target".
        if random_fixed_pool is not None:
            if not isinstance(random_fixed_pool, list):
                self._errors.append(
                    "navigation_waypoints.yaml: "
                    "random_navigation_fixed_pool must be a list"
                )
            else:
                waypoints = config.get("waypoints")
                if not isinstance(waypoints, dict):
                    waypoints = {}
                waypoint_nav_config = config.get("waypoint_nav")
                if not isinstance(waypoint_nav_config, dict):
                    waypoint_nav_config = {}
                places = waypoint_nav_config.get("places")
                if not isinstance(places, dict):
                    places = {}
                for name in random_fixed_pool:
                    if not isinstance(name, str) or not name.strip():
                        self._errors.append(
                            "navigation_waypoints.yaml: "
                            "random_navigation_fixed_pool contains "
                            f"non-string: {name!r}"
                        )
                    elif name not in waypoints:
                        self._errors.append(
                            "navigation_waypoints.yaml: unknown waypoint in "
                            f"random_navigation_fixed_pool: {name!r}"
                        )
                    elif not isinstance(places.get(name), str) or not str(
                        places.get(name)
                    ).strip():
                        self._errors.append(
                            "navigation_waypoints.yaml: "
                            "random_navigation_fixed_pool entry "
                            f"{name!r} has no waypoint_nav.places mapping"
                        )

    def _validate_sound_config(self) -> None:
        """Validate voice-command and per-behavior sound configuration."""
        config = self.sound_config
        if not config:
            return  # optional, not required

        if not isinstance(config, dict):
            self._errors.append(
                "sound_config.yaml: bark_sound must be a mapping"
            )
            return

        if not isinstance(config.get("enabled"), bool):
            self._errors.append(
                "sound_config.yaml: bark_sound.enabled must be boolean"
            )

        file_path = config.get("file", "")
        if not isinstance(file_path, str) or not file_path.strip():
            self._errors.append(
                "sound_config.yaml: bark_sound.file must be a non-empty "
                "string"
            )

        behaviors = config.get("voice_command_behaviors")
        if not isinstance(behaviors, list) or not behaviors:
            self._errors.append(
                "sound_config.yaml: bark_sound.voice_command_behaviors "
                "must be a non-empty list"
            )
        else:
            for name in behaviors:
                if not isinstance(name, str):
                    self._errors.append(
                        "sound_config.yaml: "
                        "bark_sound.voice_command_behaviors contains "
                        f"non-string: {name!r}"
                    )
                elif name not in self.behavior_tree_templates:
                    self._errors.append(
                        "sound_config.yaml: unknown behavior in "
                        f"bark_sound.voice_command_behaviors: {name!r}"
                    )

        behavior_sounds = config.get("behavior_sounds", {})
        if not isinstance(behavior_sounds, dict):
            self._errors.append(
                "sound_config.yaml: bark_sound.behavior_sounds must be a mapping"
            )
        else:
            for name, file_name in behavior_sounds.items():
                if name not in self.behavior_tree_templates:
                    self._errors.append(
                        "sound_config.yaml: unknown behavior in "
                        f"bark_sound.behavior_sounds: {name!r}"
                    )
                # YAML null is an explicit, deterministic "no sound" mapping.
                # It is distinct from an absent key, which may be eligible for
                # the generic voice-command fallback.
                if file_name is not None and (
                    not isinstance(file_name, str) or not file_name.strip()
                ):
                    self._errors.append(
                        "sound_config.yaml: behavior sound path for "
                        f"{name!r} must be a non-empty string or null"
                    )
                elif file_name is not None:
                    self._warn_if_sound_file_missing(
                        f"bark_sound.behavior_sounds[{name!r}]", file_name
                    )

        self._validate_stage_sounds(config)

    def _validate_stage_sounds(self, config: dict[str, Any]) -> None:
        """Validate the Stage-scoped audio table against the stage contract."""
        stage_sounds = config.get("stage_sounds", {})
        if not isinstance(stage_sounds, dict):
            self._errors.append(
                "sound_config.yaml: bark_sound.stage_sounds must be a mapping"
            )
            return

        for name, per_behavior in stage_sounds.items():
            if name not in self.behavior_tree_templates:
                self._errors.append(
                    "sound_config.yaml: unknown behavior in "
                    f"bark_sound.stage_sounds: {name!r}"
                )
                continue
            if not isinstance(per_behavior, dict):
                self._errors.append(
                    "sound_config.yaml: stage_sounds entry for "
                    f"{name!r} must be a mapping of stage_id to audio path"
                )
                continue

            # Mirror the node's stage_id resolution exactly, so a name accepted
            # here is a name the execution loop can actually look up.
            known_stages = {
                stage.get("stage_id")
                or stage.get("stage_name")
                or f"stage_{index}"
                for index, stage in enumerate(
                    self.behavior_tree_templates[name].get("stages", [])
                )
            }
            for stage_id, file_name in per_behavior.items():
                if stage_id not in known_stages:
                    self._errors.append(
                        "sound_config.yaml: unknown stage in "
                        f"bark_sound.stage_sounds.{name}: {stage_id!r}"
                    )
                    continue
                # A stage sound is always an explicit file: there is no
                # fallback for it, so null carries no meaning here.
                if not isinstance(file_name, str) or not file_name.strip():
                    self._errors.append(
                        "sound_config.yaml: stage sound path for "
                        f"{name}.{stage_id} must be a non-empty string"
                    )
                    continue
                self._warn_if_sound_file_missing(
                    f"bark_sound.stage_sounds.{name}.{stage_id}", file_name
                )

    def _warn_if_sound_file_missing(self, label: str, file_name: str) -> None:
        """Warn — never fail — when a configured audio file is not on disk.

        A missing asset is not worth refusing to start over, but it must not be
        silent either: the failure path runs through ``SoundPlayer.play()``
        returning False and the node skipping its "sound started" log, so an
        absent file would otherwise look like audio that simply never fires.
        """
        path = Path(str(file_name)).expanduser()
        if not path.is_absolute():
            path = self.config_dir / path
        if not path.is_file():
            logger.warning(
                "sound_config.yaml: %s audio file not found: %s — if the asset "
                "was just added, re-run colcon build; config/sounds/ is "
                "installed as a real directory, so new files are not linked "
                "until the package is rebuilt",
                label,
                path,
            )

    def _validate_wake_orientation_config(self) -> None:
        config = self.wake_orientation_config
        if not config:
            return

        if not isinstance(config.get("enabled"), bool):
            self._errors.append(
                "wake_orientation.yaml: enabled must be boolean"
            )

        action_name = config.get("action_name")
        if not isinstance(action_name, str) or not action_name.startswith("/"):
            self._errors.append(
                "wake_orientation.yaml: action_name must be an absolute "
                "ROS action name"
            )

        frame_id = config.get("required_frame_id")
        if not isinstance(frame_id, str) or not frame_id.strip():
            self._errors.append(
                "wake_orientation.yaml: required_frame_id must be non-empty"
            )

        for key in (
            "server_timeout_sec",
            "result_timeout_sec",
            "time_allowance_sec",
        ):
            value = config.get(key)
            if not self._is_finite_number(value) or float(value) <= 0.0:
                self._errors.append(
                    f"wake_orientation.yaml: {key} must be > 0"
                )

        offset = config.get("angle_zero_offset_deg")
        if not self._is_finite_number(offset):
            self._errors.append(
                "wake_orientation.yaml: angle_zero_offset_deg must be finite"
            )

        direction = config.get("angle_direction_sign")
        if (
            not self._is_finite_number(direction)
            or float(direction) == 0.0
        ):
            self._errors.append(
                "wake_orientation.yaml: angle_direction_sign must be "
                "finite and non-zero"
            )

        deadband = config.get("angle_deadband_deg")
        if (
            not self._is_finite_number(deadband)
            or not 0.0 <= float(deadband) < 180.0
        ):
            self._errors.append(
                "wake_orientation.yaml: angle_deadband_deg must be within "
                "[0, 180)"
            )

        if not isinstance(
            config.get("linear_array_back_search_enabled"), bool
        ):
            self._errors.append(
                "wake_orientation.yaml: "
                "linear_array_back_search_enabled must be boolean"
            )
        for key in (
            "visual_confirm_timeout_sec",
            "visual_max_age_ms",
        ):
            value = config.get(key)
            if not self._is_finite_number(value) or float(value) <= 0.0:
                self._errors.append(
                    f"wake_orientation.yaml: {key} must be > 0"
                )
        confidence = config.get("visual_min_confidence")
        if (
            not self._is_finite_number(confidence)
            or not 0.0 <= float(confidence) <= 1.0
        ):
            self._errors.append(
                "wake_orientation.yaml: visual_min_confidence must be "
                "within [0, 1]"
            )

        routes = self.controller_routes.get("routes", {})
        if routes.get("ACT_INTERACT_RESPOND_CALL") != "wake_orientation":
            self._errors.append(
                "ACT_INTERACT_RESPOND_CALL controller route must be "
                "'wake_orientation'"
            )

    def _validate_visual_target_approach_config(self) -> None:
        config = self.visual_target_approach_config
        if not config:
            return
        prefix = "visual_target_approach.yaml"
        if not isinstance(config.get("enabled"), bool):
            self._errors.append(f"{prefix}: enabled must be boolean")
        positive_fields = (
            "publish_rate_hz",
            "visual_timeout_sec",
            "acquire_timeout_sec",
            "lost_timeout_sec",
            "approach_timeout_sec",
            "minimum_stop_distance_m",
            "minimum_safe_distance_m",
            "distance_deadband_m",
            "distance_hysteresis_m",
            "arrival_hold_sec",
            "linear_gain",
            "angular_gain",
            "max_linear_x",
            "max_angular_z",
            "max_linear_accel",
            "max_angular_accel",
            "heading_deadband",
            "max_heading_error",
            "bbox_target_height",
            "bbox_height_deadband",
            "bbox_height_hysteresis",
        )
        for key in positive_fields:
            value = config.get(key)
            if not self._is_finite_number(value) or float(value) <= 0.0:
                self._errors.append(f"{prefix}: {key} must be > 0")
        confidence = config.get("min_confidence")
        if (
            not self._is_finite_number(confidence)
            or not 0.0 <= float(confidence) <= 1.0
        ):
            self._errors.append(
                f"{prefix}: min_confidence must be within [0, 1]"
            )
        if not isinstance(config.get("allow_bbox_distance_fallback"), bool):
            self._errors.append(
                f"{prefix}: allow_bbox_distance_fallback must be boolean"
            )
        reacquire_frames = config.get("reacquire_min_consecutive_frames")
        if (
            isinstance(reacquire_frames, bool)
            or not isinstance(reacquire_frames, int)
            or reacquire_frames < 1
        ):
            self._errors.append(
                f"{prefix}: reacquire_min_consecutive_frames must be "
                "an integer >= 1"
            )

        object_stream = config.get("object_detection_stream")
        if not isinstance(object_stream, dict):
            self._errors.append(
                f"{prefix}: object_detection_stream must be a mapping"
            )
        else:
            if object_stream.get("required") is not True:
                self._errors.append(
                    f"{prefix}: object_detection_stream.required must be true"
                )
            for key in ("service_name", "topic"):
                value = object_stream.get(key)
                if not isinstance(value, str) or not value.strip():
                    self._errors.append(
                        f"{prefix}: object_detection_stream.{key} must be "
                        "a non-empty string"
                    )
            for key in (
                "rate_hz",
                "service_timeout_sec",
                "lease_margin_sec",
            ):
                value = object_stream.get(key)
                if not self._is_finite_number(value) or float(value) <= 0.0:
                    self._errors.append(
                        f"{prefix}: object_detection_stream.{key} must be > 0"
                    )
            stream_confidence = object_stream.get("confidence")
            if (
                not self._is_finite_number(stream_confidence)
                or not 0.0 <= float(stream_confidence) <= 1.0
            ):
                self._errors.append(
                    f"{prefix}: object_detection_stream.confidence must be "
                    "within [0, 1]"
                )

        policies = config.get("behavior_policies")
        if not isinstance(policies, dict) or not policies:
            self._errors.append(
                f"{prefix}: behavior_policies must be a non-empty mapping"
            )
            return
        for behavior_name, policy in policies.items():
            if behavior_name not in self.behavior_tree_templates:
                self._errors.append(
                    f"{prefix}: unknown behavior policy {behavior_name!r}"
                )
                continue
            if not isinstance(policy, dict):
                self._errors.append(
                    f"{prefix}: policy {behavior_name!r} must be a mapping"
                )
                continue
            if policy.get("target_type") not in {
                "human",
                "animal",
                "object",
            }:
                self._errors.append(
                    f"{prefix}: policy {behavior_name!r} has invalid "
                    "target_type"
                )
            distance_mode = policy.get("distance_mode", "metric")
            if distance_mode not in {
                "metric",
                "metric_then_bbox",
                "bbox_height",
            }:
                self._errors.append(
                    f"{prefix}: policy {behavior_name!r} has invalid "
                    "distance_mode"
                )
            if (
                distance_mode in {"metric_then_bbox", "bbox_height"}
                and policy.get("target_type") != "human"
            ):
                self._errors.append(
                    f"{prefix}: bbox distance mode is allowed only for "
                    f"human policy {behavior_name!r}"
                )
            required_identity = policy.get("required_identity")
            if required_identity is not None and (
                policy.get("target_type") != "human"
                or not isinstance(required_identity, str)
                or not required_identity.strip()
            ):
                self._errors.append(
                    f"{prefix}: policy {behavior_name!r}.required_identity "
                    "requires a non-empty human identity"
                )
            for key in (
                "desired_distance_m",
                "minimum_safe_distance_m",
                "target_max_age_ms",
                "target_lost_timeout_sec",
            ):
                value = policy.get(key)
                if not self._is_finite_number(value) or float(value) <= 0.0:
                    self._errors.append(
                        f"{prefix}: policy {behavior_name!r}.{key} "
                        "must be > 0"
                    )
            for key in (
                "max_linear_x",
                "max_linear_accel",
                "bbox_target_height",
            ):
                value = policy.get(key)
                if value is not None and (
                    not self._is_finite_number(value)
                    or float(value) <= 0.0
                ):
                    self._errors.append(
                        f"{prefix}: policy {behavior_name!r}.{key} "
                        "must be > 0"
                    )
            for key in (
                "linear_gain",
                "arrival_hold_sec",
                "bbox_height_deadband",
            ):
                value = policy.get(key)
                if value is not None and (
                    not self._is_finite_number(value)
                    or float(value) < 0.0
                ):
                    self._errors.append(
                        f"{prefix}: policy {behavior_name!r}.{key} "
                        "must be >= 0"
                    )
            bbox_target_height = policy.get("bbox_target_height")
            if (
                self._is_finite_number(bbox_target_height)
                and not 0.05 <= float(bbox_target_height) <= 1.0
            ):
                self._errors.append(
                    f"{prefix}: policy {behavior_name!r}."
                    "bbox_target_height must be within [0.05, 1.0]"
                )
            policy_confidence = policy.get("min_confidence")
            if policy_confidence is not None and (
                not self._is_finite_number(policy_confidence)
                or not 0.0 <= float(policy_confidence) <= 1.0
            ):
                self._errors.append(
                    f"{prefix}: policy {behavior_name!r}.min_confidence "
                    "must be within [0, 1]"
                )
            desired = policy.get("desired_distance_m")
            minimum = policy.get("minimum_safe_distance_m")
            if (
                self._is_finite_number(desired)
                and self._is_finite_number(minimum)
                and float(desired) < float(minimum)
            ):
                self._errors.append(
                    f"{prefix}: policy {behavior_name!r} desired distance "
                    "must be >= minimum safe distance"
                )

        routes = self.controller_routes.get("routes", {})
        if routes.get("ACT_APPROACH_VISUAL_TARGET") != (
            "visual_target_approach"
        ):
            self._errors.append(
                "ACT_APPROACH_VISUAL_TARGET controller route must be "
                "'visual_target_approach'"
            )
        approach_unit = self.action_catalog.get("ACT_APPROACH_VISUAL_TARGET", {})
        if approach_unit.get("requires_controller") is not True:
            self._errors.append(
                "action_catalog.yaml: ACT_APPROACH_VISUAL_TARGET must set "
                "requires_controller=true"
            )

    def _validate_uwb_follow_config(self) -> None:
        config = self.uwb_follow_config
        if not config:
            return
        prefix = "uwb_follow.yaml"
        if not isinstance(config.get("enabled"), bool):
            self._errors.append(f"{prefix}: enabled must be boolean")
        for key in (
            "setup_script",
            "serial_device",
            "cmd_vel_topic",
            "aoa_package",
            "aoa_executable",
            "follow_package",
            "follow_launch_file",
        ):
            value = config.get(key)
            if not isinstance(value, str) or not value.strip():
                self._errors.append(f"{prefix}: {key} must be a non-empty string")
        fob_id = config.get("target_fob_id")
        if fob_id is not None and not isinstance(fob_id, str):
            self._errors.append(f"{prefix}: target_fob_id must be a string")
        for key in ("startup_grace_sec", "shutdown_timeout_sec"):
            value = config.get(key)
            if not self._is_finite_number(value) or float(value) <= 0.0:
                self._errors.append(f"{prefix}: {key} must be > 0")
        stop_count = config.get("stop_publish_count")
        if (
            not isinstance(stop_count, int)
            or isinstance(stop_count, bool)
            or not 1 <= stop_count <= 20
        ):
            self._errors.append(
                f"{prefix}: stop_publish_count must be within [1, 20]"
            )
        routes = self.controller_routes.get("routes", {})
        if routes.get("ACT_INTERACT_FOLLOW_OWNER") != "uwb_follow":
            self._errors.append(
                "ACT_INTERACT_FOLLOW_OWNER controller route must be "
                "'uwb_follow'"
            )
        follow_unit = self.action_catalog.get("ACT_INTERACT_FOLLOW_OWNER", {})
        if follow_unit.get("requires_controller") is not True:
            self._errors.append(
                "action_catalog.yaml: ACT_INTERACT_FOLLOW_OWNER must set "
                "requires_controller=true"
            )
        self._validate_uwb_ros_interface(config, prefix)

    def _validate_uwb_ros_interface(
        self, config: Mapping[str, Any], prefix: str
    ) -> None:
        """Validate the lite3-only go2_uwb_behavior client block.

        Optional on purpose: Go2 uses its external process pipeline and has
        no such block, so requiring it would stop Go2 from loading.
        """
        section = config.get("ros_interface")
        if section is None:
            return
        if not isinstance(section, Mapping):
            self._errors.append(f"{prefix}: ros_interface must be a mapping")
            return
        for key in ("follow_action", "set_behavior_service"):
            value = section.get(key)
            if not isinstance(value, str) or not value.strip():
                self._errors.append(
                    f"{prefix}: ros_interface.{key} must be a non-empty string"
                )
        for key in ("server_timeout_sec", "ready_timeout_sec"):
            value = section.get(key)
            if not self._is_finite_number(value) or float(value) <= 0.0:
                self._errors.append(
                    f"{prefix}: ros_interface.{key} must be > 0"
                )

    @staticmethod
    def _is_finite_number(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
        )

    # ── Accessors ─────────────────────────────────────────────────────────

    @property
    def config_dir(self) -> Path:
        return self._dir

    def get_behavior_template(self, name: str) -> dict[str, Any] | None:
        """Return a behavior-tree template by exact direct name."""
        return self.behavior_tree_templates.get(name)

    def get_all_behavior_templates(self) -> dict[str, Any]:
        """Return the strict behavior-tree contract."""
        return dict(self.behavior_tree_templates)

    def get_behavior_names(self) -> set[str]:
        """Return all directly executable behavior names."""
        return set(self.get_all_behavior_templates())

    def get_action_config(self, unit_id: str) -> dict[str, Any]:
        return self.action_catalog.get(unit_id, {})

    def get_go2_action_sequences(self) -> dict[str, list[dict[str, Any]]]:
        """Expand direct and reusable-group Go2 mappings by exact ACT_* ID."""
        result = {
            str(unit_id): [dict(item) for item in sequence]
            for unit_id, sequence in self.go2_sport_config.get(
                "action_sequences", {}
            ).items()
        }
        for group_config in self.go2_sport_config.get(
            "action_groups", {}
        ).values():
            sequence = [
                dict(item) for item in group_config.get("sequence", [])
            ]
            for unit_id in group_config.get("actions", []):
                result[str(unit_id)] = [dict(item) for item in sequence]
        return result

    def get_lite3_action_plans(self) -> dict[str, dict[str, Any]]:
        """Expand direct and reusable Lite3 plans by exact ``ACT_*`` ID."""
        result = {
            str(unit_id): {
                **dict(plan),
                "plan_id": str(plan.get("plan_id", unit_id)),
                "sequence": [dict(item) for item in plan.get("sequence", [])],
            }
            for unit_id, plan in self.lite3_action_config.get(
                "action_sequences", {}
            ).items()
        }
        for group_name, plan in self.lite3_action_config.get(
            "action_groups", {}
        ).items():
            for unit_id in plan.get("actions", []):
                result[str(unit_id)] = {
                    **{
                        key: value
                        for key, value in plan.items()
                        if key not in {"actions", "sequence"}
                    },
                    "plan_id": str(plan.get("plan_id", group_name)),
                    "sequence": [
                        dict(item) for item in plan.get("sequence", [])
                    ],
                }
        return result

    def get_controller_routes(
        self,
        chassis_type: str | None = None,
    ) -> dict[str, str]:
        """Return the effective routes for Go2 or Lite3."""
        routes = dict(self.controller_routes.get("routes", {}))
        selected = str(chassis_type or "go2").strip().lower()
        if selected not in {"go2", "lite3"}:
            raise ValueError(f"Unsupported chassis_type={selected!r}")
        if selected == "lite3":
            # Preserve shared closed-loop adapters and mark unsupported
            # manipulation actions before applying Lite3 plans.
            for unit_id, route in list(routes.items()):
                if route == "perception_navigation_manipulation":
                    routes[unit_id] = "unsupported"
            for unit_id in self.get_lite3_action_plans():
                routes[str(unit_id)] = "lite3"
            routes.update({
                str(unit_id): str(route)
                for unit_id, route in self.lite3_action_config.get(
                    "controller_route_overrides", {}
                ).items()
            })
            return routes
        for unit_id in self.get_go2_action_sequences():
            routes[str(unit_id)] = "go2"
        routes.update({
            str(unit_id): str(route)
            for unit_id, route in self.go2_sport_config.get(
                "controller_route_overrides", {}
            ).items()
        })
        return routes
