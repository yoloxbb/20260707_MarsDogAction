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
from typing import Any

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
        self.agv_motion_config: dict[str, Any] = {}
        self.navigation_config: dict[str, Any] = {}
        self.wake_orientation_config: dict[str, Any] = {}
        self._errors: list[str] = []

    # Mapping of filename → (attr, wrapper_key)
    # wrapper_key=None means the YAML top level IS the data (no unwrap needed).
    _CONFIG_FILES: list[tuple[str, str, str | None]] = [
        ("behavior_tree_actions.yaml", "behavior_tree_templates", "behaviors"),
        ("action_catalog.yaml", "action_catalog", "action_units"),
        ("controller_routes.yaml", "controller_routes", None),
        ("safety_policies.yaml", "safety_policies", None),
        ("agv_motion_groups.yaml", "agv_motion_config", None),
        ("navigation_waypoints.yaml", "navigation_config", None),
        ("wake_orientation.yaml", "wake_orientation_config", None),
    ]

    _REQUIRED_CONFIGS = {
        "behavior_tree_actions.yaml",
        "action_catalog.yaml",
        "controller_routes.yaml",
        "agv_motion_groups.yaml",
        "navigation_waypoints.yaml",
        "wake_orientation.yaml",
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
        self._validate_agv_motion_config()
        self._validate_navigation_config()
        self._validate_wake_orientation_config()

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
        for unit_id in routes:
            if unit_id != "_default" and unit_id not in self.action_catalog:
                self._errors.append(
                    f"controller_routes.yaml: unknown action {unit_id!r}"
                )

    def _validate_agv_motion_config(self) -> None:
        config = self.agv_motion_config
        if not config:
            return

        groups = config.get("motion_groups", {})
        action_groups = config.get("action_motion_groups", {})
        limits = config.get("limits", {})
        routes = self.controller_routes.get("routes", {})

        if not isinstance(groups, dict) or not groups:
            self._errors.append(
                "agv_motion_groups.yaml: motion_groups must be a non-empty mapping"
            )
            return
        if not isinstance(action_groups, dict) or not action_groups:
            self._errors.append(
                "agv_motion_groups.yaml: action_motion_groups must be "
                "a non-empty mapping"
            )
            return

        rate = config.get("publish_rate_hz", 10.0)
        if not self._is_finite_number(rate) or not 1.0 <= float(rate) <= 100.0:
            self._errors.append(
                "agv_motion_groups.yaml: publish_rate_hz must be within [1, 100]"
            )

        stop_count = config.get("stop_publish_count", 3)
        if (
            not isinstance(stop_count, int)
            or isinstance(stop_count, bool)
            or not 1 <= stop_count <= 20
        ):
            self._errors.append(
                "agv_motion_groups.yaml: stop_publish_count must be within [1, 20]"
            )

        for key in ("max_linear_x", "max_linear_y", "max_angular_z"):
            value = limits.get(key)
            if not self._is_finite_number(value) or float(value) < 0.0:
                self._errors.append(
                    f"agv_motion_groups.yaml: limits.{key} must be >= 0"
                )

        for group_name, segments in groups.items():
            # Dict-type groups (e.g. pick_random)
            if isinstance(segments, dict):
                self._validate_motion_group_dict(group_name, segments, limits)
                continue
            # List-type groups (traditional fixed segments)
            if not isinstance(segments, list) or not segments:
                self._errors.append(
                    f"AGV motion group {group_name!r}: segments must be non-empty"
                )
                continue
            for index, segment in enumerate(segments):
                if not isinstance(segment, dict):
                    self._errors.append(
                        f"AGV motion group {group_name!r} segment {index}: "
                        "expected mapping"
                    )
                    continue
                duration = segment.get("duration_sec")
                if not self._is_finite_number(duration) or float(duration) <= 0.0:
                    self._errors.append(
                        f"AGV motion group {group_name!r} segment {index}: "
                        "duration_sec must be > 0"
                    )
                for key in ("linear_x", "linear_y", "angular_z"):
                    value = segment.get(key, 0.0)
                    if not self._is_finite_number(value):
                        self._errors.append(
                            f"AGV motion group {group_name!r} segment {index}: "
                            f"{key} must be finite"
                        )

        for unit_id, group_name in action_groups.items():
            if unit_id not in self.action_catalog:
                self._errors.append(
                    f"agv_motion_groups.yaml: unknown action {unit_id!r}"
                )
            if group_name not in groups:
                self._errors.append(
                    f"AGV action {unit_id!r}: unknown motion group {group_name!r}"
                )
            if routes.get(unit_id) != "agv":
                self._errors.append(
                    f"AGV action {unit_id!r}: controller route must be 'agv'"
                )

        for unit_id, route in routes.items():
            if route == "agv" and unit_id not in action_groups:
                self._errors.append(
                    f"AGV controller route {unit_id!r}: no motion group configured"
                )

    def _validate_motion_group_dict(
        self, group_name: str, config: dict, limits: dict,
    ) -> None:
        """Validate a dict-type motion group (e.g. ``pick_random``)."""
        group_type = config.get("type", "")
        if group_type == "pick_random":
            candidates = config.get("candidates", [])
            if not isinstance(candidates, list) or not candidates:
                self._errors.append(
                    f"AGV motion group {group_name!r}: pick_random "
                    "candidates must be non-empty"
                )
                return
            for i, cand in enumerate(candidates):
                if not isinstance(cand, dict):
                    self._errors.append(
                        f"AGV motion group {group_name!r} candidate {i}: "
                        "expected mapping"
                    )
                    continue
                for key in ("linear_x", "linear_y", "angular_z"):
                    value = cand.get(key, 0.0)
                    if not self._is_finite_number(value):
                        self._errors.append(
                            f"AGV motion group {group_name!r} candidate {i}: "
                            f"{key} must be finite"
                        )
            dmin = config.get("duration_min_sec")
            dmax = config.get("duration_max_sec")
            if not self._is_finite_number(dmin) or float(dmin) <= 0.0:
                self._errors.append(
                    f"AGV motion group {group_name!r}: "
                    "duration_min_sec must be > 0"
                )
            if not self._is_finite_number(dmax) or float(dmax) <= 0.0:
                self._errors.append(
                    f"AGV motion group {group_name!r}: "
                    "duration_max_sec must be > 0"
                )
            if (
                self._is_finite_number(dmin)
                and self._is_finite_number(dmax)
                and float(dmin) >= float(dmax)
            ):
                self._errors.append(
                    f"AGV motion group {group_name!r}: "
                    "duration_min_sec must be < duration_max_sec"
                )
        else:
            self._errors.append(
                f"AGV motion group {group_name!r}: unknown type {group_type!r}"
            )

    def _validate_navigation_config(self) -> None:
        config = self.navigation_config
        if not config:
            return

        waypoints = config.get("waypoints", {})
        behavior_routes = config.get("behavior_routes", {})
        action_groups = config.get("action_motion_groups", {})
        motion_groups = self.agv_motion_config.get("motion_groups", {})

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
        if not isinstance(action_groups, dict) or not action_groups:
            self._errors.append(
                "navigation_waypoints.yaml: action_motion_groups must be a "
                "non-empty mapping"
            )
            return

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

        for unit_id, group_name in action_groups.items():
            if unit_id not in self.action_catalog:
                self._errors.append(
                    "navigation_waypoints.yaml: unknown action "
                    f"{unit_id!r}"
                )
            if group_name not in motion_groups:
                self._errors.append(
                    f"Navigation action {unit_id!r}: unknown motion group "
                    f"{group_name!r}"
                )
            if (
                unit_id in self.action_catalog
                and group_name in motion_groups
            ):
                action_timeout = self.action_catalog[unit_id].get(
                    "timeout_sec",
                    0.0,
                )
                group_config = motion_groups[group_name]
                if isinstance(group_config, dict):
                    # pick_random: worst-case = duration_max_sec
                    group_duration = float(
                        group_config.get("duration_max_sec", 0.0)
                    )
                else:
                    group_duration = sum(
                        float(segment["duration_sec"])
                        for segment in group_config
                    )
                if (
                    self._is_finite_number(action_timeout)
                    and float(action_timeout) > 0.0
                    and group_duration > float(action_timeout)
                ):
                    self._errors.append(
                        f"Navigation action {unit_id!r}: motion group "
                        f"{group_name!r} duration {group_duration:.3f}s "
                        f"exceeds action timeout {float(action_timeout):.3f}s"
                    )

        missing_actions = routed_actions - set(action_groups)
        if missing_actions:
            self._errors.append(
                "navigation_waypoints.yaml: routed stage actions without "
                "motion groups: "
                + ", ".join(sorted(missing_actions))
            )
        extra_actions = set(action_groups) - routed_actions
        if extra_actions:
            self._errors.append(
                "navigation_waypoints.yaml: action motion groups outside "
                "routed stages: "
                + ", ".join(sorted(extra_actions))
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

        routes = self.controller_routes.get("routes", {})
        if routes.get("ACT_INTERACT_RESPOND_CALL") != "wake_orientation":
            self._errors.append(
                "ACT_INTERACT_RESPOND_CALL controller route must be "
                "'wake_orientation'"
            )

    @staticmethod
    def _is_finite_number(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
        )

    # ── Accessors ─────────────────────────────────────────────────────────

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

    def get_controller_routes(self) -> dict[str, str]:
        return dict(self.controller_routes.get("routes", {}))
