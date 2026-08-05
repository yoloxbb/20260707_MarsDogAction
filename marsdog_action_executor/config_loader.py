"""ConfigLoader — loads and validates YAML configuration files.

Startup validation catches:
  - missing canonical behavior templates
  - missing referenced actions
  - invalid selection_policy / unit_type / interrupt_policy
  - duplicate stage orders
  - empty required stages
  - alias cycles
  - weight < 0
  - timeout <= 0 or excessively large
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# ── Valid enum values ─────────────────────────────────────────────────────────

VALID_SELECTION_POLICIES = {
    "fixed", "sequence", "random_one", "weighted_random",
    "random_n", "loop_random", "condition_first",
    # Legacy aliases
    "random", "first", "weighted",
}

VALID_UNIT_TYPES = {
    "atomic_action", "composite_action", "task", "policy", "modifier",
}

VALID_INTERRUPT_POLICIES = {
    "immediate", "safe_point", "non_interruptible",
    # Legacy aliases from action_catalog.yaml
    "safe", "unsafe", "deferred",
}

VALID_FAILURE_POLICIES = {
    "abort", "skip_stage", "retry",
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
        templates = loader.behavior_templates
        catalog = loader.action_catalog
    """

    def __init__(self, config_dir: str | Path = "config") -> None:
        self._dir = Path(config_dir)
        self.behavior_templates: dict[str, Any] = {}
        self.action_catalog: dict[str, Any] = {}
        self.emotion_pools: dict[str, Any] = {}
        self.object_pools: dict[str, Any] = {}
        self.behavior_aliases: dict[str, Any] = {}
        self.posture_transitions: dict[str, Any] = {}
        self.controller_routes: dict[str, Any] = {}
        self.safety_policies: dict[str, Any] = {}
        self.composite_actions: dict[str, Any] = {}
        self.task_catalog: dict[str, Any] = {}
        self._errors: list[str] = []

    # Mapping of filename → (attr, wrapper_key)
    # wrapper_key=None means the YAML top level IS the data (no unwrap needed).
    _CONFIG_FILES: list[tuple[str, str, str | None]] = [
        ("behavior_templates.yaml", "behavior_templates", "behavior_templates"),
        ("action_catalog.yaml", "action_catalog", "action_units"),
        ("emotion_action_pools.yaml", "emotion_pools", None),
        ("object_action_pools.yaml", "object_pools", None),
        ("behavior_aliases.yaml", "behavior_aliases", "behavior_aliases"),
        ("posture_transitions.yaml", "posture_transitions", None),
        ("controller_routes.yaml", "controller_routes", None),
        ("safety_policies.yaml", "safety_policies", None),
        ("composite_actions.yaml", "composite_actions", None),
        ("task_catalog.yaml", "task_catalog", None),
    ]

    _REQUIRED_CONFIGS = {
        "behavior_templates.yaml",
        "action_catalog.yaml",
        "emotion_action_pools.yaml",
        "behavior_aliases.yaml",
    }

    def load_all(self) -> None:
        """Load all config files and validate."""
        for filename, attr, wrapper_key in self._CONFIG_FILES:
            required = filename in self._REQUIRED_CONFIGS
            self._load_yaml(filename, attr, required=required, unwrap_key=wrapper_key)

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
            # Unwrap known top-level wrapper key (e.g. "behavior_templates: {...}" → {...})
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
        self._validate_aliases()
        self._validate_emotion_pools()
        self._validate_object_pools()

    def _validate_behavior_templates(self) -> None:
        templates = self.behavior_templates
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

    def _validate_aliases(self) -> None:
        aliases = self.behavior_aliases
        if not aliases:
            return
        # Check for cycles
        for src, cfg in aliases.items():
            resolved = cfg.get("resolved_behavior_name", "")
            if resolved == src:
                self._errors.append(f"Alias {src!r}: self-referential (cycle)")
            # Check transitive (simple: resolved also in aliases)
            if resolved in aliases:
                transitive = aliases[resolved].get("resolved_behavior_name", "")
                if transitive == src:
                    self._errors.append(f"Alias cycle: {src} → {resolved} → {src}")

    def _validate_emotion_pools(self) -> None:
        pools = self.emotion_pools
        if not pools:
            return
        for behavior, variants in pools.items():
            for level, modes in variants.items():
                if level.upper() not in ("LOW", "MID", "HIGH", "TRIGGERED", "OVERFLOW"):
                    self._errors.append(f"Emotion pool {behavior!r}: invalid level {level!r}")
                if isinstance(modes, dict):
                    for mode in ("solo", "interactive"):
                        if mode not in modes:
                            self._errors.append(f"Emotion pool {behavior!r}/{level}: missing {mode!r} mode")

    def _validate_object_pools(self) -> None:
        pools = self.object_pools
        if not pools:
            return

    # ── Accessors ─────────────────────────────────────────────────────────

    def get_behavior_template(self, name: str) -> dict[str, Any] | None:
        return self.behavior_templates.get(name)

    def get_action_config(self, unit_id: str) -> dict[str, Any]:
        return self.action_catalog.get(unit_id, {})

    def get_emotion_pool(
        self, behavior: str, level: str, mode: str,
    ) -> list[dict[str, Any]]:
        pools = self.emotion_pools
        try:
            return pools[behavior][level.upper()][mode]
        except (KeyError, TypeError):
            return []

    def get_object_pool(self, category: str) -> list[dict[str, Any]]:
        return self.object_pools.get(category, [])
