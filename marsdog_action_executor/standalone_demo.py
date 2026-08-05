"""Standalone demo — runs the v2 pipeline without ROS2.

Demonstrates the full GoalParser → Resolver → StageExecutor → ResultEvaluator
pipeline. Suitable for CI, development, and demos.

Usage::

    uv run marsdog-action-demo                  # default: sleepNow
    uv run marsdog-action-demo eatNormally      # specify behavior
    uv run marsdog-action-demo comeHere 42     # with fixed random seed
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

from .behavior_resolver import BehaviorResolver
from .config_loader import ConfigLoader
from .eligibility_checker import EligibilityChecker
from .goal_parser import GoalParser
from .interrupt_manager import InterruptManager
from .posture_manager import PostureManager
from .result_evaluator import ResultEvaluator
from .ros_node import _resolve_emotion_pool, _resolve_sleep_depth_candidates
from .stage_executor import StageExecutor


def main() -> None:
    # ── Parse arguments ──────────────────────────────────────────────────
    behavior = "sleepNow"
    seed = None

    if len(sys.argv) > 1:
        behavior = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            seed = int(sys.argv[2])
        except ValueError:
            print(f"Invalid seed: {sys.argv[2]!r} — must be an integer")
            sys.exit(1)

    # ── Load configs ─────────────────────────────────────────────────────
    config_dir = Path(__file__).resolve().parent.parent / "config"
    loader = ConfigLoader(config_dir)
    loader.load_all()

    # ── Build pipeline ────────────────────────────────────────────────────
    parser = GoalParser()
    resolver = BehaviorResolver(aliases=loader.behavior_aliases)
    stage_exec = StageExecutor(
        eligibility_checker=EligibilityChecker(action_catalog=loader.action_catalog),
        posture_manager=PostureManager(),
        interrupt_manager=InterruptManager(),
        action_catalog=loader.action_catalog,
    )
    evaluator = ResultEvaluator()

    # ── Parse & resolve ───────────────────────────────────────────────────
    ctx = parser.parse(behavior, "{}", seed=seed)
    ctx = resolver.resolve(ctx)

    if not ctx.is_valid:
        print(f"Invalid behavior: {behavior!r}")
        print(f"Reason: {ctx.error_reason}")
        avail = sorted(loader.behavior_templates.keys())
        print(f"Available: {', '.join(avail)}")
        sys.exit(1)

    canonical = ctx.resolved_behavior_name
    template = loader.get_behavior_template(canonical)
    if template is None:
        print(f"No template for: {canonical}")
        sys.exit(1)

    stages = template["stages"]
    print(f"Behavior:    {behavior}")
    print(f"Resolved:    {canonical}")
    print(f"Stages:      {len(stages)}")
    if seed is not None:
        print(f"Seed:        {seed}")
    print()

    # ── Execute stages ────────────────────────────────────────────────────
    stage_results: dict[str, bool] = {}
    t0 = time.time()

    for i, stage_cfg in enumerate(stages):
        # Resolve special pools
        stage_cfg = _resolve_emotion_pool(stage_cfg, canonical, ctx, loader)
        stage_cfg = _resolve_sleep_depth_candidates(stage_cfg, ctx)

        sid = stage_cfg.get("stage_id") or stage_cfg.get("stage_name", f"stage_{i}")
        sel_policy = stage_cfg.get("selection_policy", "random_one")

        result = stage_exec.execute_stage(stage_cfg, ctx)
        stage_results[sid] = result.success

        status = "✓" if result.success else "✗"
        print(
            f"  [{i+1}/{len(stages)}] {sid:20s} [{sel_policy:15s}] "
            f"→ {result.unit_id or '(loop)':40s} {status}"
        )

    elapsed = time.time() - t0

    # ── Evaluate ──────────────────────────────────────────────────────────
    br = evaluator.evaluate(ctx, stage_results)
    print()
    print(f"Result:      {br.status} (success={br.success})")
    print(f"Duration:    {elapsed:.1f}s")
    print(f"Stages done: {br.completed_stages}")
    print(f"Units run:   {br.executed_units}")
    print(f"Reward:      {br.reward}")


if __name__ == "__main__":
    main()
