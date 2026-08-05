#!/usr/bin/env python3
"""Migrate all old BEHAVIOR_CATALOG entries to v2 config files.

Reads behavior_catalog.py, identifies which behaviors/actions are missing
from the v2 YAML configs, and appends them.

Usage:  PYTHONPATH=. python3 scripts/migrate_to_v2.py
"""
import os, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.chdir(str(ROOT))

import yaml

from marsdog_action_executor.behavior_catalog import BEHAVIOR_CATALOG, ALIAS_MAP
from marsdog_action_executor.config_loader import ConfigLoader

# ── Load v2 configs ────────────────────────────────────────────────────
loader = ConfigLoader('config')
loader.load_all()
v2_behaviors = set(loader.behavior_templates.keys())
v2_actions = set(loader.action_catalog.keys())

# ── All known alias names (both YAML aliases and code ALIAS_MAP) ───────
all_alias_names = set(ALIAS_MAP.keys()) | set(loader.behavior_aliases.keys())

# ── Emotion single-action behaviors (already mapped to express*) ───────
# These have aliases in behavior_aliases.yaml and resolve to express* behaviors
# No separate v2 template needed — the alias resolution handles them

# ── Old legacy names with underscores (aliases to v2) ──────────────────
# seek_food_or_water → seekFood, excretion_request → defecate, etc.
# These are in ALIAS_MAP and resolve automatically.

# ── Behaviors that TRULY need v2 templates ─────────────────────────────
needs_template = set()
for name in BEHAVIOR_CATALOG:
    if name in v2_behaviors:
        continue  # already has v2 template
    if name in all_alias_names:
        continue  # alias resolves to something else
    needs_template.add(name)

# ── Collect all old action IDs from these behaviors ───────────────────
old_actions: dict[str, dict] = {}
for name in sorted(needs_template):
    for phase in BEHAVIOR_CATALOG[name]:
        for entry in phase.get('actions', []):
            aid = entry if isinstance(entry, str) else entry.get('action_id', '')
            if not aid or aid in v2_actions or aid in old_actions:
                continue
            parts = aid.split('_')
            cat = parts[1] if len(parts) > 1 else 'MISC'
            ctrl_map = {
                'POSTURE': 'posture', 'LOCO': 'motion', 'HEAD': 'head',
                'TAIL': 'tail', 'MOUTH': 'mouth', 'PAW': 'paw',
                'VOCAL': 'audio', 'EAR': 'expression', 'NAV': 'nav',
                'LIGHT': 'meta',
            }
            # Duration heuristics
            timeout = 3.0
            if cat in ('POSTURE',):
                if any(w in aid for w in ('LIE', 'CURL', 'SPLAY', 'PLAY_DEAD', 'SIDE', 'BREATHING')):
                    timeout = 20.0
                elif 'STAND' in aid or 'STOP' in aid:
                    timeout = 2.0
                elif 'SIT' in aid:
                    timeout = 4.0
                elif 'ROLL' in aid:
                    timeout = 4.0
                elif 'FREEZE' in aid:
                    timeout = 5.0
            elif cat == 'LOCO':
                if 'ZOOMIES' in aid or 'FLEE' in aid or 'FOLLOW' in aid or 'HIDE' in aid:
                    timeout = 8.0
                elif 'WALK' in aid or 'APPROACH' in aid or 'TROT' in aid or 'PACE' in aid:
                    timeout = 5.0
                elif 'SPIN' in aid or 'HOP' in aid or 'POUNCE' in aid or 'JUMP' in aid:
                    timeout = 2.0
                elif 'BACK' in aid:
                    timeout = 3.0
            elif cat in ('MOUTH',):
                if 'CHEW' in aid or 'CARRY' in aid:
                    timeout = 6.0
                elif 'LICK' in aid or 'NIBBLE' in aid:
                    timeout = 4.0
                elif 'BITE' in aid:
                    timeout = 2.0
            interrupt = 'safe'
            if any(w in aid for w in ('ZOOMIES', 'FLEE', 'PLAY_DEAD', 'EAT')):
                interrupt = 'unsafe'

            old_actions[aid] = {
                'unit_id': aid,
                'unit_type': 'atomic_action',
                'controller': ctrl_map.get(cat, 'meta'),
                'timeout_sec': timeout,
                'interrupt_policy': interrupt,
                'from_postures': ['unknown'],
                'to_posture': '',
                'conditions': {},
                'description': f'Legacy action: {aid} ({cat.lower()})',
            }

print(f'New action IDs to add: {len(old_actions)}')
print(f'New behavior templates:  {len(needs_template)}')
print(f'Behaviors: {sorted(needs_template)}')

# ── Write action catalog additions ─────────────────────────────────────
action_path = ROOT / 'config' / 'action_catalog.yaml'
with open(action_path, 'a', encoding='utf-8') as f:
    f.write('\n')
    f.write('  # ╔══════════════════════════════════════════════════════════════════╗\n')
    f.write('  # ║  SECTION Z: Legacy migrated actions (auto-generated)           ║\n')
    f.write('  # ╚══════════════════════════════════════════════════════════════════╝\n')
    f.write('\n')
    for aid in sorted(old_actions):
        m = old_actions[aid]
        f.write(f'  {aid}:\n')
        f.write(f'    unit_id: {m["unit_id"]}\n')
        f.write(f'    unit_type: {m["unit_type"]}\n')
        f.write(f'    controller: {m["controller"]}\n')
        f.write(f'    timeout_sec: {m["timeout_sec"]}\n')
        f.write(f'    interrupt_policy: {m["interrupt_policy"]}\n')
        f.write(f'    from_postures: [unknown]\n')
        f.write(f'    to_posture: ""\n')
        f.write(f'    conditions: {{}}\n')
        f.write(f'    description: "{m["description"]}"\n')
        f.write('\n')

# ── Write behavior template additions ──────────────────────────────────
tpl_path = ROOT / 'config' / 'behavior_templates.yaml'
with open(tpl_path, 'a', encoding='utf-8') as f:
    f.write('\n')
    f.write('  # ╔════════════════════════════════════════════════════════════════════════════╗\n')
    f.write('  # ║  SECTION 6: Legacy migrated behaviors (auto-generated from old catalog)  ║\n')
    f.write('  # ╚════════════════════════════════════════════════════════════════════════════╝\n')
    f.write('\n')
    for name in sorted(needs_template):
        phases = BEHAVIOR_CATALOG[name]
        f.write(f'  # ── {name} (migrated, {len(phases)} stages) ─────────────────────────────\n')
        f.write(f'  {name}:\n')
        f.write(f'    behavior_name: {name}\n')
        f.write(f'    description: "Migrated from legacy behavior catalog."\n')
        f.write(f'    stages:\n')
        for i, phase in enumerate(phases):
            phase_name = phase.get('phase', f'stage_{i}')
            policy = phase.get('selection_policy', 'random')
            required = phase.get('required', True)
            actions = phase.get('actions', [])
            f.write(f'      - stage_name: {phase_name}\n')
            f.write(f'        stage_type: {phase_name.replace(" ", "_")[:20]}\n')
            f.write(f'        selection_policy: {policy}\n')
            f.write(f'        required: {str(required).lower()}\n')
            f.write(f'        skip_on_fail: false\n')
            f.write(f'        fixed: false\n')
            f.write(f'        candidates:\n')
            for entry in actions:
                aid = entry if isinstance(entry, str) else entry.get('action_id', '')
                if aid:
                    f.write(f'          - {{unit_id: {aid}, weight: 1.0}}\n')
            f.write(f'        conditions: {{}}\n')
            f.write(f'        success_condition:\n')
            f.write(f'          type: action_completed\n')
            f.write('\n')

# ── Add success conditions for new behaviors ───────────────────────────
result_path = ROOT / 'marsdog_action_executor' / 'result_evaluator.py'
with open(result_path, 'r') as f:
    result_code = f.read()

# Add new entries to BEHAVIOR_SUCCESS_CONDITIONS
for name in sorted(needs_template):
    cond_key = f'    "{name}": "all_required_stages_completed",'
    if cond_key not in result_code:
        # Insert after the last existing behavior entry
        marker = '"expressCuriosity": "at_least_one_expression",'
        if marker in result_code:
            result_code = result_code.replace(
                marker + '\n',
                marker + '\n' + cond_key + '\n',
            )

with open(result_path, 'w') as f:
    f.write(result_code)

# ── Add canonical behavior names to resolver ───────────────────────────
resolver_path = ROOT / 'marsdog_action_executor' / 'behavior_resolver.py'
with open(resolver_path, 'r') as f:
    resolver_code = f.read()

for name in sorted(needs_template):
    if f'"{name}"' not in resolver_code:
        # Add to _CANONICAL_BEHAVIORS set
        marker = '_CANONICAL_BEHAVIORS: set[str] = {'
        if marker in resolver_code:
            resolver_code = resolver_code.replace(
                marker,
                marker + f'\n    "{name}",',
            )

with open(resolver_path, 'w') as f:
    f.write(resolver_code)

print('Done. Files updated:')
print(f'  {action_path}')
print(f'  {tpl_path}')
print(f'  {result_path}')
print(f'  {resolver_path}')
