#!/usr/bin/env python3
"""One-shot migration: convert old BEHAVIOR_CATALOG + actions to v2 YAML.

Generates:
  1. New action_catalog entries for old action IDs
  2. New behavior_templates entries for missing behaviors

Run once, then delete behavior_catalog.py.
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from marsdog_action_executor.behavior_catalog import BEHAVIOR_CATALOG
from marsdog_action_executor.config_loader import ConfigLoader

# Load existing v2 configs
loader = ConfigLoader('config')
loader.load_all()
v2_behaviors = set(loader.behavior_templates.keys())
v2_actions = set(loader.action_catalog.keys())

# ── Categorize old action IDs ────────────────────────────────────────────
OLD_ACTION_META: dict[str, dict] = {}

# Collect all old action IDs and infer metadata
for name, phases in BEHAVIOR_CATALOG.items():
    # Skip aliases — they resolve to v2 behaviors
    if name in loader.behavior_aliases:
        continue
    for phase in phases:
        for entry in phase.get('actions', []):
            aid = entry if isinstance(entry, str) else entry.get('action_id', '')
            if not aid or aid in v2_actions or aid in OLD_ACTION_META:
                continue
            # Infer controller from category prefix
            parts = aid.split('_')
            cat = parts[1] if len(parts) > 1 else ''
            controller_map = {
                'POSTURE': 'posture', 'LOCO': 'motion',
                'HEAD': 'head', 'TAIL': 'tail',
                'MOUTH': 'mouth', 'PAW': 'paw',
                'VOCAL': 'audio', 'EAR': 'expression',
                'NAV': 'nav', 'LIGHT': 'meta',
            }
            OLD_ACTION_META[aid] = {
                'unit_id': aid,
                'unit_type': 'atomic_action',
                'controller': controller_map.get(cat, 'meta'),
                'timeout_sec': 3.0,
                'interrupt_policy': 'safe',
                'from_postures': ['unknown'],
                'to_posture': '',
                'conditions': {},
                'description': f'Legacy action: {aid}',
            }

# Category-specific overrides
_SLEEP_TIMEOUTS = {'ACT_POSTURE_LIE_FLAT', 'ACT_POSTURE_LIE_SIDE', 'ACT_POSTURE_CURL_UP',
                   'ACT_POSTURE_SPLAY_LEGS', 'ACT_POSTURE_GENTLE_BREATHING', 'ACT_POSTURE_PLAY_DEAD'}
_LOCO_TIMEOUTS = {'ACT_LOCO_ZOOMIES', 'ACT_LOCO_FOLLOW', 'ACT_LOCO_FLEE'}

for aid in OLD_ACTION_META:
    if aid in _SLEEP_TIMEOUTS:
        OLD_ACTION_META[aid]['timeout_sec'] = 20.0
    elif aid in _LOCO_TIMEOUTS:
        OLD_ACTION_META[aid]['timeout_sec'] = 8.0

print(f'Old action IDs to add: {len(OLD_ACTION_META)}')
print(f'Missing behaviors: {len([b for b in BEHAVIOR_CATALOG if b not in v2_behaviors and b not in loader.behavior_aliases])}')

# ── Print YAML for action_catalog (append to file) ───────────────────────
print('\n--- action_catalog.yaml additions ---')
for aid in sorted(OLD_ACTION_META):
    m = OLD_ACTION_META[aid]
    print(f"""  {aid}:
    unit_id: {m['unit_id']}
    unit_type: {m['unit_type']}
    controller: {m['controller']}
    timeout_sec: {m['timeout_sec']}
    interrupt_policy: {m['interrupt_policy']}
    from_postures: [{', '.join(m['from_postures'])}]
    to_posture: ""
    conditions: {{}}
    description: "{m['description']}" """)

# ── Print YAML for behavior_templates (new behaviors) ────────────────────
print('\n--- behavior_templates.yaml additions ---')
for name in sorted(BEHAVIOR_CATALOG):
    if name in v2_behaviors or name in loader.behavior_aliases:
        continue
    phases = BEHAVIOR_CATALOG[name]
    print(f'  {name}:')
    print(f'    behavior_name: {name}')
    print(f'    description: "Migrated from legacy catalog ({len(phases)} stages)"')
    print(f'    stages:')
    for i, phase in enumerate(phases):
        policy = phase.get('selection_policy', 'random')
        actions = phase.get('actions', [])
        candidates = []
        for a in actions:
            if isinstance(a, str):
                candidates.append(a)
            elif isinstance(a, dict):
                candidates.append(a.get('action_id', ''))
        print(f'      - stage_name: {phase.get("phase", f"stage_{i}")}')
        print(f'        stage_type: {phase.get("phase", f"stage_{i}").replace(" ", "_")[:20]}')
        print(f'        selection_policy: {policy}')
        print(f'        required: {phase.get("required", True)}')
        print(f'        skip_on_fail: false')
        print(f'        fixed: false')
        print(f'        candidates:')
        for c in candidates:
            print(f'          - {{unit_id: {c}, weight: 1.0}}')
        print(f'        conditions: {{}}')
        print(f'        success_condition:')
        print(f'          type: action_completed')
    print()

# Stats
missing = [b for b in BEHAVIOR_CATALOG if b not in v2_behaviors and b not in loader.behavior_aliases]
print(f'\n# {len(OLD_ACTION_META)} old action IDs + {len(missing)} behavior templates generated', file=sys.stderr)
