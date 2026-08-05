# MarsDog Action Executor — 行为与动作对照表

> 本文档列出所有 canonical behavior → stage → action unit 的完整映射关系。
> 配置来源：[behavior_templates.yaml](../config/behavior_templates.yaml) · [action_catalog.yaml](../config/action_catalog.yaml) · [emotion_action_pools.yaml](../config/emotion_action_pools.yaml)

---

## 目录

1. [Behavior 总览](#1-behavior-总览)
2. [生理需求行为](#2-生理需求行为)
3. [动物社交行为](#3-动物社交行为)
4. [人类社交行为](#4-人类社交行为)
5. [探索行为](#5-探索行为)
6. [情绪表达行为 (Emotion Pool)](#6-情绪表达行为-emotion-pool)
7. [Action Unit 完整索引](#7-action-unit-完整索引)
8. [Stages 与 Selection Policies 说明](#8-stages-与-selection-policies-说明)

---

## 1. Behavior 总览

| # | Behavior | 类别 | 成功条件 | Stage 数 | 执行单元总数 |
|---|----------|------|---------|---------|-------------|
| 1 | `eatNormally` | 生理需求 | eating_stage_completed | 4 | 22 |
| 2 | `eatExcitedly` | 生理需求 | eating_stage_completed | 3 | 17 |
| 3 | `defecate` | 生理需求 | eliminating_stage_completed | 3 | 17 |
| 4 | `cleanSelf` | 生理需求 | groom_stage_completed | 1 | 6 |
| 5 | `sleepNow` | 生理需求 | sleep_pose_entered | 4 | 28 |
| 6 | `restInPlace` | 生理需求 | recover_stage_completed | 1 | 6 |
| 7 | `recharge` | 生理需求 | charging_detected | 1 | 2 |
| 8 | `testAnimalBoundary` | 动物社交 | express_stage_completed | 3 | 10 |
| 9 | `greetAnimal` | 动物社交 | greet_stage_completed | 2 | 7 |
| 10 | `inviteAnimalToPlay` | 动物社交 | invite_stage_completed | 2 | 7 |
| 11 | `requestResourceFromHuman` | 人类社交 | request_stage_completed | 1 | 8 |
| 12 | `seekHumanInteraction` | 人类社交 | interact_stage_completed | 3 | 12 |
| 13 | `inviteHumanToPlay` | 人类社交 | invite_stage_completed | 2 | 9 |
| 14 | `exploreRoom` | 探索 | explore_stage_completed | 1 | 8 |
| 15 | `inspectObject` | 探索 | inspect_stage_completed | 2 | 10 |
| 16 | `inspectKnownObject` | 探索 | inspect_stage_completed | 1 | 32 |
| 17 | `expressCalm` | 情绪表达 | at_least_one_expression | 1 | 9 |
| 18 | `expressJoy` | 情绪表达 | at_least_one_expression | 1 | 14 |
| 19 | `expressExcitement` | 情绪表达 | at_least_one_expression | 1 | 13 |
| 20 | `expressAnxiety` | 情绪表达 | at_least_one_expression | 1 | 14 |
| 21 | `expressFear` | 情绪表达 | at_least_one_expression | 1 | 13 |
| 22 | `expressCuriosity` | 情绪表达 | at_least_one_expression | 1 | 13 |

---

## 2. 生理需求行为

### 2.1 eatNormally — 正常进食

**Pipeline**: `prepare → eating → interaction(optional,skipOnFail) → exit`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **prepare** | `random_one` | ✅ | `ACT_SNIFF_BOWL_EDGE` | atomic_action | head | 4.0s |
| | | | `ACT_PAW_AT_BOWL` | atomic_action | paw | 3.0s |
| | | | `ACT_SIT_OR_LIE_BY_BOWL` | composite_action | posture | 5.0s |
| | | | `ACT_SNIFF_GROUND_FOR_CRUMBS` | atomic_action | head | 5.0s |
| | | | `ACT_CHANGE_POSTURE` | atomic_action | posture | 3.0s |
| | | | `TASK_APPROACH_OBJECT` | task | motion | 8.0s |
| **eating** | `random_one` | ✅ | `ACT_LICK_FOOD` | atomic_action | mouth | 6.0s |
| | | | `ACT_CHEW_OR_CARRY_FOOD` | composite_action | mouth | 8.0s |
| | | | `ACT_SCRATCH_FOOD` | atomic_action | paw | 4.0s |
| | | | `ACT_FAST_LICK_AND_SWALLOW` | atomic_action | mouth | 4.0s |
| | | | `ACT_LICK_LIPS_AND_SWALLOW` | atomic_action | mouth | 2.0s |
| **interaction** | `random_one` | ❌ skip_on_fail | `ACT_PAUSE_AND_LOOK_AT_OWNER` | atomic_action | head | 3.5s |
| | | | `ACT_GROWL_WHILE_EATING` | atomic_action | audio | 6.0s |
| | | | `ACT_LICK_LIPS_OR_NOSE` | atomic_action | mouth | 1.5s |
| **exit** | `random_one` | ✅ | `ACT_LICK_LIPS_AND_SWALLOW` | atomic_action | mouth | 2.0s |
| | | | `ACT_TURN_HEAD_AND_WIPE_MOUTH` | composite_action | head | 3.0s |
| | | | `ACT_BURP` | atomic_action | mouth | 1.5s |
| | | | `ACT_LICK_LIPS_OR_NOSE` | atomic_action | mouth | 1.5s |
| | | | `ACT_SHAKE_HEAD` | atomic_action | head | 1.5s |
| | | | `ACT_WALK_AWAY_OR_LIE_DOWN` | composite_action | motion | 6.0s |

### 2.2 eatExcitedly — 兴奋进食

**Pipeline**: `prepare(fixed) → eating(fixed) → exit(fixed)`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **prepare** | `random_one` | ✅ fixed | `ACT_CHASE_ROLLING_FOOD` | task | motion | 10.0s |
| | | | `ACT_SNIFF_BOWL_EDGE` | atomic_action | head | 4.0s |
| | | | `ACT_PAW_AT_BOWL` | atomic_action | paw | 3.0s |
| | | | `TASK_APPROACH_OBJECT` | task | motion | 8.0s |
| **eating** | `random_one` | ✅ fixed | `ACT_FAST_LICK_AND_SWALLOW` | atomic_action | mouth | 4.0s |
| | | | `ACT_CHEW_OR_CARRY_FOOD` | composite_action | mouth | 8.0s |
| | | | `ACT_LICK_FOOD` | atomic_action | mouth | 6.0s |
| | | | `ACT_GROWL_WHILE_EATING` | atomic_action | audio | 6.0s |
| | | | `ACT_SCRATCH_FOOD` | atomic_action | paw | 4.0s |
| **exit** | `random_one` | ✅ fixed | `ACT_LICK_LIPS_AND_SWALLOW` | atomic_action | mouth | 2.0s |
| | | | `ACT_BURP` | atomic_action | mouth | 1.5s |
| | | | `ACT_SHAKE_HEAD` | atomic_action | head | 1.5s |
| | | | `ACT_TURN_HEAD_AND_WIPE_MOUTH` | composite_action | head | 3.0s |

### 2.3 defecate — 排泄

**Pipeline**: `prepare → eliminating → exit`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **prepare** | `random_one` | ✅ | `ACT_SNIFF_AND_CIRCLE` | composite_action | motion | 8.0s |
| | | | `ACT_SCRATCH_GROUND` | atomic_action | paw | 4.0s |
| | | | `ACT_HESITATE` | atomic_action | posture | 3.0s |
| | | | `ACT_LOWER_HEAD_OR_TURN` | atomic_action | head | 4.0s |
| **eliminating** | `random_one` | ✅ | `ACT_SQUAT_TO_PEE` | atomic_action | posture | 4.0s |
| | | | `ACT_SQUAT_TO_POOP` | atomic_action | posture | 5.0s |
| | | | `ACT_TENSE_BODY` | atomic_action | posture | 4.0s |
| | | | `ACT_SLIGHT_TREMOR` | atomic_action | motion | 3.0s |
| | | | `ACT_TAIL_MOVEMENT` | atomic_action | tail | 3.0s |
| **exit** | `random_one` | ✅ | `ACT_STAND_UP_WITH_HIND_LEGS` | atomic_action | posture | 2.5s |
| | | | `ACT_SCRATCH_SOIL_OR_GROUND` | atomic_action | paw | 4.0s |
| | | | `ACT_SNIFF_EXCREMENT` | atomic_action | head | 3.0s |
| | | | `ACT_WALK_AWAY_OR_SHAKE_HEAD` | composite_action | motion | 4.0s |

### 2.4 cleanSelf — 自我清洁

**Pipeline**: `groom (single stage)`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **groom** | `random_one` | ✅ | `ACT_LICK_PAWS_OR_FUR` | atomic_action | mouth | 6.0s |
| | | | `ACT_SCRATCH` | atomic_action | paw | 4.0s |
| | | | `ACT_SHAKE_OFF_WATER` | atomic_action | motion | 2.0s |
| | | | `ACT_STRETCH_LAZILY` | atomic_action | posture | 4.0s |
| | | | `ACT_ROLL_OVER` | composite_action | posture | 4.0s |
| | | | `ACT_RUB_AGAINST_OBJECT` | atomic_action | motion | 5.0s |

### 2.5 sleepNow — 入睡

**Pipeline**: `prepare → sleep_pose → sleeping(loop: 0-3 shallow / 0-2 deep) → wakeup`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **prepare** | `random_one` | ✅ | `ACT_CIRCLE_AROUND` | composite_action | motion | 5.0s |
| | | | `ACT_SCRATCH_BED_OR_GROUND` | atomic_action | paw | 5.0s |
| | | | `ACT_LIE_ON_SIDE_AND_STRETCH` | composite_action | posture | 5.0s |
| | | | `ACT_LICK_FUR_OR_PAWS` | atomic_action | mouth | 5.0s |
| **sleep_pose** | `random_one` | ✅ | `ACT_SLEEP_CURLED_UP` | atomic_action | posture | 20.0s |
| | | | `ACT_SLEEP_ON_STOMACH` | atomic_action | posture | 20.0s |
| | | | `ACT_SLEEP_ON_SIDE_CURLED_UP` | atomic_action | posture | 20.0s |
| | | | `ACT_SLEEP_ON_SIDE` | atomic_action | posture | 20.0s |
| | | | `ACT_SLEEP_ON_BACK` | atomic_action | posture | 20.0s |
| **sleeping** | `random_one` | ✅ | *shallow:* `ACT_FLIP_BODY` / `ACT_WHINE_SOFTLY` / `ACT_TWITCH_OR_KICK_LEGS` / `ACT_SHAKE_HEAD_OR_SMACK_LIPS` / `ACT_WAG_TAIL` | — | — | — |
| | | | *deep:* `ACT_SLEEP_ON_SIDE` / `ACT_SLEEP_CURLED_UP` / `ACT_SLEEP_ON_BACK` / `ACT_SLEEP_ON_STOMACH` | — | — | — |
| **wakeup** | `random_one` | ✅ | `ACT_YAWN` | atomic_action | mouth | 3.0s |
| | | | `ACT_GETUP_CRAWL` | composite_action | motion | 5.0s |
| | | | `ACT_GETUP_ROLL` | composite_action | motion | 4.0s |
| | | | `ACT_GETUP_BOUNCE` | composite_action | motion | 3.0s |
| | | | `ACT_GETUP_STRETCH` | composite_action | posture | 6.0s |
| | | | `ACT_GETUP_SIT` | composite_action | posture | 3.0s |
| | | | `ACT_SHAKE_HEAD` | atomic_action | head | 1.5s |

### 2.6 restInPlace — 原地休息

**Pipeline**: `recover (single stage)`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **recover** | `random_one` | ✅ | `ACT_PANT_IN_PLACE` | atomic_action | mouth | 6.0s |
| | | | `ACT_SLOW_MOVEMENT` | **modifier** | meta | 0.0s |
| | | | `ACT_LIE_ON_SIDE_AND_STRETCH` | composite_action | posture | 5.0s |
| | | | `ACT_YAWN` | atomic_action | mouth | 3.0s |
| | | | `ACT_WAG_TAIL` | atomic_action | tail | 3.0s |
| | | | `ACT_LIE_BY_DOOR` | atomic_action | posture | 10.0s |

### 2.7 recharge — 充电

**Pipeline**: `recharge (condition_first)`

| Stage | Policy | Required | Condition | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-----------|-------------|------|-----------|---------|
| **recharge** | `condition_first` | ✅ | `charger_location_known` + `battery_low` | `ACT_RETURN_TO_CHARGER` | task | nav | 30.0s |
| | | | `charger_location_unknown` + `battery_low` | `ACT_BARK_AND_LIE_DOWN_IF_NO_CHARGER` | composite_action | vocal | 10.0s |

---

## 3. 动物社交行为

### 3.1 testAnimalBoundary — 测试动物边界

**Pipeline**: `orient(fixed) → express → observe(optional,skipOnFail,fixed)`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **orient** | `random_one` | ✅ fixed | `TASK_ORIENT_TO_ANIMAL` | task | motion | 6.0s |
| | | | `ACT_STARE_AND_TILT_HEAD` | composite_action | head | 4.0s |
| | | | `ACT_APPROACH_SLOWLY_SIDEWAYS` | atomic_action | motion | 6.0s |
| **express** | `random_one` | ✅ | `ACT_LOWER_HEAD_FLOP_EARS_WAG_TAIL` | composite_action | head | 4.0s |
| | | | `ACT_BARK_OR_HOWL` | atomic_action | audio | 3.0s |
| | | | `ACT_SNIFF_FACE_OR_EARS` | atomic_action | head | 4.0s |
| | | | `ACT_SNIFF_BUTT_OR_TAIL` | atomic_action | head | 3.0s |
| **observe** | `random_one` | ❌ skip_on_fail | `TASK_OBSERVE_ANIMAL_RESPONSE` | task | head | 6.0s |
| | | | `ACT_STARE_AND_TILT_HEAD` | composite_action | head | 4.0s |

### 3.2 greetAnimal — 动物问候

**Pipeline**: `approach(fixed) → greet`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **approach** | `random_one` | ✅ fixed | `TASK_APPROACH_ANIMAL_SAFELY` | task | motion | 10.0s |
| | | | `ACT_APPROACH_SLOWLY_SIDEWAYS` | atomic_action | motion | 6.0s |
| **greet** | `random_one` | ✅ | `ACT_SNIFF_FACE_OR_EARS` | atomic_action | head | 4.0s |
| | | | `ACT_SNIFF_BUTT_OR_TAIL` | atomic_action | head | 3.0s |
| | | | `ACT_TOUCH_NOSE_OR_HEAD_GENTLY` | atomic_action | head | 2.0s |
| | | | `ACT_LOWER_HEAD_FLOP_EARS_WAG_TAIL` | composite_action | head | 4.0s |
| | | | `ACT_LIE_BESIDE_OTHER_ANIMAL` | atomic_action | posture | 8.0s |

### 3.3 inviteAnimalToPlay — 邀请动物玩耍

**Pipeline**: `orient(fixed) → invite`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **orient** | `random_one` | ✅ fixed | `TASK_ORIENT_TO_ANIMAL` | task | motion | 6.0s |
| | | | `ACT_APPROACH_SLOWLY_SIDEWAYS` | atomic_action | motion | 6.0s |
| **invite** | `random_one` | ✅ | `ACT_PLAY_BOW` | atomic_action | posture | 3.0s |
| | | | `ACT_POUNCE_GENTLY` | atomic_action | motion | 2.0s |
| | | | `ACT_RUN_IN_CIRCLES_OR_CHASE` | composite_action | motion | 8.0s |
| | | | `ACT_PAW_GENTLY_AT_OTHER` | atomic_action | paw | 2.0s |
| | | | `ACT_CARRY_AND_SHAKE_OBJECT` | composite_action | mouth | 5.0s |

---

## 4. 人类社交行为

### 4.1 requestResourceFromHuman — 向人类请求资源

**Pipeline**: `request (single stage)`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **request** | `random_one` | ✅ | `ACT_STARE_AT_FOOD_IN_HAND` | atomic_action | head | 5.0s |
| | | | `ACT_SCRATCH_FOOD_BOWL_OR_SNACK_CABINET` | atomic_action | paw | 5.0s |
| | | | `ACT_CARRY_LEASH_OR_SHOE` | atomic_action | mouth | 5.0s |
| | | | `ACT_PRESS_BELL_OR_TRIGGER_MECHANISM` | atomic_action | paw | 3.0s |
| | | | `ACT_WHINE_OR_BARK_SOFTLY` | atomic_action | audio | 2.0s |
| | | | `ACT_PAW_AT_ARM_OR_PANTS` | atomic_action | paw | 2.0s |
| | | | `ACT_NUDGE_HAND_WITH_HEAD` | atomic_action | head | 2.0s |
| | | | `ACT_GUARD_FOOD_BOWL_OR_SNACK_CABINET` | composite_action | posture | 12.0s |

### 4.2 seekHumanInteraction — 寻求人类互动

**Pipeline**: `search(optional,condition_first) → approach(optional,condition_first) → interact`

| Stage | Policy | Required | Condition | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-----------|-------------|------|-----------|---------|
| **search** | `condition_first` | ❌ skip_on_fail | `person_visible: false` | `ACT_SEARCH_FOR_PERSON` | task | nav | 20.0s |
| | | | `person_visible: true` | `ACT_FOLLOW_AND_CLING` | task | motion | 15.0s |
| **approach** | `condition_first` | ❌ skip_on_fail | `person_detected` + `distance_gt: 0.5` | `TASK_APPROACH_PERSON` | task | motion | 10.0s |
| | | | `person_nearby: true` | `ACT_RUB_AGAINST_LEG_OR_LEAN` | atomic_action | motion | 4.0s |
| **interact** | `random_one` | ✅ | — | `ACT_NUDGE_HAND_WITH_HEAD` | atomic_action | head | 2.0s |
| | | | | `ACT_SIT_OR_LIE_AT_FEET` | composite_action | posture | 5.0s |
| | | | | `ACT_LICK_HAND_OR_FACE` | atomic_action | mouth | 4.0s |
| | | | | `ACT_RUB_AGAINST_LEG_OR_LEAN` | atomic_action | motion | 4.0s |
| | | | | `ACT_WHINE_OR_BARK_SOFTLY` | atomic_action | audio | 2.0s |
| | | | | `ACT_PAW_AT_ARM_OR_PANTS` | atomic_action | paw | 2.0s |
| | | | | `ACT_JUMP_ON_PERSON` | atomic_action | motion | 2.0s |
| | | | | `ACT_FOLLOW_AND_CLING` | task | motion | 15.0s |

### 4.3 inviteHumanToPlay — 邀请人类玩耍

**Pipeline**: `approach(optional,fixed) → invite`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **approach** | `random_one` | ❌ skip_on_fail | `TASK_APPROACH_PERSON` | task | motion | 10.0s |
| | | | `ACT_FOLLOW_AND_CLING` | task | motion | 15.0s |
| **invite** | `random_one` | ✅ | `ACT_PLAY_BOW_INVITE` | atomic_action | posture | 3.0s |
| | | | `ACT_DROP_TOY_IN_FRONT_OF_OWNER` | atomic_action | mouth | 3.0s |
| | | | `ACT_SHAKE_TOY_WITH_MOUTH` | atomic_action | head | 4.0s |
| | | | `ACT_RUN_IN_CIRCLES_OR_ZOOMIES` | composite_action | motion | 8.0s |
| | | | `ACT_NIP_GENTLY_AT_PANTS_OR_HAND` | atomic_action | mouth | 1.5s |
| | | | `ACT_PLACE_PAW_ON_KNEE` | atomic_action | paw | 2.0s |
| | | | `ACT_PAW_AT_ARM_OR_PANTS` | atomic_action | paw | 2.0s |

---

## 5. 探索行为

### 5.1 exploreRoom — 探索房间

**Pipeline**: `explore (single stage)`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **explore** | `random_one` | ✅ | `ACT_TROT_AND_LOOK_AROUND` | composite_action | motion | 8.0s |
| | | | `ACT_WALK_SLOWLY_AND_SNIFF_GROUND` | composite_action | motion | 10.0s |
| | | | `ACT_CRAWL_THROUGH_LOW_GAP` | composite_action | motion | 6.0s |
| | | | `ACT_STAND_AND_SCRATCH_HIGH` | atomic_action | paw | 4.0s |
| | | | `ACT_SCRATCH_DOOR_OR_FENCE` | atomic_action | paw | 5.0s |
| | | | `ACT_FIND_COOL_SPOT_AND_LIE_DOWN` | task | nav | 15.0s |
| | | | `ACT_PANT_IN_PLACE` | atomic_action | mouth | 6.0s |
| | | | `ACT_SLOW_MOVEMENT` | **modifier** | meta | 0.0s |

### 5.2 inspectObject — 检查物体

**Pipeline**: `approach(optional,fixed) → inspect`

| Stage | Policy | Required | Action Unit | Type | Controller | Timeout |
|-------|--------|----------|-------------|------|-----------|---------|
| **approach** | `random_one` | ❌ skip_on_fail | `TASK_APPROACH_OBJECT` | task | motion | 8.0s |
| **inspect** | `random_one` | ✅ | `ACT_SNIFF_OBJECT` | atomic_action | head | 4.0s |
| | | | `ACT_PUSH_OBJECT_WITH_PAW` | atomic_action | paw | 3.0s |
| | | | `ACT_SCRATCH_OBJECT_GENTLY` | atomic_action | paw | 3.0s |
| | | | `ACT_TOUCH_OR_CARRY_OBJECT_WITH_MOUTH` | composite_action | mouth | 5.0s |
| | | | `ACT_BARK_AT_OBJECT` | atomic_action | vocal | 3.0s |
| | | | `ACT_NIBBLE_OBJECT` | atomic_action | mouth | 4.0s |
| | | | `ACT_KNOCK_OVER_OBJECT` | atomic_action | paw | 2.0s |
| | | | `ACT_CARRY_AND_DROP_OBJECT_AGAIN` | composite_action | mouth | 6.0s |
| | | | `ACT_CARRY_AND_HIDE_OBJECT` | composite_action | mouth | 10.0s |

### 5.3 inspectKnownObject — 检查已知类别物体

**Pipeline**: `inspect (single stage, candidate pool determined by object_category param)`

| Category | Action Unit | Type | Controller | Timeout |
|----------|-------------|------|-----------|---------|
| **slippers_socks** | `ACT_SNIFF_SLIPPERS_OR_SOCKS` | atomic_action | head | 3.0s |
| | `ACT_BITE_SLIPPERS_OR_SOCKS` | atomic_action | mouth | 4.0s |
| | `ACT_POUNCE_ON_SLIPPERS_OR_SOCKS` | atomic_action | motion | 2.0s |
| | `ACT_CARRY_SLIPPERS_OR_SOCKS_TO_PERSON` | task | mouth | 12.0s |
| | `ACT_SCRATCH_SLIPPERS_OR_SOCKS_WITH_PAW` | atomic_action | paw | 3.0s |
| | `ACT_IGNORE_SLIPPERS_OR_SOCKS` | **policy** | meta | 0.0s |
| **trash_can** | `ACT_SNIFF_TRASH_CAN` | atomic_action | head | 3.0s |
| | `ACT_RUMMAGE_THROUGH_TRASH_CAN` | task | motion | 15.0s |
| | `ACT_IGNORE_TRASH_CAN` | **policy** | meta | 0.0s |
| **delivery_box** | `ACT_SNIFF_DELIVERY_BOX` | atomic_action | head | 4.0s |
| | `ACT_BITE_DELIVERY_BOX` | atomic_action | mouth | 4.0s |
| | `ACT_SCRATCH_DELIVERY_BOX_WITH_PAW` | atomic_action | paw | 3.0s |
| | `ACT_CARRY_DELIVERY_BOX_TO_PERSON` | task | mouth | 12.0s |
| | `ACT_IGNORE_DELIVERY_BOX` | **policy** | meta | 0.0s |
| **tissue** | `ACT_SNIFF_TISSUE` | atomic_action | head | 2.0s |
| | `ACT_SCRATCH_TISSUE_WITH_PAW` | atomic_action | paw | 2.0s |
| | `ACT_CARRY_TISSUE_TO_PERSON` | task | mouth | 10.0s |
| | `ACT_IGNORE_TISSUE` | **policy** | meta | 0.0s |
| **door** | `ACT_SNIFF_AROUND_DOOR` | atomic_action | head | 4.0s |
| | `ACT_LEAN_AGAINST_DOOR` | atomic_action | motion | 4.0s |
| | `ACT_LIE_BY_DOOR` | atomic_action | posture | 10.0s |
| | `ACT_SCRATCH_DOOR` | atomic_action | paw | 4.0s |
| | `ACT_IGNORE_DOOR` | **policy** | meta | 0.0s |
| **food_bowl** | `ACT_SNIFF_BOWL_EDGE` | atomic_action | head | 4.0s |
| | `ACT_PAW_AT_BOWL` | atomic_action | paw | 3.0s |
| | `ACT_SIT_OR_LIE_BY_BOWL` | composite_action | posture | 5.0s |
| | `ACT_SCRATCH_FOOD_BOWL_OR_SNACK_CABINET` | atomic_action | paw | 5.0s |
| | `ACT_GUARD_FOOD_BOWL_OR_SNACK_CABINET` | composite_action | posture | 12.0s |
| **toy** | `ACT_SNIFF_OBJECT` | atomic_action | head | 4.0s |
| | `ACT_TOUCH_OR_CARRY_OBJECT_WITH_MOUTH` | composite_action | mouth | 5.0s |
| | `ACT_CARRY_AND_SHAKE_OBJECT` | composite_action | mouth | 5.0s |
| | `ACT_CARRY_AND_DROP_OBJECT_AGAIN` | composite_action | mouth | 6.0s |
| | `ACT_CARRY_AND_HIDE_OBJECT` | composite_action | mouth | 10.0s |
| | `ACT_PUSH_OBJECT_WITH_PAW` | atomic_action | paw | 3.0s |
| **generic** (fallback) | `ACT_SNIFF_OBJECT` | atomic_action | head | 4.0s |
| | `ACT_PUSH_OBJECT_WITH_PAW` | atomic_action | paw | 3.0s |
| | `ACT_SCRATCH_OBJECT_GENTLY` | atomic_action | paw | 3.0s |
| | `ACT_TOUCH_OR_CARRY_OBJECT_WITH_MOUTH` | composite_action | mouth | 5.0s |
| | `ACT_NIBBLE_OBJECT` | atomic_action | mouth | 4.0s |
| | `ACT_BARK_AT_OBJECT` | atomic_action | vocal | 3.0s |

---

## 6. 情绪表达行为 (Emotion Pool)

6 个情绪表达 Behavior 使用 **动态候选池**——候选动作从 [emotion_action_pools.yaml](../config/emotion_action_pools.yaml) 按 `emotion + level + interaction_mode` 三维度查找。每个行为只有 **一个 express stage**（`random_one`）。

### 6.1 expressCalm — 表达平静

| Level | Mode | Action Unit | Type |
|-------|------|-------------|------|
| **LOW** | solo | `ACT_LIE_FLAT_ON_BELLY` | atomic_action |
| | | `ACT_LIE_ON_SIDE` | atomic_action |
| | | `ACT_STRETCH_LAZILY` | atomic_action |
| | | `ACT_LICK_PAWS` | atomic_action |
| | | `ACT_YAWN_SLOWLY` | atomic_action |
| LOW | interactive | `ACT_LIE_BELLY_UP` | atomic_action |
| | | `ACT_WAIT_BY_DOOR` | task |
| **HIGH** | solo | `ACT_PATROL_SURROUNDINGS_SLOWLY` | task |
| HIGH | interactive | `ACT_LIE_ON_SIDE_BESIDE_PERSON` | task |
| | | `ACT_PAW_AT_OWNER` | atomic_action |

### 6.2 expressJoy — 表达快乐

| Level | Mode | Action Unit | Type |
|-------|------|-------------|------|
| **LOW** | solo | `ACT_SIT_AND_WAG_TAIL_GENTLY` | composite_action |
| | | `ACT_STRETCH_BODY_RELAXEDLY` | atomic_action |
| LOW | interactive | `ACT_WAG_TAIL_GENTLY` | atomic_action |
| **MID** | solo | `ACT_STEP_EXCITEDLY_IN_PLACE` | atomic_action |
| | | `ACT_SWAY_BODY_WITH_WAGGING_TAIL` | composite_action |
| MID | interactive | `ACT_WAG_TAIL_FAST_AND_STEP_IN_PLACE` | composite_action |
| | | `ACT_HOP_IN_PLACE_WITH_FRONT_PAWS_UP` | atomic_action |
| | | `ACT_NUDGE_OWNER_HAND_OR_LEG_WITH_NOSE` | atomic_action |
| | | `ACT_PLAY_BOW_INVITE` | atomic_action |
| **HIGH** | solo | `ACT_RUN_BACK_AND_FORTH` | task |
| | | `ACT_SPIN_IN_CIRCLE` | atomic_action |
| HIGH | interactive | `ACT_RAISE_TAIL_AND_WAG` | composite_action |
| | | `ACT_POUNCE_FORWARD` | atomic_action |
| | | `ACT_BARK_SHORT_EXCITED` | atomic_action |
| | | `ACT_ROLL_OVER_AND_SHOW_BELLY` | composite_action |

### 6.3 expressExcitement — 表达兴奋

| Level | Mode | Action Unit | Type |
|-------|------|-------------|------|
| **LOW** | solo | `ACT_BEG_FOR_FOOD` | composite_action |
| LOW | interactive | `ACT_CARRY_AND_SHAKE` | task |
| | | `ACT_WIGGLE_BODY` | atomic_action |
| | | `ACT_TROT_AND_BOUNCE` | composite_action |
| **HIGH** | solo | `ACT_MOVE_RAPIDLY` | task |
| | | `ACT_CIRCLE_AROUND` | composite_action |
| | | `ACT_RUN_ZOOMIES` | task |
| HIGH | interactive | `ACT_JUMP_ON_PERSON` | atomic_action |
| | | `ACT_BARK_OR_WHINE` | atomic_action |
| | | `ACT_MOUTH_GENTLY` | atomic_action |
| | | `ACT_PLACE_PAW_ON_KNEE` | atomic_action |
| | | `ACT_FETCH_TOY` | task |

### 6.4 expressAnxiety — 表达焦虑

| Level | Mode | Action Unit | Type |
|-------|------|-------------|------|
| **LOW** | solo | `ACT_PACE_BACK_AND_FORTH` | composite_action |
| | | `ACT_LICK_LIPS` | atomic_action |
| LOW | interactive | `ACT_WHINE_LOW` | atomic_action |
| | | `ACT_STARE_INTENTLY` | atomic_action |
| | | `ACT_TILT_HEAD` | atomic_action |
| **HIGH** | solo | `ACT_SCRATCH_FREQUENTLY` | atomic_action |
| | | `ACT_STIFFEN_BODY` | atomic_action |
| | | `ACT_TUCK_TAIL` | atomic_action |
| | | `ACT_DILATE_PUPILS` | atomic_action |
| | | `ACT_HIDE_AWAY` | task |
| HIGH | interactive | `ACT_WHINE_HIGH` | atomic_action |
| | | `ACT_CHECK_OWNER` | atomic_action |

### 6.5 expressFear — 表达恐惧

| Level | Mode | Action Unit | Type |
|-------|------|-------------|------|
| **LOW** | solo | `ACT_FREEZE_ALERT` | atomic_action |
| | | `ACT_WALK_AWAY` | atomic_action |
| | | `ACT_RETREAT_WITH_TAIL_TUCKED` | composite_action |
| LOW | interactive | `ACT_GROWL_LOW` | atomic_action |
| **HIGH** | solo | `ACT_FLEE_QUICKLY` | task |
| | | `ACT_AVOID_AND_HIDE` | task |
| | | `ACT_TREMBLE_AND_SHAKE` | atomic_action |
| | | `ACT_LOSE_CONTROL` | atomic_action |
| | | `ACT_DILATE_PUPILS` | atomic_action |
| HIGH | interactive | `ACT_APPROACH_OWNER_FEARFULLY` | task |
| | | `ACT_CALL_FOR_HELP` | atomic_action |

### 6.6 expressCuriosity — 表达好奇

| Level | Mode | Action Unit | Type |
|-------|------|-------------|------|
| **LOW** | solo | `ACT_OBSERVE_QUIETLY` | atomic_action |
| | | `ACT_LOWER_BODY_AND_EXPLORE` | composite_action |
| LOW | interactive | `ACT_HOLD_TAIL_LEVEL_AND_WAG` | composite_action |
| | | `ACT_TILT_HEAD` | atomic_action |
| | | `ACT_STARE_INTENTLY` | atomic_action |
| **HIGH** | solo | `ACT_STAND_AND_OBSERVE` | atomic_action |
| | | `ACT_STAND_ALERT` | atomic_action |
| HIGH | interactive | `ACT_APPROACH_SLOWLY` | atomic_action |
| | | `ACT_SNIFF_GROUND` | atomic_action |
| | | `ACT_PAW_AT_OBJECT` | atomic_action |
| | | `ACT_FOLLOW_MOVEMENT` | task |
| | | `ACT_CIRCLE_AND_INSPECT` | task |

---

## 7. Action Unit 完整索引

### 7.1 按单元类型分组

#### Atomic Action (117 个) — 短时确定动作

| Action Unit | Controller | Timeout | Description |
|-------------|-----------|---------|-------------|
| `ACT_SNIFF_BOWL_EDGE` | head | 4.0s | Sniff the edge/rim of food or water bowl |
| `ACT_PAW_AT_BOWL` | paw | 3.0s | Tap or nudge the food bowl with front paw |
| `ACT_LICK_FOOD` | mouth | 6.0s | Lick food or water from bowl repeatedly |
| `ACT_SCRATCH_FOOD` | paw | 4.0s | Scratch at food on the ground to break it apart |
| `ACT_LICK_LIPS_AND_SWALLOW` | mouth | 2.0s | Lick around mouth and swallow after eating |
| `ACT_PAUSE_AND_LOOK_AT_OWNER` | head | 3.5s | Gaze at owner for approval or interaction |
| `ACT_CHANGE_POSTURE` | posture | 3.0s | Transition between postures (stand/sit/lie) |
| `ACT_GROWL_WHILE_EATING` | audio | 6.0s | Low rumble growl — food guarding signal |
| `ACT_BURP` | mouth | 1.5s | Small burp after eating/drinking quickly |
| `ACT_LICK_LIPS_OR_NOSE` | mouth | 1.5s | Quick lick — self-soothing or post-eating |
| `ACT_SHAKE_HEAD` | head | 1.5s | Shake head side-to-side rapidly |
| `ACT_SNIFF_GROUND_FOR_CRUMBS` | head | 5.0s | Systematic sniff for dropped food crumbs |
| `ACT_FAST_LICK_AND_SWALLOW` | mouth | 4.0s | Rapid lick and swallow — urgent eating |
| `ACT_SCRATCH_GROUND` | paw | 4.0s | Scratch ground before elimination |
| `ACT_SQUAT_TO_PEE` | posture | 4.0s | Lower into squatting posture to urinate |
| `ACT_SQUAT_TO_POOP` | posture | 5.0s | Lower into deeper squat to defecate |
| `ACT_HESITATE` | posture | 3.0s | Pause with body tension before committing |
| `ACT_TENSE_BODY` | posture | 4.0s | Tense core/leg muscles during elimination |
| `ACT_TAIL_MOVEMENT` | tail | 3.0s | Lift/adjust tail during elimination |
| `ACT_SLIGHT_TREMOR` | motion | 3.0s | Subtle tremor to hindquarters |
| `ACT_LOWER_HEAD_OR_TURN` | head | 4.0s | Lower/turn head to check surroundings |
| `ACT_STAND_UP_WITH_HIND_LEGS` | posture | 2.5s | Push up from squat to standing |
| `ACT_SCRATCH_SOIL_OR_GROUND` | paw | 4.0s | Territorial ground scratch after elimination |
| `ACT_SNIFF_EXCREMENT` | head | 3.0s | Lower nose to inspect elimination result |
| `ACT_LICK_PAWS_OR_FUR` | mouth | 6.0s | Extend tongue to lick paws/fur — cleaning |
| `ACT_SCRATCH` | paw | 4.0s | Hind paw scratch behind ear or body |
| `ACT_SHAKE_OFF_WATER` | motion | 2.0s | Vigorous full-body shake to fling off water |
| `ACT_STRETCH_LAZILY` | posture | 4.0s | Lazy stretch — front legs forward, back arched |
| `ACT_RUB_AGAINST_OBJECT` | motion | 5.0s | Press body flank against surface and rub |
| `ACT_SCRATCH_BED_OR_GROUND` | paw | 5.0s | Fluff/prepare sleeping surface |
| `ACT_LICK_FUR_OR_PAWS` | mouth | 5.0s | Pre-sleep grooming routine |
| `ACT_SLEEP_CURLED_UP` | posture | 20.0s | Tight curl, tail wrapped around nose |
| `ACT_SLEEP_ON_STOMACH` | posture | 20.0s | Flat on stomach, legs tucked |
| `ACT_SLEEP_ON_SIDE_CURLED_UP` | posture | 20.0s | On side, legs partially curled |
| `ACT_FLIP_BODY` | posture | 3.0s | Flip from one side to the other |
| `ACT_WHINE_SOFTLY` | vocal | 2.0s | Soft low whine — dreaming or discomfort |
| `ACT_TWITCH_OR_KICK_LEGS` | motion | 3.0s | Gentle twitch or kick — dreaming motor activity |
| `ACT_WAG_TAIL` | tail | 3.0s | Tail wag — occurs even during light sleep |
| `ACT_YAWN` | mouth | 3.0s | Wide-mouth yawn — waking or transitioning |
| `ACT_SLEEP_ON_SIDE` | posture | 20.0s | Fully on side, legs extended — deep sleep |
| `ACT_SLEEP_ON_BACK` | posture | 20.0s | On back, belly exposed — total trust |
| `ACT_PANT_IN_PLACE` | mouth | 6.0s | Heavy panting in place — cooling behavior |
| `ACT_RESIST_WALKING` | motion | 8.0s | Brace legs — stubborn refusal to walk |
| `ACT_BARK_OR_HOWL` | audio | 3.0s | Bark or howl — communication or alert |
| `ACT_SNIFF_FACE_OR_EARS` | head | 4.0s | Sniff face/ears — standard canine greeting |
| `ACT_SNIFF_BUTT_OR_TAIL` | head | 3.0s | Sniff rear/tail — information gathering |
| `ACT_TOUCH_NOSE_OR_HEAD_GENTLY` | head | 2.0s | Gentle nose/head touch — friendly greeting |
| `ACT_APPROACH_SLOWLY_SIDEWAYS` | motion | 6.0s | Sideways approach — de-escalation body language |
| `ACT_LIE_BESIDE_OTHER_ANIMAL` | posture | 8.0s | Lie beside another — trust and shared space |
| `ACT_PLAY_BOW` | posture | 3.0s | Classic play bow — front down, rear up |
| `ACT_POUNCE_GENTLY` | motion | 2.0s | Soft pounce near playmate — gentle invitation |
| `ACT_PAW_GENTLY_AT_OTHER` | paw | 2.0s | Gently tap another animal — playful invitation |
| `ACT_STARE_AT_FOOD_IN_HAND` | head | 5.0s | Intense begging stare at food in hand |
| `ACT_SCRATCH_FOOD_BOWL_OR_SNACK_CABINET` | paw | 5.0s | Scratch bowl/cabinet — demand behavior |
| `ACT_CARRY_LEASH_OR_SHOE` | mouth | 5.0s | Carry leash/shoe — "let's go for a walk" signal |
| `ACT_PRESS_BELL_OR_TRIGGER_MECHANISM` | paw | 3.0s | Press bell/button — trained communication |
| `ACT_WHINE_OR_BARK_SOFTLY` | audio | 2.0s | Soft whine/bark — get attention |
| `ACT_PAW_AT_ARM_OR_PANTS` | paw | 2.0s | Paw at person's arm/pants — attention seeking |
| `ACT_NUDGE_HAND_WITH_HEAD` | head | 2.0s | Push head under hand — soliciting pets |
| `ACT_JUMP_ON_PERSON` | motion | 2.0s | Spring up and place paws on person |
| `ACT_LICK_HAND_OR_FACE` | mouth | 4.0s | Lick person's hand/face — affectionate grooming |
| `ACT_RUB_AGAINST_LEG_OR_LEAN` | motion | 4.0s | Press body against leg — affectionate contact |
| `ACT_PLAY_BOW_INVITE` | posture | 3.0s | Play bow — classic joyful play invitation |
| `ACT_SHAKE_TOY_WITH_MOUTH` | head | 4.0s | Grip toy and shake rapidly — play display |
| `ACT_NIP_GENTLY_AT_PANTS_OR_HAND` | mouth | 1.5s | Soft inhibited nip — playful mouthing |
| `ACT_PLACE_PAW_ON_KNEE` | paw | 2.0s | Paw on person's knee — polite attention request |
| `ACT_STAND_AND_SCRATCH_HIGH` | paw | 4.0s | Stretch up and scratch high surface |
| `ACT_SCRATCH_DOOR_OR_FENCE` | paw | 5.0s | Scratch at door/fence — open/pass/mark |
| `ACT_SNIFF_OBJECT` | head | 4.0s | Close-up sniffing of object |
| `ACT_PUSH_OBJECT_WITH_PAW` | paw | 3.0s | Push/nudge object — test if it moves |
| `ACT_SCRATCH_OBJECT_GENTLY` | paw | 3.0s | Gentle scratch — exploratory tactile |
| `ACT_BARK_AT_OBJECT` | vocal | 3.0s | Bark repeatedly — alarm/curiosity/frustration |
| `ACT_KNOCK_OVER_OBJECT` | paw | 2.0s | Deliberately knock over upright object |
| `ACT_NIBBLE_OBJECT` | mouth | 4.0s | Gentle nibble/gnaw — investigative or comfort |
| `ACT_SNIFF_SLIPPERS_OR_SOCKS` | head | 3.0s | Sniff slipper/sock — investigate owner's scent |
| `ACT_BITE_SLIPPERS_OR_SOCKS` | mouth | 4.0s | Grip and bite/chew slipper or sock |
| `ACT_POUNCE_ON_SLIPPERS_OR_SOCKS` | motion | 2.0s | Spring and land on slipper/sock — predatory play |
| `ACT_SCRATCH_SLIPPERS_OR_SOCKS_WITH_PAW` | paw | 3.0s | Scratch/bat at slipper/sock |
| `ACT_SNIFF_TRASH_CAN` | head | 3.0s | Sniff exterior of trash can |
| `ACT_SNIFF_DELIVERY_BOX` | head | 4.0s | Sniff delivery box — check outside scents |
| `ACT_BITE_DELIVERY_BOX` | mouth | 4.0s | Bite corner/edge — try to open/shred |
| `ACT_SCRATCH_DELIVERY_BOX_WITH_PAW` | paw | 3.0s | Scratch at box surface — open or investigate |
| `ACT_SNIFF_TISSUE` | head | 2.0s | Sniff tissue — investigate scent |
| `ACT_SCRATCH_TISSUE_WITH_PAW` | paw | 2.0s | Bat at tissue — shred or play |
| `ACT_SNIFF_AROUND_DOOR` | head | 4.0s | Sniff door edges/gap — check outside scents |
| `ACT_LEAN_AGAINST_DOOR` | motion | 4.0s | Press body/shoulder against door |
| `ACT_LIE_BY_DOOR` | posture | 10.0s | Lie beside door and wait |
| `ACT_SCRATCH_DOOR` | paw | 4.0s | Scratch at door — request to open |
| `ACT_LIE_FLAT_ON_BELLY` | posture | 3.0s | Belly flat against ground — calm posture |
| `ACT_LIE_ON_SIDE` | posture | 3.0s | Settle onto side — relaxed posture |
| `ACT_LICK_PAWS` | mouth | 5.0s | Repeatedly lick front paws — calm self-soothing |
| `ACT_YAWN_SLOWLY` | mouth | 3.0s | Slow deliberate yawn — calming signal |
| `ACT_LIE_BELLY_UP` | posture | 4.0s | Roll onto back, belly exposed — ultimate trust |
| `ACT_PAW_AT_OWNER` | paw | 2.0s | Gentle paw — calm attention seeking |
| `ACT_STRETCH_BODY_RELAXEDLY` | posture | 4.0s | Full-body relaxed stretch |
| `ACT_WAG_TAIL_GENTLY` | tail | 3.0s | Gentle relaxed tail wag — low-intensity joy |
| `ACT_STEP_EXCITEDLY_IN_PLACE` | motion | 3.0s | Shift weight paw-to-paw — excited anticipation |
| `ACT_HOP_IN_PLACE_WITH_FRONT_PAWS_UP` | motion | 2.0s | Small hops, front paws lifting — bouncy joy |
| `ACT_NUDGE_OWNER_HAND_OR_LEG_WITH_NOSE` | head | 2.0s | Push nose against owner — joyful greeting |
| `ACT_SPIN_IN_CIRCLE` | motion | 3.0s | Rotate in tight circles — high-intensity joy |
| `ACT_POUNCE_FORWARD` | motion | 2.0s | Spring forward, both paws ahead — excited pounce |
| `ACT_BARK_SHORT_EXCITED` | audio | 1.5s | Short high-pitched bark — joyful play vocalization |
| `ACT_WIGGLE_BODY` | motion | 3.0s | Full-body wiggle — excitement greeting |
| `ACT_BARK_OR_WHINE` | audio | 3.0s | Excited barks or high-pitched whines |
| `ACT_MOUTH_GENTLY` | mouth | 3.0s | Gently take hand/sleeve — no bite pressure |
| `ACT_LICK_LIPS` | mouth | 2.0s | Quick lip flick — anxiety/appeasement signal |
| `ACT_WHINE_LOW` | audio | 3.0s | Soft low-pitched whine — anxious unease |
| `ACT_STARE_INTENTLY` | head | 5.0s | Unwavering wide-eyed stare — hypervigilance |
| `ACT_TILT_HEAD` | head | 2.0s | Head tilt — uncertainty/curiosity |
| `ACT_SCRATCH_FREQUENTLY` | paw | 5.0s | Rapid body scratching — displacement behavior |
| `ACT_STIFFEN_BODY` | posture | 4.0s | Tense all muscles — freeze-like anxiety |
| `ACT_TUCK_TAIL` | tail | 3.0s | Tail tucked between legs — classic fear posture |
| `ACT_DILATE_PUPILS` | expression | 2.0s | Widen pupils — whale eye / anxiety response |
| `ACT_WHINE_HIGH` | audio | 2.0s | High-pitched whine — acute anxiety |
| `ACT_CHECK_OWNER` | head | 2.0s | Frequent glance toward owner — seeking reassurance |
| `ACT_FREEZE_ALERT` | posture | 5.0s | Stop abruptly, ears forward — fear alert |
| `ACT_WALK_AWAY` | motion | 5.0s | Turn and walk away — avoidance response |
| `ACT_GROWL_LOW` | audio | 3.0s | Deep rumbling growl — warning/fear defensive |
| `ACT_TREMBLE_AND_SHAKE` | motion | 5.0s | Shiver and tremble — intense fear/cold stress |
| `ACT_LOSE_CONTROL` | posture | 3.0s | Collapse — extreme fear, submissive urination |
| `ACT_CALL_FOR_HELP` | audio | 4.0s | Loud distressed barks — summon assistance |
| `ACT_OBSERVE_QUIETLY` | head | 5.0s | Silent distant observation |
| `ACT_STAND_AND_OBSERVE` | posture | 5.0s | Stand still and focus — alert curiosity |
| `ACT_STAND_ALERT` | posture | 4.0s | Fully alert, ears forward — orienting posture |
| `ACT_APPROACH_SLOWLY` | motion | — | Slow cautious approach (from approachSlowly alias) |
| `ACT_SNIFF_GROUND` | head | — | Nose-to-ground sniff (from sniffGround alias) |
| `ACT_PAW_AT_OBJECT` | paw | — | Paw at object (from pawAtObject alias) |

#### Composite Action (32 个) — 多子动作序列

| Action Unit | Controller | Timeout | Description |
|-------------|-----------|---------|-------------|
| `ACT_SIT_OR_LIE_BY_BOWL` | posture | 5.0s | Choose sit or lie next to food bowl |
| `ACT_CHEW_OR_CARRY_FOOD` | mouth | 8.0s | Chew food or carry piece to safe spot |
| `ACT_TURN_HEAD_AND_WIPE_MOUTH` | head | 3.0s | Turn head, wipe mouth on foreleg |
| `ACT_SNIFF_AND_CIRCLE` | motion | 8.0s | Sniff ground while circling for elimination spot |
| `ACT_WALK_AWAY_OR_LIE_DOWN` | motion | 6.0s | Walk away or lie down — post-eating wind-down |
| `ACT_WALK_AWAY_OR_SHAKE_HEAD` | motion | 4.0s | Walk away or shake head — post-elimination |
| `ACT_ROLL_OVER` | posture | 4.0s | Rotate from one side to the other |
| `ACT_LIE_ON_SIDE_AND_STRETCH` | posture | 5.0s | Lower to side-lying, extend limbs |
| `ACT_CIRCLE_AROUND` | motion | 5.0s | Move in circles — excited orbiting behavior |
| `ACT_GETUP_CRAWL` | motion | 5.0s | Crawl forward then stand up |
| `ACT_GETUP_ROLL` | motion | 4.0s | Roll onto back, rotate, push up to stand |
| `ACT_GETUP_BOUNCE` | motion | 3.0s | Spring up from lying in one bounce |
| `ACT_GETUP_STRETCH` | posture | 6.0s | Stretch front then hind legs before standing |
| `ACT_GETUP_SIT` | posture | 3.0s | Transition from lying to sitting |
| `ACT_SHAKE_HEAD_OR_SMACK_LIPS` | head | 2.0s | Light-sleep adjustment |
| `ACT_BARK_AND_LIE_DOWN_IF_NO_CHARGER` | vocal | 10.0s | Bark alert, lie down — low-battery fallback |
| `ACT_STARE_AND_TILT_HEAD` | head | 4.0s | Fix gaze and tilt head — curiosity/confusion |
| `ACT_LOWER_HEAD_FLOP_EARS_WAG_TAIL` | head | 4.0s | Appeasement and friendly signal |
| `ACT_CARRY_AND_SHAKE_OBJECT` | mouth | 5.0s | Grip object, shake side-to-side — predatory play |
| `ACT_RUN_IN_CIRCLES_OR_CHASE` | motion | 8.0s | Run circles or chase — high-energy social play |
| `ACT_GUARD_FOOD_BOWL_OR_SNACK_CABINET` | posture | 12.0s | Stand/lie protectively near food resource |
| `ACT_SIT_OR_LIE_AT_FEET` | posture | 5.0s | Settle at person's feet — calm companionship |
| `ACT_RUN_IN_CIRCLES_OR_ZOOMIES` | motion | 8.0s | High-speed unpredictable patterns — zoomies |
| `ACT_TROT_AND_LOOK_AROUND` | motion | 8.0s | Trot while scanning surroundings |
| `ACT_WALK_SLOWLY_AND_SNIFF_GROUND` | motion | 10.0s | Slow walk, nose to ground |
| `ACT_CRAWL_THROUGH_LOW_GAP` | motion | 6.0s | Lower body, crawl under opening |
| `ACT_CARRY_AND_DROP_OBJECT_AGAIN` | mouth | 6.0s | Pick up, carry, drop, pick up again |
| `ACT_CARRY_AND_HIDE_OBJECT` | mouth | 10.0s | Pick up, carry away, hide |
| `ACT_TOUCH_OR_CARRY_OBJECT_WITH_MOUTH` | mouth | 5.0s | Touch/mouth/pick up object |
| `ACT_SIT_AND_WAG_TAIL_GENTLY` | posture | 5.0s | Sit with gentle wag — contented joy |
| `ACT_SWAY_BODY_WITH_WAGGING_TAIL` | motion | 4.0s | Whole-body happy dance |
| `ACT_WAG_TAIL_FAST_AND_STEP_IN_PLACE` | motion | 3.0s | Fast tail wag + stepping — mid-level joy |
| `ACT_RAISE_TAIL_AND_WAG` | tail | 3.0s | High tail position + enthusiastic wag |
| `ACT_ROLL_OVER_AND_SHOW_BELLY` | posture | 4.0s | Roll to back, expose belly — joyful trust |
| `ACT_BEG_FOR_FOOD` | posture | 6.0s | Sitting/standing begging posture |
| `ACT_TROT_AND_BOUNCE` | motion | 6.0s | Bouncy springy trot — excited happy movement |
| `ACT_PACE_BACK_AND_FORTH` | motion | 10.0s | Fixed path pacing — stereotypical anxiety |
| `ACT_RETREAT_WITH_TAIL_TUCKED` | motion | 6.0s | Backward retreat with tail tucked |
| `ACT_LOWER_BODY_AND_EXPLORE` | motion | 6.0s | Crouch and move forward cautiously |
| `ACT_HOLD_TAIL_LEVEL_AND_WAG` | tail | 3.0s | Horizontal tail + slow wag — curious interest |

#### Task (18 个) — 需导航/感知/跟踪

| Action Unit | Controller | Timeout | Description |
|-------------|-----------|---------|-------------|
| `ACT_CHASE_ROLLING_FOOD` | motion | 10.0s | Track and intercept rolling food |
| `ACT_RETURN_TO_CHARGER` | nav | 30.0s | Navigate to dock, dock and rest |
| `ACT_RUMMAGE_THROUGH_TRASH_CAN` | motion | 15.0s | Explore/rummage through trash can |
| `ACT_SEARCH_FOR_PERSON` | nav | 20.0s | Systematic search for specific person |
| `ACT_FOLLOW_AND_CLING` | motion | 15.0s | Follow person closely — attachment behavior |
| `ACT_FETCH_TOY` | motion | 15.0s | Run, retrieve, return — classic fetch |
| `ACT_FIND_COOL_SPOT_AND_LIE_DOWN` | nav | 15.0s | Search for cool surface, lie down |
| `ACT_FLEE_QUICKLY` | motion | 8.0s | Run at max speed — panic flight |
| `ACT_HIDE_AWAY` | motion | 10.0s | Move to hiding spot, conceal — anxious retreat |
| `ACT_AVOID_AND_HIDE` | motion | 10.0s | Move away from threat and hide |
| `ACT_MOVE_RAPIDLY` | motion | 8.0s | Sustained high pace — directed excited travel |
| `ACT_RUN_BACK_AND_FORTH` | motion | 8.0s | Run between two points — joyful indecision |
| `ACT_RUN_ZOOMIES` | motion | 10.0s | Full-speed irregular bursts — zoomies |
| `ACT_PATROL_SURROUNDINGS_SLOWLY` | motion | 15.0s | Slow deliberate patrol, scanning calmly |
| `ACT_WAIT_BY_DOOR` | posture | 15.0s | Wait calmly by door for it to open |
| `ACT_LIE_ON_SIDE_BESIDE_PERSON` | posture | 12.0s | Settle beside person — calm companionship |
| `ACT_CARRY_AND_SHAKE` | mouth | 6.0s | Carry object while shaking head — intense play |
| `ACT_CARRY_SLIPPERS_OR_SOCKS_TO_PERSON` | mouth | 12.0s | Pick up slipper/sock and carry to person |
| `ACT_CARRY_DELIVERY_BOX_TO_PERSON` | mouth | 12.0s | Pick up delivery box and carry to person |
| `ACT_CARRY_TISSUE_TO_PERSON` | mouth | 10.0s | Pick up tissue and carry to person |
| `ACT_APPROACH_OWNER_FEARFULLY` | motion | 10.0s | Approach owner with lowered body — seeking protection |
| `ACT_FOLLOW_MOVEMENT` | head | 8.0s | Track moving target visually + reposition |
| `ACT_CIRCLE_AND_INSPECT` | motion | 10.0s | Circle target while sniffing/inspecting |
| `TASK_ORIENT_TO_ANIMAL` | motion | 6.0s | Turn body/head to face detected animal |
| `TASK_OBSERVE_ANIMAL_RESPONSE` | head | 6.0s | Watch and interpret animal body language |
| `TASK_APPROACH_ANIMAL_SAFELY` | motion | 10.0s | Approach animal with de-escalation body language |
| `TASK_APPROACH_PERSON` | motion | 10.0s | Walk toward detected person |
| `TASK_APPROACH_OBJECT` | motion | 8.0s | Walk toward detected object |

#### Policy (5 个) — 策略决策，不产生身体动作

| Action Unit | Controller | Timeout | Description |
|-------------|-----------|---------|-------------|
| `ACT_IGNORE_SLIPPERS_OR_SOCKS` | meta | 0.0s | Trained leave-it for slippers/socks |
| `ACT_IGNORE_TRASH_CAN` | meta | 0.0s | Trained leave-it for trash can |
| `ACT_IGNORE_DELIVERY_BOX` | meta | 0.0s | Trained impulse control for delivery boxes |
| `ACT_IGNORE_TISSUE` | meta | 0.0s | Trained leave-it for tissues |
| `ACT_IGNORE_DOOR` | meta | 0.0s | Trained calm behavior near doors |

#### Modifier (1 个) — 修改后续执行参数

| Action Unit | Controller | Timeout | Effects |
|-------------|-----------|---------|---------|
| `ACT_SLOW_MOVEMENT` | meta | 0.0s | speed×0.45, acceleration×0.60, amplitude×0.80 |

### 7.2 按控制器分组

| Controller | Count | 说明 |
|-----------|-------|------|
| `motion` | 43 | 底盘/身体运动 |
| `posture` | 27 | 姿态变换 |
| `mouth` | 25 | 嘴巴/舌头/进食 |
| `head` | 24 | 头部/颈部/鼻子 |
| `paw` | 23 | 前爪/后爪 |
| `audio` | 13 | 声音输出（吠/嚎/呜咽） |
| `tail` | 6 | 尾巴动作 |
| `vocal` | 3 | 叫声（bark/how label） |
| `nav` | 4 | 导航 |
| `expression` | 1 | 面部表情 |
| `meta` | 6 | 元控制（policy/modifier） |

---

## 8. Stages 与 Selection Policies 说明

### 8.1 Stage Types (阶段类型)

| stage_type | 典型 Behavior | 说明 |
|-----------|-------------|------|
| `prepare` | eat* / defecate / sleepNow | 准备阶段（接近目标、定位） |
| `eating` | eatNormally / eatExcitedly | 进食阶段 |
| `eliminating` | defecate | 排泄阶段 |
| `groom` | cleanSelf | 梳理阶段 |
| `sleep_pose` / `sleeping` / `wakeup` | sleepNow | 睡眠三个阶段 |
| `recover` | restInPlace | 恢复阶段 |
| `recharge` | recharge | 充电阶段 |
| `orient` | testAnimalBoundary / inviteAnimalToPlay | 朝向阶段 |
| `express` | testAnimalBoundary / express* | 表达阶段 |
| `observe` | testAnimalBoundary | 观察阶段 |
| `approach` | greetAnimal / seekHumanInteraction / inviteHumanToPlay / inspectObject | 接近阶段 |
| `greet` / `invite` | greetAnimal / inviteAnimalToPlay / inviteHumanToPlay | 问候/邀请阶段 |
| `request` | requestResourceFromHuman | 请求阶段 |
| `search` | seekHumanInteraction | 搜索阶段 |
| `interact` | seekHumanInteraction | 互动阶段 |
| `explore` | exploreRoom | 探索阶段 |
| `inspect` | inspectObject / inspectKnownObject | 检查阶段 |
| `exit` | eat* / defecate | 退出阶段 |

### 8.2 Selection Policies (选择策略)

| 策略 | 说明 | 使用场景 |
|------|------|---------|
| `random_one` | 均匀随机选 1 个 | 大多数 stage（默认） |
| `weighted_random` | 按 weight 权重随机 | — |
| `condition_first` | 选第一个满足条件的 | recharge 分支、seekHumanInteraction 搜索/接近 |
| `fixed` | 固定候选（全执行） | — |
| `sequence` | 按顺序依次执行 | — |
| `random_n` | 随机选 N 个 | — |
| `loop_random` | 循环随机（min~max loops） | sleepNow sleeping stage |

### 8.3 Stage Modifiers (阶段修饰)

| 修饰 | 含义 |
|------|------|
| `required: true` | 该 stage 必须成功，行为才算成功 |
| `required: false` (opt) | 可选 stage，失败不影响行为成功 |
| `skip_on_fail: true` | 失败后跳过，继续下一个 stage |
| `skip_on_fail: false` | 失败后中止整个 behavior |
| `fixed: true` | 固定 stage，不可跳过/重排 |
| `loop` | 循环执行 (shallow: 0-3, deep: 0-2) |

### 8.4 Success Condition Types (成功条件类型)

| 类型 | 说明 | 使用 Behavior |
|------|------|-------------|
| `action_completed` | 动作正常完成 | 大部分 stage |
| `posture_reached` | 到达目标姿态 | prepare, exit, sleep_pose, wakeup |
| `distance_reached` | 到达目标距离 | approach, orient |
| `orientation_achieved` | 朝向容差内 | testAnimalBoundary orient |
| `duration_elapsed_or_food_consumed` | 最少进食时间或食物耗尽 | eating |
| `elimination_completed` | 完成排泄周期 | eliminating |
| `min_duration_elapsed` | 最少持续 time | groom, recover, explore |
| `posture_held` | 维持姿态最少 time | sleep_pose |
| `sleep_cycle_completed` | 一个睡眠周期完成 | sleeping |
| `charging_or_resting` | 已 dock 或正在躺下 | recharge |
| `person_found` | 搜索找到人 | search |
| `observation_completed` | 观察最少 time | observe |
