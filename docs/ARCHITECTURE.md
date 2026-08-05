# MarsDog Action Executor — 架构与通信设计 (v2)

## 目录

1. [系统架构](#1-系统架构)
2. [请求处理管道](#2-请求处理管道)
3. [核心模块](#3-核心模块)
4. [数据模型](#4-数据模型)
5. [配置体系](#5-配置体系)
6. [执行单元系统](#6-执行单元系统)
7. [条件与过滤](#7-条件与过滤)
8. [姿态与中断管理](#8-姿态与中断管理)
9. [行为成功条件](#9-行为成功条件)
10. [与行为树通信](#10-与行为树通信)
11. [场景详解](#11-场景详解)
12. [扩展指南](#12-扩展指南)

---

## 1. 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│                      marsdog_interfaces                      │
│                  (公共接口定义，纯 action/msg)                  │
└──────────────────────┬───────────────────────────────────────┘
                       │  import ExecuteBehavior
          ┌────────────┴────────────┐
          ▼                         ▼
┌─────────────────────┐   ┌─────────────────────────────────┐
│  marsdog_behavior   │   │   marsdog_action_executor       │
│  (Action Client)    │   │   (Action Server)               │
│                     │   │                                 │
│  行为决策            │   │  ┌──────────────────────────┐  │
│  情绪/需求仲裁        │   │  │     Action Server        │  │
│  候选池管理           │   │  └──────────┬───────────────┘  │
│  语音命令处理         │   │             │                  │
│                     │   │  ┌──────────▼───────────────┐  │
│                     │   │  │      GoalParser          │  │
│                     │   │  │  params_json → Context   │  │
│                     │   │  └──────────┬───────────────┘  │
│                     │   │             │                  │
│                     │   │  ┌──────────▼───────────────┐  │
│                     │   │  │   BehaviorResolver       │  │
│                     │   │  │  alias / inject / fallback│  │
│                     │   │  └──────────┬───────────────┘  │
│                     │   │             │                  │
│                     │   │  ┌──────────▼───────────────┐  │
│                     │   │  │    StageExecutor         │  │
│                     │   │  │  filter → select → exec  │  │
│                     │   │  └──────────┬───────────────┘  │
│                     │   │             │                  │
│                     │   │  ┌──────────▼───────────────┐  │
│                     │   │  │   Controller Adapters    │  │
│                     │   │  │  motion/gimbal/audio/... │  │
│                     │   │  └──────────────────────────┘  │
└─────────────────────┘   └─────────────────────────────────┘
          │                            │
          │   /execute_behavior        │
          └──────── Action ────────────┘
```

---

## 2. 请求处理管道

```
ROS2 Action Goal
  │  goal_id, behavior_id, behavior_name, priority_level, params_json, timeout_sec
  ▼
┌─────────────────────────────────────────────────────────────┐
│ 1. GoalParser.parse_from_ros_goal(goal_request)             │
│    → ExecutionContext.from_goal(behavior_name, params_json) │
│    → _safe_parse_params (JSON parse, 失败 → is_valid=False) │
│    → _normalise_level (LOW/MID/HIGH → UPPERCASE)           │
│    → _normalise_sleep_depth (shallow/deep)                 │
│    → _normalise_interaction (mode ↔ interactive 互推导)     │
│    → _normalise_intensity (numeric validation)              │
│    → _normalise_target (dict check)                         │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. BehaviorResolver.resolve(ctx)                            │
│    → ALIAS_MAP lookup (38 entries)                          │
│    → inject default params (level/interaction_mode/...)     │
│    → validate canonical name                                │
│    → interaction fallback (interactive + no target → solo)  │
│    → level from variant derivation                          │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. StageExecutor.execute_stage() × N stages                 │
│    For each Stage:                                          │
│      a. Read candidates from behavior_templates.yaml        │
│      b. EligibilityChecker.filter_candidates(ctx)           │
│         - condition checks (40+ named conditions)           │
│         - posture validation (from_postures)                │
│         - safety policy check                               │
│      c. Select by policy (7 strategies)                     │
│      d. Execute via UnitExecutor (5 types)                  │
│      e. PostureManager.apply_unit_to_posture()              │
│      f. InterruptManager check                              │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. ResultEvaluator.evaluate(ctx, stage_results)             │
│    → Check behavior-level success condition                 │
│    → Build BehaviorResult (JSON-serializable)               │
│    → Return via Action Server                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 核心模块

### 3.1 execution_context.py

`ExecutionContext` — 所有上游参数的统一标准化载体。48 个字段覆盖：
- 行为名（原始 + 解析后）
- 来源元数据（source, trigger_event, intent）
- 优先级（priority_level, sub_priority）
- 参数（variant, level, intensity, interaction_mode, interactive, target）
- 上下文（sleep_depth, elimination_type, charger_*, object_category）
- 运行时（current_stage, current_unit, current_posture, cancel_requested）
- 缩放（speed/accel/amplitude/duration_scale）
- 累积（completed_stages, executed_units）

**设计原则**：
- 缺失可选字段 → 默认值，不崩溃
- 未知字段 → 保留在 metadata
- 非法值 → warning + 安全默认值
- `is_valid=False` 时 read `.error_reason`

### 3.2 goal_parser.py

`GoalParser` — ROS2 Goal → ExecutionContext 的唯一入口。
- `parse(behavior_name, params_json, seed, priority_level, timeout_sec)`
- `parse_from_ros_goal(goal_request)` — duck-typing，不依赖 action 类型

### 3.3 behavior_resolver.py

`BehaviorResolver` — 三阶段解析：
1. **Alias lookup** — 38 条内置映射 + YAML 覆盖
2. **参数注入** — alias 可注入 `level` / `interaction_mode` 等默认值到 typed fields
3. **交互回退** — `interactive` + 无 target → `solo`（记录 fallback_reason）
4. **Level 推导** — 从 `variant` 字符串提取 level

### 3.4 eligibility_checker.py

`EligibilityChecker` — 统一条件系统。
- 40+ 内置条件（`owner_visible`, `charger_known`, `elimination_type_pee`, ...）
- `filter_candidates(candidates, ctx)` → eligible list
- 检查顺序：条件 → 姿态 → 安全策略
- 未知条件名 → error（启动校验），不静默跳过

### 3.5 stage_executor.py

`StageExecutor` — 单 Stage 执行引擎。
- 7 种 selection_policy
- 调用 `EligibilityChecker` 过滤
- 调用 `UnitExecutor` 执行
- 支持 `failure_policy`（abort / skip_stage / retry）
- 支持 `loop_policy`（loop_random + min/max_loops）

### 3.6 config_loader.py

`ConfigLoader` — YAML 加载 + 启动校验。
- 加载 10 个配置文件（部分可选）
- 15+ 项校验（非法 policy、空 required stage、alias 循环、weight < 0、timeout 异常等）
- 严重错误 → `ConfigurationError` → 阻止节点启动

---

## 4. 数据模型

### 4.1 BehaviorGoal（上游输入）

| 字段 | 类型 | 来源 |
|------|------|------|
| goal_id | str | 行为树 |
| behavior_id | str | 行为树 |
| behavior_name | str | 行为树候选池 |
| priority_level | int | 行为树 |
| params | dict | params_json 解析 |
| timeout_sec | float | 行为树 |

### 4.2 ExecutionContext（内部标准化）

| 字段 | 类型 | 说明 |
|------|------|------|
| requested_behavior_name | str | 原始名 |
| resolved_behavior_name | str | alias 解析后 |
| source | str\|None | need/emotion/audio |
| trigger_event | str\|None | NEED_HUNGER_TRIGGERED |
| intent | str\|None | eat_normal |
| level | str\|None | LOW/MID/HIGH |
| interaction_mode | str | solo/interactive |
| target | dict\|None | {target_type, visible, ...} |
| sleep_depth | str\|None | shallow/deep |
| elimination_type | str\|None | pee/poop |
| current_posture | str | standing/sitting/lying/... |

### 4.3 BehaviorResult（输出）

| 字段 | 说明 |
|------|------|
| success | bool |
| status | success/failure/canceled/timeout/no_eligible_action/... |
| requested_behavior_name | 原始名 |
| resolved_behavior_name | canonical 名 |
| completed_stages | 已完成的 stage_id 列表 |
| executed_units | 已执行的 unit_id 列表 |
| result_code | 结果码 |
| reward | -1.0 ~ 1.0 |

---

## 5. 配置体系

| 文件 | 大小 | 职责 |
|------|------|------|
| `behavior_templates.yaml` | 989 行 | 22 canonical behavior → Stage 定义 |
| `action_catalog.yaml` | 2614 行 | 202 执行单元元数据 |
| `behavior_aliases.yaml` | 181 行 | 38 条 alias + 参数注入 |
| `emotion_action_pools.yaml` | 125 行 | 6 情绪 × 2-3 level × 2 mode 动作池 |
| `controller_routes.yaml` | 28 行 | action → adapter 路由 |
| `safety_policies.yaml` | 11 行 | 10 个高风险动作开关 |

### behavior_templates.yaml 结构

```yaml
eatNormally:
  success_condition: eating_stage_completed
  stages:
    - stage_id: prepare
      order: 1
      required: true
      selection_policy: random_one
      candidates:
        - {unit_id: ACT_SNIFF_BOWL_EDGE, weight: 1.0}
        - {unit_id: ACT_PAW_AT_BOWL, weight: 1.0}
    - stage_id: eating
      order: 2
      required: true
      selection_policy: random_one
      candidates: [...]
    - stage_id: interaction
      order: 3
      required: false
      selection_policy: random_one
      failure_policy: skip_stage
      candidates:
        - {unit_id: ACT_PAUSE_AND_LOOK_AT_OWNER, conditions: [owner_visible]}
    - stage_id: exit
      order: 4
      required: true
      selection_policy: random_one
      candidates: [...]
```

### action_catalog.yaml 结构

```yaml
ACT_WAG_TAIL_GENTLY:
  unit_type: atomic_action
  controller: motion
  timeout_sec: 3.0
  interrupt_policy: safe_point
  from_postures: [standing, sitting]
  to_posture: same
  conditions: []
  description: "Gently wag tail — low-arousal joy expression"

ACT_RETURN_TO_CHARGER:
  unit_type: task
  controller: navigation_charging
  timeout_sec: 60.0
  interrupt_policy: safe_point
  success_condition: charger_reached
  failure_conditions: [charger_not_found, path_blocked, timeout]

ACT_IGNORE_DOOR:
  unit_type: policy
  controller: none
  interrupt_policy: immediate
  effect:
    complete_stage_without_motion: true

ACT_SLOW_MOVEMENT:
  unit_type: modifier
  controller: none
  interrupt_policy: immediate
  effects:
    speed_scale: 0.45
    acceleration_scale: 0.60
    amplitude_scale: 0.80
```

---

## 6. 执行单元系统

### 6.1 五种执行器

| 类型 | 类 | 行为 |
|------|-----|------|
| atomic_action | `AtomicActionExecutor` | Mock: `time.sleep(duration)` |
| composite_action | `CompositeActionExecutor` | 依次执行子动作 |
| task | `TaskExecutor` | RUNNING → SUCCESS/FAILURE/TIMEOUT/CANCELED 生命周期 |
| policy | `PolicyExecutor` | 直接返回 SUCCESS，不调控制器 |
| modifier | `ModifierExecutor` | 修改 `ctx` 的 scale 参数，不调控制器 |

### 6.2 Task 生命周期

```
IDLE → RUNNING → SUCCESS  (条件达成)
              → FAILURE  (条件失败)
              → TIMEOUT  (超时)
              → CANCELED (取消请求)
```

Mock task: `timeout_sec` 内每 0.1s 检查取消，到达 `duration_scale * 2s` 后返回 SUCCESS。

### 6.3 控制器路由

从 `controller_routes.yaml` 读取，默认 fallback: `motion`。

| 控制器 | Mock 类 | 真实目标 |
|--------|---------|---------|
| motion | MockMotionAdapter | `/motion/execute_motion` |
| gimbal_motion | MockGimbalAdapter | `/gimbal/set_target` |
| audio | MockAudioAdapter | — |
| navigation_motion | MockNavigationAdapter | `/navigation/navigate_to` |
| expression | MockExpressionAdapter | `/expression/play` |

---

## 7. 条件与过滤

### 条件检查器

40+ 内置条件，分类：

| 类别 | 条件 |
|------|------|
| 目标可见性 | `owner_visible`, `target_visible`, `person_not_visible`, `animal_target_available`, `human_target_available` |
| 距离 | `person_too_far`, `contact_distance`, `target_distance_valid` |
| 目标状态 | `target_moving`, `target_not_aggressive` |
| 充电 | `charger_known`, `charger_available`, `charger_unavailable` |
| 排泄 | `elimination_type_pee`, `elimination_type_poop` |
| 物品 | `toy_available`, `carrying_toy`, `object_carryable`, `object_safe_for_mouth` |
| 资源 | `food_resource_visible`, `food_in_hand_visible`, `rolling_food_detected` |
| 安全授权 | `jump_interaction_allowed`, `gentle_mouthing_allowed`, `allow_bite_object`, `allow_carry_object`, `allow_rummage_trash`, `allow_scratch_door` |
| 其他 | `leash_or_shoe_available`, `bell_interaction_supported`, `suitable_rubbing_object_available`, `contact_allowed`, `paw_contact_allowed` |

### 过滤流程

```
candidates (全部候选)
  → 条件检查 (named conditions)
  → 姿态检查 (from_postures)
  → 安全检查 (safety_policies)
  → 冷却检查 (cooldown_sec, max_repeat)
  → eligible (合法候选)
  → 按 selection_policy 选择
```

---

## 8. 姿态与中断管理

### 8.1 PostureManager

8 种姿态: `standing`, `sitting`, `lying`, `lying_side`, `lying_back`, `lying_belly`, `sleep_curled`, `moving`, `unknown`

- `is_posture_valid_for(from_postures)` — 检查 + 转换查询
- `apply_unit_to_posture(to_posture)` — 执行后更新
- `unknown` 姿态允许所有动作

### 8.2 InterruptManager

| 策略 | 状态转换 |
|------|---------|
| `immediate` | IDLE → CANCEL_REQUESTED → CANCELED（立即） |
| `safe_point` | IDLE → CANCEL_REQUESTED → 等安全点 → CANCELED |
| `non_interruptible` | IDLE → CANCEL_REQUESTED → 完成单元 → CANCELED |

Cleanup 流程：停止底盘 → 停止导航 → 停止云台 → 停止音频 → 释放物体 → 清除 Modifier → 安全姿态。

---

## 9. 行为成功条件

```yaml
eatNormally:         eating_stage_completed
eatExcitedly:        eating_stage_completed
defecate:            eliminating_stage_completed
cleanSelf:           groom_stage_completed
sleepNow:            sleep_pose_entered
restInPlace:         recover_stage_completed
recharge:            charging_detected
testAnimalBoundary:  express_stage_completed
greetAnimal:         greet_stage_completed
inviteAnimalToPlay:  invite_stage_completed
requestResourceFromHuman: request_stage_completed
seekHumanInteraction:     interact_stage_completed
inviteHumanToPlay:        invite_stage_completed
exploreRoom:              explore_stage_completed
inspectObject:            inspect_stage_completed
expressCalm:         at_least_one_expression
expressJoy:          at_least_one_expression
# ... (all 6 express* use at_least_one_expression)
```

---

## 10. 与行为树通信

### Action 字段

```
Goal:     goal_id, behavior_id, behavior_name, priority_level, params_json, timeout_sec
Result:   goal_id, behavior_id, behavior_name, status, result, reason, reward, emotion_delta_json, need_delta_json
Feedback: goal_id, behavior_id, behavior_name, status, progress, safe_to_interrupt, current_action, message
```

### params_json 契约

行为树下发的 `params_json` 示例：

```json
{
  "source": "need",
  "trigger_event": "NEED_HUNGER_TRIGGERED",
  "intent": "eat_normal",
  "priority_level": 3,
  "level": "TRIGGERED",
  "intensity": 82,
  "interaction_mode": "solo"
}
```

情绪示例：

```json
{
  "source": "emotion",
  "trigger_event": "EMO_JOY_MID",
  "intent": "express_joy",
  "level": "MID",
  "interaction_mode": "interactive",
  "target": {"target_type": "human", "identity": "owner", "visible": true}
}
```

---

## 11. 场景详解

### 11.1 需求触发 → eatNormally

```
行为树下发: behavior_name="eatNormally"
           params_json={"source":"need","trigger_event":"NEED_HUNGER_TRIGGERED","level":"TRIGGERED"}

GoalParser → ExecutionContext(
    requested_behavior_name="eatNormally",
    source="need",
    level="TRIGGERED",
    interaction_mode="solo", ...
)

BehaviorResolver → canonical="eatNormally" (no alias)

StageExecutor:
  Stage prepare: filter → eligible=[ACT_SNIFF_BOWL_EDGE, ACT_PAW_AT_BOWL, ACT_SIT_OR_LIE_BY_BOWL]
                 random_one → ACT_PAW_AT_BOWL → AtomicActionExecutor → SUCCESS
  Stage eating:   filter → eligible=[ACT_LICK_FOOD, ACT_CHEW_OR_CARRY_FOOD, ...]
                 random_one → ACT_LICK_FOOD → SUCCESS
  Stage interaction(optional): filter → ACT_PAUSE_AND_LOOK_AT_OWNER needs owner_visible → False
                               → ACT_CHASE_ROLLING_FOOD needs rolling_food_detected → False
                               → eligible=[ACT_CHANGE_POSTURE, ACT_GROWL_WHILE_EATING, ACT_BURP]
                               random_one → ACT_BURP → SUCCESS
  Stage exit:     random_one → ACT_LICK_LIPS_OR_NOSE → SUCCESS

ResultEvaluator → eating_stage_completed=True → SUCCESS
```

### 11.2 情绪 + alias: wagTailFast → expressJoy

```
行为树下发: behavior_name="wagTailFast"
           params_json={}

GoalParser → ExecutionContext(requested="wagTailFast", level=None, interaction_mode="solo")

BehaviorResolver:
  alias lookup → "wagTailFast" → resolved="expressJoy"
  inject → level="MID", interaction_mode="interactive"
  typed fields → ctx.level="MID", ctx.interaction_mode="interactive"
  fallback → ctx.interaction_mode="interactive" + no target → "solo"
  log: "interaction fallback: no valid target, requested=interactive → resolved=solo"

StageExecutor:
  从 emotion_action_pools.yaml 查 expressJoy/MID/solo:
    → [ACT_STEP_EXCITEDLY_IN_PLACE, ACT_SWAY_BODY_WITH_WAGGING_TAIL]
  random_one → ACT_STEP_EXCITEDLY_IN_PLACE → SUCCESS

ResultEvaluator → at_least_one_expression=True → SUCCESS
```

### 11.3 充电: recharge

```
行为树下发: behavior_name="recharge"
           params_json={"charger_known":true,"charger_available":true}

EligibilityChecker:
  condition_first:
    ACT_RETURN_TO_CHARGER → charger_known=True ✓, charger_available=True ✓ → selected
    ACT_BARK_AND_LIE_DOWN_IF_NO_CHARGER → charger_unavailable=False → skipped

TaskExecutor(ACT_RETURN_TO_CHARGER):
  RUNNING → ... → SUCCESS (charger_reached)
  ctx.metadata["charging_detected"] = True

ResultEvaluator → charging_detected=True → SUCCESS
```

---

## 12. 扩展指南

### 添加新 canonical behavior

1. 在 `config/behavior_templates.yaml` 添加 behavior 定义
2. 在 `behavior_resolver.py` 的 `_CANONICAL_BEHAVIORS` 集合中注册
3. 如需行为级成功条件，在 `result_evaluator.py` 的 `BEHAVIOR_SUCCESS_CONDITIONS` 中添加

### 添加新 action unit

1. 在 `config/action_catalog.yaml` 添加 unit 元数据
2. 如需特定 controller，在 `config/controller_routes.yaml` 添加路由
3. 如需安全授权，在 `config/safety_policies.yaml` 添加开关

### 添加新条件

1. 在 `eligibility_checker.py` 实现 `_cond_*` 函数
2. 调用 `register_condition("condition_name", _cond_*)` 注册
3. 在 behavior templates 的 `conditions` 列表中使用

### 添加旧行为别名

在 `config/behavior_aliases.yaml` 添加：

```yaml
old_name:
  resolved_behavior_name: canonical_name
  injected_params:
    level: MID
    interaction_mode: interactive
  alias_reason: "migration from legacy naming"
```

---

## 附录：关键文件索引

| 文件 | 行数 | 职责 |
|------|------|------|
| `execution_context.py` | ~280 | ExecutionContext + 参数标准化 |
| `goal_parser.py` | ~65 | GoalParser |
| `behavior_resolver.py` | ~210 | alias + inject + fallback |
| `eligibility_checker.py` | ~230 | 40+ 条件 + 过滤 |
| `stage_executor.py` | ~180 | 7 种策略 + 5 种执行单元 |
| `units/unit_executors.py` | ~200 | 5 种 UnitExecutor |
| `units/base_unit_executor.py` | ~65 | UnitState + UnitResult |
| `posture_manager.py` | ~70 | 姿态状态机 |
| `interrupt_manager.py` | ~110 | 3 种中断策略 |
| `result_evaluator.py` | ~160 | 行为成功条件 |
| `config_loader.py` | ~200 | YAML 加载 + 15 项校验 |
| `adapters/mock_adapters.py` | ~95 | 6 种 Mock adapter |
| `config/action_catalog.yaml` | 2614 | 202 执行单元 |
| `config/behavior_templates.yaml` | 989 | 22 canonical behavior |
| `config/behavior_aliases.yaml` | 181 | 38 条 alias |
| `config/emotion_action_pools.yaml` | 125 | 6 情绪动作池 |
