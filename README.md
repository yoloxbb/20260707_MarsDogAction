# MarsDog Action Executor

仿生机器狗**行为执行引擎**。接收上游行为树下发的 `behavior_name` + `params_json`，解析参数 → 解析别名 → 生成阶段化动作计划 → 按条件过滤候选 → 选择执行单元 → 驱动控制器 → 返回 feedback 和 result。

> **职责**：本项目是「行为 → 参数解析 → 模板 → Stage → 条件过滤 → 单元选择 → 控制器执行」的执行器，**不是**行为决策器。

## 三包架构

```
marsdog_interfaces          ← 公共接口定义（action / msg）
       ↑              ↑
       │              │
marsdog_behavior    marsdog_action_executor
(Action Client)     (Action Server)  ← 本包
```

| 包 | 角色 | 说明 |
|---|---|---|
| `marsdog_interfaces` | 接口定义 | `ExecuteBehavior.action`，只需 `colcon build` |
| `marsdog_behavior` | 行为树 / Action Client | 决策层，下发 `behavior_name` + `params_json` |
| **`marsdog_action_executor`** | Action Server | **本包**，解析 → 计划 → 执行 → 反馈 |

## 快速开始

```bash
uv sync                                    # 安装依赖
uv run pytest -q                           # 全部测试（619 个）
uv run marsdog-action-demo                 # 独立 Demo（默认行为）
uv run marsdog-action-demo sit 42          # 指定行为 + 固定随机种子
uv run marsdog-action-demo expressJoy      # 情绪行为
uv run marsdog-action-demo wagTailFast     # 旧行为名 (alias → expressJoy)
```

## 项目结构

```
marsdog_action_executor/
├── marsdog_action_executor/
│   ├── models.py                  # BehaviorGoal / ActionStep / ActionStage / ActionPlan / ExecutionFeedback / ExecutionResult
│   ├── execution_context.py       # ExecutionContext — 参数标准化与校验
│   ├── goal_parser.py             # GoalParser — ROS2 Goal → ExecutionContext
│   ├── behavior_resolver.py       # BehaviorResolver — alias 解析 / 参数注入 / 交互回退
│   ├── behavior_catalog.py        # 旧版行为目录 (保留兼容)
│   ├── planner.py                 # 旧版 Planner (保留兼容)
│   ├── executor.py                # 旧版 Executor (保留兼容)
│   ├── eligibility_checker.py     # EligibilityChecker — 40+ 条件过滤
│   ├── posture_manager.py         # PostureManager — 姿态状态机
│   ├── interrupt_manager.py       # InterruptManager — 中断策略 (immediate/safe_point/non_interruptible)
│   ├── result_evaluator.py        # ResultEvaluator — 行为级成功条件
│   ├── stage_executor.py          # StageExecutor — 7 种选择策略 + 5 种执行单元
│   ├── config_loader.py           # ConfigLoader — YAML 加载 + 启动校验
│   ├── controller_adapters.py     # 旧版 Mock adapter (保留兼容)
│   ├── ros_node.py                # ROS2 Action Server
│   ├── debug_publishers.py        # /debug/execute_behavior/* 发布器
│   ├── ros2_compat.py             # HAS_ROS2 + marsdog_interfaces 优先导入
│   ├── standalone_demo.py         # 独立 Demo
│   ├── units/
│   │   ├── base_unit_executor.py  # BaseUnitExecutor + UnitState
│   │   └── unit_executors.py      # 5 种执行器: atomic/composite/task/policy/modifier
│   └── adapters/
│       └── mock_adapters.py       # 6 种 Mock: motion/gimbal/audio/nav/perception/expression
│
├── config/
│   ├── behavior_templates.yaml    # 22 canonical behavior 的完整 Stage 定义 (989 行)
│   ├── action_catalog.yaml        # 202 个执行单元元数据 (2614 行)
│   ├── behavior_aliases.yaml      # 38 条旧名→新名映射 (含参数注入)
│   ├── emotion_action_pools.yaml  # 6 情绪 × level × solo/interactive 动作池
│   ├── controller_routes.yaml     # action → adapter 路由
│   └── safety_policies.yaml       # 高风险动作授权开关
│
├── docs/ARCHITECTURE.md           # 架构与通信设计文档
├── launch/action_executor.launch.py
├── tests/                         # 619 tests
└── README.md
```

## 数据流 (v2)

```
ROS2 Action Goal (behavior_name + params_json)
    │
    ▼
GoalParser
    │  → ExecutionContext (标准化 level/sleep_depth/interaction_mode/target)
    ▼
BehaviorResolver
    │  1. alias 解析 (旧名 → canonical + 参数注入)
    │  2. 交互模式回退 (interactive 无 target → solo)
    │  3. level 从 variant 推导
    ▼
StageExecutor
    │  对每个 Stage:
    │    1. 读取候选池 (从 behavior_templates.yaml)
    │    2. EligibilityChecker 过滤 (条件/姿态/安全)
    │    3. UnitSelector 按策略选择 (random_one/weighted_random/condition_first/...)
    │    4. UnitExecutor 执行 (atomic_action/composite_action/task/policy/modifier)
    │    5. PostureManager 更新姿态
    │    6. InterruptManager 检查中断
    ▼
ExecutionFeedback (10Hz)              ExecutionResult
    progress / current_action            status / result_code / reward
    safe_to_interrupt / current_stage    completed_stages / executed_units
    current_posture
```

## Canonical Behaviors（22 个）

| 类别 | 行为 |
|------|------|
| 生理需求 | `eatNormally` `eatExcitedly` `defecate` `cleanSelf` `sleepNow` `restInPlace` `recharge` |
| 动物社交 | `testAnimalBoundary` `greetAnimal` `inviteAnimalToPlay` |
| 人类社交 | `requestResourceFromHuman` `seekHumanInteraction` `inviteHumanToPlay` |
| 探索 | `exploreRoom` `inspectObject` `inspectKnownObject` |
| 情绪表达 | `expressCalm` `expressJoy` `expressExcitement` `expressAnxiety` `expressFear` `expressCuriosity` |

行为树只下发 `behavior_name` + `params_json`（含 `level`、`interaction_mode`、`target` 等）。本包根据 `behavior_name` 选择模板，根据 `level` + `interaction_mode` 从情绪动作池中选候选。

## 别名机制

`config/behavior_aliases.yaml` 管理旧名→新名映射，支持参数注入：

| 旧名 | 新名 | 注入参数 |
|------|------|---------|
| `wagTailFast` | `expressJoy` | `level: MID, interaction_mode: interactive` |
| `wagTailGently` | `expressJoy` | `level: LOW, interaction_mode: interactive` |
| `headTilt` | `expressCuriosity` | `level: LOW, interaction_mode: interactive` |
| `hideAway` | `expressFear` | `level: HIGH, interaction_mode: solo` |
| `seek_food_or_water` | `eatNormally` | — |
| `emergency_stop` | `emergencyStop` | — |

## 执行单元分类

| unit_type | 说明 | 示例 |
|-----------|------|------|
| `atomic_action` | 短时确定动作，单控制器完成 | `ACT_WAG_TAIL_GENTLY`, `ACT_TILT_HEAD` |
| `composite_action` | 多子动作序列 | `ACT_PLAY_BOW_INVITE` |
| `task` | 需导航/感知/跟踪/循环检查 | `ACT_RETURN_TO_CHARGER`, `ACT_FETCH_TOY` |
| `policy` | 不产生身体动作，直接完成 Stage | `ACT_IGNORE_DOOR` |
| `modifier` | 修改后续执行参数 | `ACT_SLOW_MOVEMENT` (speed×0.45) |

## Selection Policies（7 种）

| 策略 | 说明 |
|------|------|
| `fixed` | 固定候选（按顺序全执行） |
| `sequence` | 按顺序依次执行 |
| `random_one` | 均匀随机选一个 |
| `weighted_random` | 按权重随机选一个 |
| `random_n` | 随机选 N 个 |
| `loop_random` | 循环随机（min_loops ~ max_loops） |
| `condition_first` | 选第一个满足条件的（用于充电分支等） |

选择流程：候选池 → 条件过滤 → 姿态检查 → 安全检查 → 冷却检查 → eligible → 选择。

## 条件系统（40+ 条件）

`EligibilityChecker` 统一管理，条件名在启动时校验。支持：

`owner_visible`, `target_visible`, `person_not_visible`, `person_too_far`, `contact_distance`, `target_moving`, `charger_known`, `charger_available`, `charger_unavailable`, `elimination_type_pee`, `elimination_type_poop`, `toy_available`, `carrying_toy`, `food_resource_visible`, `rolling_food_detected`, `object_carryable`, `object_safe_for_mouth`, `jump_interaction_allowed`, `gentle_mouthing_allowed`, `allow_bite_object`, `allow_carry_object`, `allow_rummage_trash`, `allow_scratch_door` 等。

## 中断与取消

| 策略 | 行为 |
|------|------|
| `immediate` | 立即停止控制器 → cleanup → CANCELED |
| `safe_point` | 设标志 → 等安全点 → cleanup → CANCELED |
| `non_interruptible` | 完成当前单元 → 不进入下一单元 → cleanup → CANCELED |

Cleanup：停止底盘/导航/云台/音频 → 释放物体 → 清除 Modifier → 安全姿态。

## 行为级成功条件

| Behavior | 条件 |
|----------|------|
| `eatNormally` / `eatExcitedly` | eating Stage 完成 |
| `defecate` | eliminating Stage 完成 |
| `sleepNow` | sleep_pose Stage 完成 |
| `recharge` | `charging_detected` |
| `seekHumanInteraction` | interact Stage 完成 |
| `express*` | 至少一个表达动作成功 |

## ROS2 集成

### 编译与启动

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select marsdog_interfaces marsdog_action_executor --symlink-install
source install/setup.bash

ros2 run marsdog_action_executor action_executor_node
```

### 接口清单

| 接口 | 类型 | 方向 |
|------|------|------|
| `/execute_behavior` | ROS2 Action Server | ← Client |
| `/debug/execute_behavior/goal` | `std_msgs/String` JSON | → 调试 |
| `/debug/execute_behavior/feedback` | `std_msgs/String` JSON | → 调试 (10Hz) |
| `/debug/execute_behavior/result` | `std_msgs/String` JSON | → 调试 |

### 手动验证

```bash
# 生理行为
ros2 action send_goal /execute_behavior marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 't1', behavior_name: 'eatNormally', priority_level: 3, params_json: '{\"source\":\"need\",\"level\":\"TRIGGERED\"}', timeout_sec: 30.0}"

# 情绪行为
ros2 action send_goal /execute_behavior marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 't2', behavior_name: 'expressJoy', priority_level: 5, params_json: '{\"source\":\"emotion\",\"level\":\"MID\",\"interaction_mode\":\"interactive\",\"target\":{\"target_type\":\"human\",\"visible\":true}}', timeout_sec: 10.0}"

# 旧行为名 (alias)
ros2 action send_goal /execute_behavior marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 't3', behavior_name: 'wagTailFast', priority_level: 5, params_json: '{}', timeout_sec: 10.0}"
```

## 控制器适配层

6 种 adapter（当前均为 mock）：

| Adapter | 目标接口 | 路由的动作 |
|---------|---------|-----------|
| Motion | `/motion/execute_motion` | 默认 |
| Gimbal | `/gimbal/set_target` | `ACT_TILT_HEAD` 等 |
| Audio | — | `ACT_BARK_*`, `ACT_WHINE_*`, `ACT_GROWL_*` |
| Navigation | `/navigation/navigate_to` | `ACT_RETURN_TO_CHARGER`, `ACT_RUN_ZOOMIES` |
| Perception | — | `TASK_APPROACH_*`, `ACT_FETCH_TOY` |
| Expression | `/expression/play` | `ACT_DILATE_PUPILS` |

## 运行测试

```bash
uv run pytest -q                              # 全部 619 tests
uv run pytest -q tests/test_catalog.py        # 目录完整性
uv run pytest -q tests/test_catalog_new.py    # 新行为专项
uv run pytest -q tests/test_executor.py       # 执行引擎
uv run pytest -q tests/test_planner.py        # weighted / alias / seed
uv run pytest -q tests/test_new_executor.py   # 新 API + adapter
```

> 无 ROS2 时测试不受影响（`HAS_ROS2` 条件导入）。

## 职责边界

### 本包负责 ✅

- `/execute_behavior` Action Server
- `params_json` → `ExecutionContext` 解析
- alias 解析 + 参数注入 + 交互模式回退
- Stage 顺序执行 + 条件过滤 + 姿态管理
- atomic/composite/task/policy/modifier 5 种执行单元
- 中断管理（immediate/safe_point/non_interruptible）
- 行为级成功条件判断
- controller adapter 路由
- debug topic（`/debug/*`）

### 本包不负责 ❌

- 行为决策 → `marsdog_behavior`
- 需求数值计算 / 阈值判断
- 情绪强度区间判断
- 全局行为优先级仲裁
- 需求/情绪值更新

## 待完成

- [ ] 真实硬件 controller adapters
- [ ] ConfigLoader 集成到 ros_node 启动流程
- [ ] 完整单元测试套件 (新模块)
- [ ] 移除 `action/ExecuteBehavior.action` 本地副本
- [ ] 移除 `MARSDOG_LEGACY_DEBUG_TOPICS` 兼容代码
