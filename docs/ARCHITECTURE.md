# MarsDog Action Executor 架构

## 1. 职责边界

本包是行为执行器，不负责行为决策。上游行为树选择一个
`behavior_name`；本包根据新动作对照表选择并执行对应 `ACT_*`。

```text
marsdog_behavior
  Behavior Tree / Action Client
           │ ExecuteBehavior
           ▼
marsdog_action_executor
  strict name validation
  stage selection
  action execution
           │ ACT_*
           ▼
controller / simulator
```

## 2. 严格配置边界

### 行为

唯一运行时行为源：

```text
config/behavior_tree_actions.yaml
```

加载后必须恰好得到新表中的 53 个行为。其他 YAML 中存在的历史行为不会进入
运行时注册表。

### 动作

唯一运行时动作目录：

```text
config/action_catalog.yaml
```

该文件只包含 `behavior_tree_actions.yaml` 引用的 188 个唯一动作。
`ConfigLoader` 会校验动作目录和引用集合一致；出现表外动作或缺少引用动作都会
阻止节点启动。

### 不支持的兼容机制

- 不解析旧 behavior alias；
- 不加载 `behavior_templates.yaml`；
- 不加载 `behaviors.yaml`；
- 不从情绪池动态生成候选动作；
- 不接受新对照表之外的行为；
- 不暴露新对照表之外的动作。

## 3. 请求处理管道

```text
ExecuteBehavior.Goal
        │
        ▼
GoalParser
  params_json -> ExecutionContext
        │
        ▼
BehaviorResolver
  requested_behavior_name ∈ configured 53?
        │ yes
        ▼
ConfigLoader.get_behavior_template(name)
        │
        ▼
for stage in template.stages
  cancel / timeout check
        │
        ▼
StageExecutor
  candidates
    -> EligibilityChecker
    -> random_one
    -> controller route
       -> AGV adapter / UnitExecutor
        │
        ├─ PostureManager update
        ├─ InterruptManager update
        └─ executed_units append
        │
        ▼
ResultEvaluator
  all_required_stages_completed
```

名称校验是精确匹配。无法匹配时：

```text
is_valid = false
error_reason = "unsupported_behavior: '<name>'"
```

## 4. 核心模块

| 模块 | 职责 |
|---|---|
| `goal_parser.py` | ROS Goal / JSON 转 `ExecutionContext` |
| `behavior_resolver.py` | 严格校验 53 个直接行为名 |
| `config_loader.py` | 加载新表、校验严格动作目录、启动校验 |
| `eligibility_checker.py` | 条件、姿态和安全过滤 |
| `stage_executor.py` | Stage 候选选择和执行 |
| `adapters/agv_adapter.py` | 精确动作到 `/cmd_vel` Twist 运动组 |
| `adapters/navigation_adapter.py` | Nav2 点位导航与现有 Stage 动作物理代理 |
| `interrupt_manager.py` | immediate / safe_point / non_interruptible |
| `posture_manager.py` | 执行后的姿态状态更新 |
| `result_evaluator.py` | 行为终态评估 |
| `ros_node.py` | `/execute_behavior` ROS2 Action Server |
| `debug_publishers.py` | Goal / Feedback / Result 调试 JSON |

## 5. 主要数据结构

### ExecutionContext

| 字段 | 含义 |
|---|---|
| `requested_behavior_name` | 上游原始名称 |
| `resolved_behavior_name` | 严格模式下与 requested 相同 |
| `params` | 原始 JSON 参数 |
| `priority_level` | 0–6 |
| `target` | 可选目标 |
| `current_stage` | 当前 Stage |
| `current_unit` | 当前 `ACT_*` |
| `completed_stages` | 已完成 Stage |
| `executed_units` | 已成功执行动作 |
| `is_valid` / `error_reason` | 请求校验状态 |

### Behavior Template

```yaml
behavior_name:
  behavior_name: behavior_name
  success_condition: all_required_stages_completed
  stages:
    - stage_id: action
      order: 1
      selection_policy: random_one
      required: true
      candidates:
        - {unit_id: ACT_EXAMPLE}
```

## 6. 选择与执行

新表中的所有 Stage 使用 `random_one`：

1. 读取候选动作；
2. 过滤不满足执行条件的候选；
3. 对剩余候选均匀随机选择一个；
4. 按动作目录元数据创建 UnitExecutor；
5. 成功后将精确 ID 写入 `executed_units`。

动作 ID 不做改名、归一化或相近动作替换。

## 7. AGV 路由

`StageExecutor` 现在会读取 `controller_routes.yaml`。路由为 `agv` 时，完整
`ACT_*` 交给 `AgvMotionAdapter` 执行；AGV 未启用或没有对应适配器时沿用原
UnitExecutor。

当前只为 13 个直接指令动作启用 AGV 路由。其余运动动作保留给后续导航、
定位和目标跟随适配器。

```text
ACT_*
  -> controller_routes.yaml: agv
  -> agv_motion_groups.yaml: motion group
  -> 10Hz geometry_msgs/Twist
  -> /cmd_vel
```

AGV 默认关闭。所有运动组结束、取消、紧急停止，以及 ROS context 仍有效的正常
退出都会发送零速度。底盘仍必须配置速度命令超时看门狗。普通行为串行执行，
`emergency_stop` 可绕过普通行为锁触发停止。

启用语义点位导航后，`navigation_waypoints.yaml` 先按 `behavior_name` 选择
A–E 点位并调用 Nav2；成功到点后，StageExecutor 仍从原行为树候选中选择精确
`ACT_*`，`BehaviorMobilityAdapter` 再按该动作 ID 选择 Twist 代理。导航与
Stage Twist 严格串行，取消和急停同时作用于 Nav2 Goal 与 `/cmd_vel`。

`respond_owner_call` 的 `ACT_INTERACT_RESPOND_CALL` 使用独立
`wake_orientation` 路由：从 `ExecutionContext.wake_angle_deg` 读取动态声源
角度，经零点/方向校准后调用 Nav2 `/spin`。它不进入固定 Twist 运动组。

## 8. ROS2 可观测性

正式接口：

```text
/execute_behavior
marsdog_interfaces/action/ExecuteBehavior
```

调试接口：

```text
/debug/execute_behavior/goal
/debug/execute_behavior/feedback
/debug/execute_behavior/result
```

Feedback 每个 Stage 完成后发布一次。`current_action` 是该 Stage 选择的精确
动作 ID。仿真页面细节见
[SIMULATION_PAGE_INTEGRATION.md](SIMULATION_PAGE_INTEGRATION.md)。

AGV 细节见 [AGV_ROS2_INTEGRATION.md](AGV_ROS2_INTEGRATION.md)。

## 9. 启动校验

`ConfigLoader.load_all()` 会检查：

- 行为存在非空 Stage；
- Stage order 不重复；
- `selection_policy` 合法；
- required Stage 候选不为空；
- 每个候选动作存在于动作目录；
- 动作目录没有新表之外的动作；
- `unit_type`、`interrupt_policy`、timeout、weight 合法。
- AGV 动作、运动组、controller route 三方一致；
- AGV 频率、速度和时长字段合法。

## 10. 扩展流程

增加行为：

1. 在 `behavior_tree_actions.yaml` 增加行为和 Stage；
2. 在 `action_catalog.yaml` 注册新增动作，并保证不保留未引用动作；
3. 更新行为契约测试的期望名称和映射摘要；
4. 同步仿真页面动作资源；
5. 运行测试。

不要通过添加 alias、旧模板或隐式动作替换扩展接口。
