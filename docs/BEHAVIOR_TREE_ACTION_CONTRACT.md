# 行为树动作对接契约

仿真页面的 ROS2/rosbridge 字段、状态机和联调示例见
[仿真页面接入说明](SIMULATION_PAGE_INTEGRATION.md)。

## 运行链路

```text
marsdog_behavior / BehaviorTree
  ExecuteBehavior.Goal
    behavior_name + params_json + IDs/priority/timeout
        │
        ▼
GoalParser
  params_json -> ExecutionContext
        │
        ▼
BehaviorResolver
  behavior_name 精确属于新表 53 个行为?
        │
        ▼
ConfigLoader
  仅从 behavior_tree_actions.yaml 获取模板
        │
        ▼
StageExecutor
  有序 Stage -> eligible candidates -> random_one
        │
        ▼
UnitExecutor
  unit_id 原样执行
        │
        ├─ Feedback.current_action = 精确 ACT_*
        └─ Result.behavior_name = 原 behavior_name
```

## 唯一配置源

行为和 Stage：

[config/behavior_tree_actions.yaml](../config/behavior_tree_actions.yaml)

动作元数据：

[config/action_catalog.yaml](../config/action_catalog.yaml)

运行时边界：

```text
53 direct behaviors
188 referenced actions
0 behavior aliases
0 legacy behavior fallbacks
0 unreferenced actions
```

## 上游 Goal 字段

| 字段 | 处理 |
|---|---|
| `goal_id` | 执行唯一 ID，原样回传 |
| `behavior_id` | 上游行为实例 ID，原样回传 |
| `behavior_name` | 精确匹配 53 个直接行为，大小写敏感 |
| `priority_level` | 写入执行上下文 |
| `params_json` | 解析为参数字典 |
| `timeout_sec` | 行为总超时 |

不在新表中的名称返回：

```text
unsupported_behavior: '<requested name>'
```

不会尝试 alias、相似名称或历史模板。

### 唤醒朝向参数

`respond_owner_call` 必须携带：

| `params_json` 字段 | 约束 |
|---|---|
| `use_wake_angle` | `true` |
| `wake_angle_deg` | 声源角度，单位度，有限数值 |
| `wake_confidence` | 上游唤醒置信度/分数 |
| `wake_frame_id` | 当前必须为 `base_link` |

这些字段只决定 `ACT_INTERACT_RESPOND_CALL` 的动态底盘朝向，不修改行为名或
动作 ID。详见 [唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)。

## Stage 字段

| 字段 | 含义 |
|---|---|
| `stage_id` | 阶段结构化 ID |
| `order` | 执行顺序 |
| `selection_policy` | 新表统一使用 `random_one` |
| `required` | 是否为必需 Stage |
| `candidates[].unit_id` | 可执行的精确 `ACT_*` |

所有 Stage 均按配置顺序执行。每个 Stage 从过滤后的候选中选择一个动作。

## 下游字段

### Feedback

| 字段 | 含义 |
|---|---|
| `behavior_name` | 与请求的直接行为名相同 |
| `progress` | 已完成 Stage / 总 Stage |
| `current_action` | 本 Stage 选择的精确 `ACT_*` |
| `safe_to_interrupt` | 当前是否可普通取消 |

Debug Feedback 额外包含 `current_stage`。

### Result

| 字段 | 含义 |
|---|---|
| `behavior_name` | 与请求名称相同 |
| `status` | 业务终态 |
| `result` | `completed` 或 `failed` |
| `reason` | 终态原因 |

## 名称约束

- 不支持旧行为名称；
- 不支持旧行为 alias；
- 不支持新表外的旧动作；
- 不改写行为名称；
- 不改写动作 ID；
- 不静默选择相近动作；
- 名称大小写敏感。

以下旧名称必须被拒绝：

```text
defecate
go_back
expressJoy
emergencyStop
wagTailFast
seek_food_or_water
```

## 原始表整理

- `expressCuriosiexpressCuriosityWithHumanty` 整理为
  `expressCuriosityWithHuman`，错误字符串不接受。
- 第一组重复的 `expressFearWithHuman` 按动作语义整理为
  `expressAnxietyAlone`。
- 动作数量标注与动作行不一致时，以实际列出的 `ACT_*` 为准。
- `barkShortAlert` 暂按表执行排泄相关动作。
