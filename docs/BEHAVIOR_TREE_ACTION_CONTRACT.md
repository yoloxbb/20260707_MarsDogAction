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
  behavior_name 精确属于新表 73 个行为?
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
73 direct behaviors
182 referenced actions
0 behavior aliases
0 legacy behavior fallbacks
0 unreferenced actions
```

## 上游 Goal 字段

| 字段 | 处理 |
|---|---|
| `goal_id` | 执行唯一 ID，原样回传 |
| `behavior_id` | 上游行为实例 ID，原样回传 |
| `behavior_name` | 精确匹配 73 个直接行为，大小写敏感 |
| `priority_level` | 写入执行上下文 |
| `params_json` | 解析为参数字典 |
| `timeout_sec` | 行为总墙钟超时；从 Goal 开始执行计时，覆盖解析、校验、waypoint_nav/随机 Nav2 导航及全部 Stage/Unit |

不在新表中的名称返回：

```text
unsupported_behavior: '<requested name>'
```

不会尝试 alias、相似名称或历史模板。

### 精确语音指令身份

词库、KWS 和模型意图产生的可执行语音事件共用同一条边界：行为树先用精确
`event_type + command_id` 白名单选择语义 Behavior，再通过 `params_json` 传递：

| 字段 | 用途 |
|---|---|
| `trigger_event` / `specific_event_type` | 保留精确 AudioEvent v2 事件 |
| `command_key` / `command_id` | 稳定词库键与指令身份 |
| `command_catalog_version` | 追踪识别时使用的词库版本 |
| `intent_source` / `dispatch_role` | 记录识别来源与分发角色 |
| `voice_slots` | 行为树审核后允许透传的槽位 |

这些字段在 `ExecutionContext` 中结构化保存，只用于追踪、参数化和后续结果关联；
它们不参与 Behavior 或 `ACT_*` 选择。Action 仍只按 Goal 顶层
`behavior_name` 查找 Stage，Voice 槽位中的 `action_name`、`behavior` 不可信且不应
由行为树透传。同一执行语义复用既有 Behavior；只有目标、执行流程、安全约束或
结果语义发生变化时，才需要新增 Behavior 和相应 Stage/动作映射。

`HOLD_POSITION` 是普通保持指令：Voice 词库中的产品参考名是
`ACT_HOLD_POSITION`，但不作为 Action 的路由输入。完整执行链为
`EVT_VOICE_COMMAND_HOLD_POSITION` → `hold_position` →
`ACT_CONTROL_HOLD_POSITION`，保持 Lv1 语义，不得改写或提升为 Lv0
`emergency_stop` / `ACT_SYSTEM_EMERGENCY_STOP`。

本批 19 个核心语音指令已全部进入正式行为表。新增的 8 个行为及动作边界为：

| Behavior | Action Stage |
|---|---|
| `walk_to_random_point` | waypoint_nav `place="K"` + `ACT_NAV_WALK_RANDOM` |
| `go_out_to_play` | waypoint_nav `place="K"` + `ACT_NAV_GO_OUT_TO_PLAY` |
| `go_home` | waypoint_nav home 点 + `ACT_NAV_GO_HOME` |
| `approach_owner` | `ACT_INTERACT_APPROACH_OWNER_CLOSER` |
| `back_up` | `ACT_BASIC_BACK_UP` |
| `stand_still` | `ACT_BASIC_STAND` 后 `ACT_CONTROL_HOLD_STANDING` |
| `hold_position` | `ACT_CONTROL_HOLD_POSITION` |
| `quiet` | `ACT_CONTROL_QUIET`，同时停止当前行为音频 |

三个导航行为要求真实导航控制器，不能在控制器缺失时走 mock 成功。
`walk_to_random_point` 与 `go_out_to_play` 都提交保留目标 `K`，由 waypoint_nav 在实时
`/map` 中选点；`go_home` 使用与地点文件一致的精确名称。`hold_position` 是普通 Lv1 保持，不具有
Lv0 `emergency_stop` 的全局抢占语义。

控制器路由默认值为 `unsupported`。只有明确的无物理动作策略
`ACT_CONTROL_QUIET` 显式使用 `mock`；任何未配置路由的动作都会失败，不会通过
等待后返回伪成功。`bring_object`、`fetch_object` 和 `drop_object` 已绑定
`perception_navigation_manipulation` 且要求真实控制器。Go2 与 Lite3 的有效
路由由各自的动作配置覆盖；Go2 的 `fidelity: proxy` 身体表达只表示 SportMode
序列完成，不声称物体操作完成。

### 唤醒朝向参数

`respond_owner_call` 必须携带：

| `params_json` 字段 | 约束 |
|---|---|
| `use_wake_angle` | `true` |
| `wake_angle_deg` | 声源角度，单位度，有限数值 |
| `wake_confidence` | 上游唤醒置信度/分数 |
| `wake_frame_id` | 必须为原始输入帧 `microphone_array` |

这些字段只决定 `ACT_INTERACT_RESPOND_CALL` 的动态底盘朝向，不修改行为名或
动作 ID。详见 [唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)。

### 语音呼叫者接近参数

`approach_voice_caller` 必须携带非空 `interaction_id`，并明确
`strict_target_lock=true`、`allow_target_switch=false`。`target` 必须是
`human` 且包含完整 `vision_epoch:human:track_id`。还须携带非空 `wake_id`，
与 `speaker_role=owner/family` 相符的 `speaker_id`、`speaker_status=matched`，
以及不小于 1.5 m 的有限 `stand_off_distance_m`。Action 只向 VisionTask 发一次
`locate_person_once`，收到 `status=0`、目标 ID 一致的有效结果后，按
`navigation_required` 决定是否发送一次 Nav2 `/navigate_to_pose`。

### 视觉安全事件行为

`respond_person_fall` 和 `respond_stop_gesture` 是两个独立的精确行为名。它们
分别只能选择 `ACT_PERCEPTION_RESPOND_PERSON_FALL` 与
`ACT_PERCEPTION_RESPOND_STOP_GESTURE`。Go2/Lite3 均将其路由为停车动作，
不允许用带侧移、前进或旋转的动作代替。

### 社交、动物和物体目标接近

以下行为的第一个必需 Stage 固定为
`ACT_APPROACH_VISUAL_TARGET -> visual_target_approach`：

```text
seekHumanInteraction, seekInteraction, inviteHumanToPlay
expressCalmWithHuman, expressJoyWithHuman, expressCuriosityWithHuman
expressExcitementWithHuman, expressAnxietyWithHuman, expressFearWithHuman
testAnimalBoundary, greetAnimal, inviteAnimalToPlay
inspectObject, inspectFamiliarPlayItem, inspectTrashCan
inspectDeliveryBox, inspectTissuePaper, inspectDoor, inspectDogFood
unhappy, miss_owner, farewell_leave
```

`params_json.target` 必须包含匹配行为策略的 `target_type` 和非空
`target_id`。若上游提供 `vision_epoch`，Action 强制与当前视觉流一致；当前普通
Need 目标未携带 epoch 时，Action 会先在最新有效视觉快照中唯一绑定同一
`track_id/target_id/identity`，再把当前 epoch 与轨迹固定到本次执行。不能唯一
绑定、没有稳定轨迹、公制距离无效或目标丢失时，接近 Stage 失败并阻止后续动作。
`exploreRoom` 没有具体目标，不包含该 Stage。
目标在 Goal 边界短暂进入 `temporarily_lost` 时，Action 先零速等待配置的视觉重捕获
窗口，不会立即把情绪延续判为 `visual_target_not_found`。

三个主人定制行为均为两阶段精确契约：

| Behavior | 视觉 Stage | 表达 Stage |
|---|---|---|
| `unhappy` | `ACT_APPROACH_VISUAL_TARGET` | `ACT_OWNER_UNHAPPY` |
| `miss_owner` | `ACT_APPROACH_VISUAL_TARGET` | `ACT_EXPRESS_MISS_YOU` |
| `farewell_leave` | `ACT_APPROACH_VISUAL_TARGET` | `ACT_OWNER_GOING_OUT` |

上游必须传入 `target_type=human + identity=owner`，不得仅凭行为名由 Action
猜测人物。速度、人体框目标高度和
到达保持由 Action 配置收紧；`farewell_leave` 保持阶段只跟随同一锁定 Track。

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
| `current_action` | Stage 内为精确 `ACT_*`；前置点位导航为 `WAYPOINT_NAV/<code>` |
| `safe_to_interrupt` | 当前是否可普通取消；点位导航只透传服务端明确给出的安全阶段 |

Debug Feedback 额外包含 `current_stage`。

### Result

| 字段 | 含义 |
|---|---|
| `behavior_name` | 与请求名称相同 |
| `status` | 业务终态 |
| `result` | `completed`、`canceled` 或 `failed` |
| `reason` | 终态原因 |

`status=TIMEOUT` 时 `result=failed`、`reason=behavior_goal_timeout`。同一 Goal
预算会下压到 Nav2 和当前 Unit；适配器即使在截止时间后返回成功，也不能把行为
终态改回成功。

### 取消与抢占状态机

Tree 对一个 Goal 至少区分 `SENDING`、`RUNNING`、`CANCEL_REQUESTED`、
`TERMINAL`。调用 `cancel_goal_async` 或收到取消响应都只进入
`CANCEL_REQUESTED`，不能删除 goal handle、结果 Future、goal_id/behavior 映射，
也不能伪造 `CANCELED` Result。Tree 必须继续接收该 Goal 的 Action Result：

- 真实 `CANCELED` 才允许按取消释放执行权；
- 取消竞争中真实 `SUCCEEDED` 仍按成功处理；
- 真实 `FAILED`/`TIMEOUT` 仍按失败处理；
- 真实 Result 到达前不启动替代 Goal，不清除当前行为和 inflight reservation。

普通抢占只有最新 Feedback 的 `safe_to_interrupt=true` 时才发起；紧急停止可立即
请求取消，但仍需等待真实 Result。点位 `DISPATCHED` 与
`RECOVERY_REQUIRED` 必须反馈 `safe_to_interrupt=false`；后者表示恢复结果未知，
Tree 应保持当前 Goal 活跃并阻塞后续导航。

Action 内部每 5 秒 query 是取消确认/漏状态恢复周期，不是 Tree 可据此释放锁的
超时。替代 Goal 的 8 秒锁等待是失败关闭兜底；生产路径应由 Tree 在旧 Goal
终态到达后才下发替代 Goal，正常情况下不应依赖这 8 秒竞争窗口。

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
- `barkShortAlert` 暂按表执行排泄相关动作。2026-09-18 起其 Stage 顺序为
  `circle` → `action` → `exit` → `head_up`，最后一步 `ACT_RAISE_HEAD`（抬头）
  是完成信号。`ACT_RAISE_HEAD` 是本项目新增的动作，不在上游原表中。
