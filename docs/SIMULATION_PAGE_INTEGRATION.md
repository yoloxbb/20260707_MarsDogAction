# 仿真页面接入说明

> 面向仿真页面、Web 前端和可视化联调人员。同步版本：2026-07-31。

## 0. ROS Domain 部署约束

当前板端底盘 `/agv_pro_node` 位于 `ROS_DOMAIN_ID=1`。行为树、动作执行器、
仿真桥及 rosbridge 必须全部从相同环境启动：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

WebSocket 页面本身没有 ROS Domain，但它所连接的 rosbridge 进程必须位于
Domain 1。否则页面可能看到一套行为节点，而动作执行器或底盘处于另一套互相
隔离的 ROS 图中。

## 1. 页面应使用的接口

| 用途 | ROS2 接口 | 类型 | 页面使用建议 |
|---|---|---|---|
| 发起/取消行为 | `/execute_behavior` | `marsdog_interfaces/action/ExecuteBehavior` | 正式业务接口 |
| Goal 可视化 | `/debug/execute_behavior/goal` | `std_msgs/msg/String` | 调试订阅 |
| Stage/动作/进度 | `/debug/execute_behavior/feedback` | `std_msgs/msg/String` | 当前页面展示接口 |
| 终态可视化 | `/debug/execute_behavior/result` | `std_msgs/msg/String` | 当前页面展示接口 |

Debug Topic 的 `String.data` 本身还是一段 JSON 字符串，因此经 rosbridge
接收时需要解析两层：

```javascript
function onRosbridgeMessage(envelope) {
  // 第一层由 rosbridge 客户端解析，第二层是 std_msgs/String.data。
  const event = JSON.parse(envelope.msg.data);
  renderBehaviorEvent(event);
}
```

业务控制以 Action 为准。Debug Topic 适合当前仿真显示，但属于调试协议；
前端不要向这些 Topic 发布消息。

如果执行器启用了 AGV 适配层，页面仍然只发送 `ExecuteBehavior.Goal`，不要
从页面直接发布 `/cmd_vel`。动作执行器会根据最终选中的精确 `ACT_*` 生成
Twist 运动组。当前仅 13 个直接指令动作接入 AGV；其他动作后续由导航功能
实现，不会在当前版本中直接驱动小车。

`respond_owner_call` 是额外的动态角度动作：行为树把语音事件的
`wake_angle` 作为 `params_json.wake_angle_deg` 发送，动作执行器调用 Nav2
`/spin` 让车头朝向声源。页面不要自行换算角度或直接调用 `/spin`。

启用 Nav2 适配层后，睡眠、清洁、进食、排泄和充电等已配置行为会先导航到
A–E 语义点位，再执行原行为树 Stage 对应的小车运动。页面仍只发送
`ExecuteBehavior.Goal`，不要直接发布 `/goal_pose`。点位与行为范围见
[Nav2 语义点位适配说明](NAV2_WAYPOINT_INTEGRATION.md)。

## 2. Action Goal

正式 Action 类型来自 `marsdog_interfaces`。

```text
string  goal_id
string  behavior_id
string  behavior_name
int32   priority_level
string  params_json
float64 timeout_sec
```

页面请求对象示例：

```json
{
  "goal_id": "sim-20260729-0001",
  "behavior_id": "ui-click-0001",
  "behavior_name": "sit_down",
  "priority_level": 5,
  "params_json": "{}",
  "timeout_sec": 10.0
}
```

字段约束：

| 字段 | 页面规则 |
|---|---|
| `goal_id` | 每次执行生成新的唯一值；用于关联 Goal、Feedback 和 Result |
| `behavior_id` | 页面或行为树侧的业务 ID；建议每次执行也唯一 |
| `behavior_name` | 大小写敏感，只发送第 6 节列出的直接行为名 |
| `priority_level` | 当前合法范围 0–6；普通页面操作建议使用 5 |
| `params_json` | 必须是合法 JSON 字符串；无参数时传 `"{}"` |
| `timeout_sec` | 行为总超时，必须大于页面预期动画时长 |

命令行联调：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'sim-0001', behavior_id: 'ui-0001', \
    behavior_name: 'sit_down', priority_level: 5, \
    params_json: '{}', timeout_sec: 10.0}"
```

## 3. 页面接收数据

### 3.1 Goal Debug JSON

```json
{
  "goal_id": "sim-0001",
  "behavior_id": "ui-0001",
  "behavior_name": "sit_down",
  "priority_level": 5,
  "params": {},
  "timeout_sec": 10.0,
  "timestamp": 1785312000.123
}
```

这里的 `behavior_name` 是上游请求名称。

唤醒 Goal 的 `params` 示例：

```json
{
  "use_wake_angle": true,
  "wake_angle_deg": 35.0,
  "wake_confidence": 1205.0,
  "wake_frame_id": "base_link"
}
```

页面可据此显示声源方向；字段必须从 Goal `params` 读取。

### 3.2 Feedback Debug JSON

```json
{
  "goal_id": "sim-0001",
  "behavior_id": "ui-0001",
  "behavior_name": "sit_down",
  "status": "RUNNING",
  "progress": 1.0,
  "current_stage": "action",
  "current_action": "ACT_BASIC_SIT",
  "safe_to_interrupt": true,
  "message": "Stage 1/1: ACT_BASIC_SIT — OK",
  "timestamp": 1785312001.123
}
```

页面字段使用规则：

- `behavior_name` 与 Goal 中的直接行为名完全相同，不做 alias 或改写。
- `progress` 是已完成 Stage 数除以总 Stage 数，范围 0–1。
- `current_stage` 只存在于 Debug JSON，不在正式 Action Feedback 中。
- `current_action` 是行为树对应表里的精确 `ACT_*`，大小写不能修改。
- `message` 只用于显示和日志，不作为结构化数据解析来源。
- `safe_to_interrupt=false` 时禁用普通“取消”按钮；紧急停止按钮不受此限制。

当前实现不是 10Hz 流式反馈，而是每个 Stage 执行完成后发布一次。页面不能
依赖固定刷新频率，也不能用“长时间没有 Feedback”直接判定失败。

### 3.3 Result Debug JSON

```json
{
  "goal_id": "sim-0001",
  "behavior_id": "ui-0001",
  "behavior_name": "sit_down",
  "status": "SUCCESS",
  "result": "completed",
  "reason": "behavior completed",
  "duration_sec": 1.0,
  "failed_action": "",
  "interrupted_by": "",
  "reward": 1.0,
  "emotion_delta_json": "{}",
  "need_delta_json": "{}",
  "timestamp": 1785312001.223
}
```

页面终态判断：

```javascript
const succeeded =
  event.status === "SUCCESS" && event.result === "completed";

const terminal = new Set([
  "SUCCESS",
  "FAILURE",
  "FAILED",
  "CANCELED",
  "TIMEOUT",
  "NO_ELIGIBLE_ACTION",
  "INVALID_PARAMS",
  "UNSUPPORTED_BEHAVIOR"
]).has(event.status);
```

当前 ROS2 Action transport terminal state 与业务结果并非完全等价。页面必须
以 Result payload 的 `status` 和 `result` 为准。

## 4. 推荐页面状态机

```text
IDLE
  │ send goal
  ▼
PENDING
  │ goal accepted
  ▼
RUNNING
  │ feedback: 更新 progress/current_stage/current_action
  ├──────────────────────────────┐
  │ result SUCCESS/completed     │ cancel/failure/timeout result
  ▼                              ▼
SUCCEEDED                      FAILED_OR_CANCELED
  └────────────── reset/new goal ──────────────► PENDING
```

推荐页面保存：

```typescript
type BehaviorExecutionView = {
  goalId: string;
  behaviorId: string;
  requestedBehaviorName: string;
  resolvedBehaviorName: string;
  status: "IDLE" | "PENDING" | "RUNNING" | "SUCCEEDED" | "FAILED";
  progress: number;
  currentStage: string;
  currentAction: string;
  safeToInterrupt: boolean;
  message: string;
  resultReason: string;
};
```

所有事件使用 `goal_id` 关联。收到其他 Goal 的事件时，不要覆盖当前卡片；
多 Goal 页面应按 `goal_id` 建立 Map。

## 5. rosbridge 订阅示例

原生 rosbridge JSON：

```json
{
  "op": "subscribe",
  "topic": "/debug/execute_behavior/feedback",
  "type": "std_msgs/msg/String"
}
```

同样订阅：

```text
/debug/execute_behavior/goal
/debug/execute_behavior/result
```

使用 roslibjs 时：

```javascript
const feedbackTopic = new ROSLIB.Topic({
  ros,
  name: "/debug/execute_behavior/feedback",
  messageType: "std_msgs/msg/String"
});

feedbackTopic.subscribe((message) => {
  const feedback = JSON.parse(message.data);
  updateExecutionByGoalId(feedback.goal_id, {
    resolvedBehaviorName: feedback.behavior_name,
    status: feedback.status,
    progress: feedback.progress,
    currentStage: feedback.current_stage,
    currentAction: feedback.current_action,
    safeToInterrupt: feedback.safe_to_interrupt,
    message: feedback.message
  });
});
```

不同 rosbridge/roslibjs 版本可能要求 `std_msgs/String`；以实际 bridge 的
ROS2 类型解析规则为准，消息字段始终是 `data: string`。

## 6. 仿真页面直接行为名

页面和 Action Server 都只使用以下 53 个行为。其他行为名会在 Goal 验证阶段
被拒绝。

### 生理、资源与休息（10）

```text
eatNormally
seekFood
eatExcitedly
seekFoodUrgently
barkShortAlert
lickPaws
sleepOnSide
sleepNow
restInPlace
recharge
```

### 动物互动（3）

```text
testAnimalBoundary
greetAnimal
inviteAnimalToPlay
```

### 人类与资源互动（3）

```text
seekHumanInteraction
seekInteraction
inviteHumanToPlay
```

### 探索和物体（8）

```text
exploreRoom
inspectObject
inspectFamiliarPlayItem
inspectTrashCan
inspectDeliveryBox
inspectTissuePaper
inspectDoor
inspectDogFood
```

### 情绪表达（12）

```text
expressCalmWithHuman
expressCalmAlone
expressJoyWithHuman
expressJoyAlone
expressCuriosityWithHuman
expressCuriosityAlone
expressExcitementWithHuman
expressExcitementAlone
expressAnxietyWithHuman
expressAnxietyAlone
expressFearWithHuman
expressFearAlone
```

### 直接指令（17）

```text
respond_owner_call
sit_down
lie_down
stand_up
wait_in_place
come_to_owner
follow_owner
give_paw
high_five
roll_over
spin_around
return_to_owner
drop_object
play_dead
bring_object
fetch_object
emergency_stop
```

完整的行为、Stage 和动作候选关系见
[`config/behavior_tree_actions.yaml`](../config/behavior_tree_actions.yaml)。
页面如果需要生成“行为详情/候选动作”面板，可在构建期读取该文件；不要维护
第二份手写动作表。

## 7. 动作触发和动画映射

`current_action` 应直接作为仿真动画映射键：

```typescript
const animation = animationByActionId[feedback.current_action];
if (!animation) {
  showUnsupportedAction(feedback.current_action);
} else {
  playAnimation(animation);
}
```

动作 ID 共 188 个，唯一目录为 `config/action_catalog.yaml`。

仿真端必须记录未实现的 `ACT_*`，不能静默替换成相近动作，否则会再次产生
“行为名称正确但执行动作不一致”的问题。

当前 Feedback 在 Stage 完成后才发布。如果仿真动画需要在动作开始时同步触发，
现有接口还缺少 `unit_started` 事件；在接口扩展前，不要假设 Feedback 是动作
起始信号。

AGV 真车可能已在 Feedback 到达前完成当前运动组。AGV Topic、限速、运动映射
和急停联调见 [AGV ROS2 运动适配说明](AGV_ROS2_INTEGRATION.md)。

对于带定点导航的行为，首个 Stage Feedback 会在 Nav2 到点后才出现。页面从
Goal 被接受到首个 Feedback 之间应保持“执行中/导航中”，不能因暂时没有
`current_action` 判定超时。页面 Goal 的 `timeout_sec` 也必须覆盖导航时间。

当前四个定制 AGV 动作的页面展示语义如下：

| `current_action` | 真车代理运动 | 页面动画建议 |
|---|---|---|
| `ACT_BASIC_SIT` | 左转 30°、右转 30°回正 | 坐下/左右轻摆 |
| `ACT_BASIC_WAIT` | 右转 90°、左转 90°回正 | 等待/左右观察 |
| `ACT_TRICK_PLAY_DEAD` | 顺时针原地旋转 360° | 装死 |
| `ACT_INTERACT_RETURN_OWNER` | 掉头 180°后前进一小段 | 返回主人 |

这些真车运动是动作语义的开环代理，不改变行为树名称和 `current_action`。
页面仍按精确 `ACT_*` 选择动画，不需要根据 `/cmd_vel` 反推动作。

唤醒朝向的页面展示语义：

| `behavior_name` | `current_action` | Goal 动态参数 | 真车运动 |
|---|---|---|---|
| `respond_owner_call` | `ACT_INTERACT_RESPOND_CALL` | `wake_angle_deg` | Nav2 `/spin` 原地转向声源 |

该动作的角度是每次事件动态产生的，不能维护成固定动画角度。详细契约见
[唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)。

## 8. 已知数据注意事项

- `behavior_name` 和 `ACT_*` 都大小写敏感。
- 原表拼写 `expressCuriosiexpressCuriosityWithHumanty` 已整理为
  `expressCuriosityWithHuman`；错误字符串不会被接受。
- 原表重复的第一组 `expressFearWithHuman` 已按语义归入
  `expressAnxietyAlone`。
- 原表“随机行为 N 选 1”的 N 与动作行数不一致时，以实际列出的动作行为准。
- `barkShortAlert` 在原表中对应排泄动作。当前代码严格按表执行，仿真端也应
  显示实际 `ACT_*`；后续如需修正，必须先更新双方采用的新对照表。

## 9. 联调检查清单

- [ ] `ros2 action info /execute_behavior` 能看到 Action Server。
- [ ] 页面生成唯一 `goal_id` 和 `behavior_id`。
- [ ] `params_json` 是字符串形式的合法 JSON。
- [ ] Goal、Feedback、Result 能按 `goal_id` 关联。
- [ ] 页面按 Result payload 而不是 transport state 判断成功。
- [ ] Feedback 按 Stage 到达，不要求 10Hz。
- [ ] `current_action` 原样匹配动画键，未知动作明确告警。
- [ ] 新页面主列表只使用第 6 节的 53 个直接行为。
- [ ] `emergency_stop` 映射为 `ACT_SYSTEM_EMERGENCY_STOP`。
- [ ] `sit_down` 映射为 `ACT_BASIC_SIT`。

## 10. 相关文件

- [ROS2 集成说明](ROS2.md)
- [AGV ROS2 运动适配说明](AGV_ROS2_INTEGRATION.md)
- [Nav2 语义点位适配说明](NAV2_WAYPOINT_INTEGRATION.md)
- [唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)
- [行为树动作对接契约](BEHAVIOR_TREE_ACTION_CONTRACT.md)
- [行为树运行映射](../config/behavior_tree_actions.yaml)
- [严格动作目录](../config/action_catalog.yaml)
- 正式接口：`marsdog_interfaces/action/ExecuteBehavior`
