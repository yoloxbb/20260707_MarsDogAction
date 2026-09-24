# 仿真页面接入说明

> 面向外部 2D 仿真页面、Web 前端和可视化联调人员。同步版本：2026-09-04。

## 0. ROS Domain 部署约束

板端使用的 ROS Domain 需与行为树、动作执行器、
仿真桥及 rosbridge 必须全部从相同环境启动：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

WebSocket 页面本身没有 ROS Domain，但它所连接的 rosbridge 进程必须位于
Domain 1。否则页面可能看到一套行为节点，而动作执行器或底盘处于另一套互相
隔离的 ROS 图中。

### 0.1 系统边界和节点职责

外部 2D 页面不是 ROS2 节点。它通过 WebSocket 连接 `rosbridge_websocket`，
由 rosbridge 代它订阅 ROS2 Topic、发送或取消 ROS2 Action。不要把本仓库的
`emotion_display_node` 算入外部 2D 链路；它是另一套本机 Qt 调试界面。

| 组件/节点 | 所属模块 | 输入 | 输出 | 职责 |
|---|---|---|---|---|
| Voice 节点 | MarsDogVoiceInteraction | 麦克风音频 | `/perception/audio_event` | 生成 `EVT_VOICE_*` 事件；例如唤醒词及声源角度 |
| Vision 节点 | MarsDogVisionInteraction | 相机图像/深度 | `/perception/visual_event`、`/perception/vision/object_detections`、`/perception/vision/task` | 生成人、动物、物体及手势观测，为闭环接近提供目标数据 |
| `marsdog_behavior` | MarsDogTree | Voice/Vision/状态事件 | `/execute_behavior` Action Goal、`/behavior/attention_tracking` | 把上游事件映射为行为候选，仲裁优先级并派发精确 `behavior_name` |
| `action_executor_node` | MarsDogAction | `/execute_behavior`，可选的视觉/跟踪输入 | 三个 `/debug/execute_behavior/*` Topic；可选 `/cmd_vel`、waypoint_nav/Nav2 调用 | 校验行为、选择每个 Stage 的 `ACT_*`、执行控制器并返回终态 |
| `rosbridge_websocket` | rosbridge_suite | ROS2 Action/Topic | WebSocket JSON | ROS2 与外部网页之间的协议桥；不做业务决策 |
| 外部 2D 页面 | Web/仿真程序 | rosbridge WebSocket | 页面状态、动画；可选 Action Goal/Cancel | 按 `goal_id` 展示行为生命周期，按 `current_action` 选择动画 |
| Go2/Lite3 底盘驱动 | 底盘节点 | SportMode 或 `/cmd_vel`、`/simple_cmd` | 底盘实际运动及自身状态 | 执行速度或姿态指令；不是页面动画的数据源 |
| waypoint_nav / Nav2 | 点位导航、Nav2 | `/waypoint_nav/task`、`/spin` | `/waypoint_nav/status`、Nav2 result、底盘控制 | 完成 YAML 精确地点名/`K` 随机目标导航和动态声源朝向 |
| Go2 UWB 跟随节点组 | `uwb_aoa_pkg`、`go2_uwb_local_follow` | UWB 串口、双目、里程计 | `/cmd_vel` | 仅 `chassis_type:=go2`；完成定位跟随、障碍感知和局部规划 |

节点名以运行时实际启动配置为准；表中的 Topic/Action 名才是跨模块稳定边界。
页面只需直接连接 rosbridge，不需要分别连接 Voice、Vision、Tree、Action 或底盘节点。

### 0.2 端到端数据流

自动事件触发链路：

```text
Voice / Vision / 状态生产者
  -> EVT_* 事件
  -> marsdog_behavior：映射、候选去重、优先级仲裁
  -> /execute_behavior Goal {goal_id, behavior_name, params_json, ...}
  -> action_executor_node
       -> behavior_name -> ordered stages -> selected ACT_*
       -> controller route -> /cmd_vel、waypoint_nav 或 Nav2 /spin
       -> /debug/execute_behavior/goal|feedback|result
  -> rosbridge_websocket
  -> 外部 2D 页面
```

页面直接点选行为链路：

```text
外部 2D 页面
  -> rosbridge Action client
  -> /execute_behavior Goal
  -> action_executor_node
  -> Debug Goal / Feedback / Result
  -> rosbridge
  -> 页面按同一个 goal_id 更新状态和动画
```

页面直发 Goal 会绕过 Tree 的事件映射、候选去重和优先级仲裁，只适合手工联调。
正式自动运行应由 Tree 派发。`ros2_pose.py` 中的 `topic_navigator` 是独立的 Nav2
点位测试工具，不属于外部 2D 页面的必需链路。

### 0.3 “事件、行为、动作”三层含义

| 层级 | 命名示例 | 产生方 | 含义 | 页面用途 |
|---|---|---|---|---|
| 感知事件 | `EVT_VOICE_CALL_NAME`、`EVT_VISION_FALL` | Voice/Vision | 发生了什么 | 可选旁路展示；不是本页执行状态的权威来源 |
| 行为 | `respond_owner_call`、`sit_down` | Tree 或页面 | 要完成的业务意图 | 显示行为卡片、关联一次执行 |
| 动作单元 | `ACT_INTERACT_RESPOND_CALL`、`ACT_BASIC_SIT` | Action 配置与规划器 | 当前 Stage 选择的具体执行单元 | 作为 2D 动画映射键 |
| 控制命令 | `Twist`、Nav2 Goal | Action/控制器 | 真正驱动底盘的数据 | 页面不要发送，也不要反推语义 |

当前给外部 2D 页面定义的“执行事件”只有 Goal、Feedback、Result 三类。原始
`EVT_*` 不会被复制进这三个 Debug JSON；若页面还要展示原始语音或视觉事件，
需另行订阅对应感知 Topic，并且不能拿它们替代 Result 判断行为是否成功。

已在跨模块契约中明确的典型映射如下；其余 Voice/Emotion/Need 到行为的映射由
MarsDogTree 配置维护，不应在 Action 或页面中复制一份：

| 上游事件/条件 | Tree 行为 | Action 单元 | 控制器/效果 |
|---|---|---|---|
| `EVT_VOICE_CALL_NAME` + `wake_angle` | `respond_owner_call` | `ACT_INTERACT_RESPOND_CALL` | `wake_orientation` -> Nav2 `/spin` |
| 已识别并锁定的语音呼叫者 | `approach_voice_caller` | `ACT_INTERACT_APPROACH_VOICE_CALLER` | `person_nav_approach` -> VisionTask `locate_person_once` -> Nav2 `/navigate_to_pose` |
| `EVT_VISION_FALL` | `respond_person_fall` | `ACT_PERCEPTION_RESPOND_PERSON_FALL` | 安全响应并保持/发布零速度 |
| `EVT_VISION_STOP_GESTURE` | `respond_stop_gesture` | `ACT_PERCEPTION_RESPOND_STOP_GESTURE` | 停止响应并保持/发布零速度 |
| 页面“坐下” | `sit_down` | `ACT_BASIC_SIT` | Go2/Lite3 平台动作；页面播放坐下动画 |
| 页面“紧急停止” | `emergency_stop` | `ACT_SYSTEM_EMERGENCY_STOP` | 停止底盘；紧急语义 |

上表中的事件名大小写敏感。页面若旁路订阅感知事件，应保留原始字段并按上游模块
的 schema 校验，不能仅凭显示文案自行构造 `behavior_name`。

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

页面只发送 `ExecuteBehavior.Goal`，不要直接发布 `/cmd_vel`。
动作执行器根据精确 `ACT_*` 选择 Go2 或 Lite3 后端；
`approach_voice_caller` 对已锁定的人执行单次 SLAM 定位和按需 Nav2 导航。

`respond_owner_call` 是额外的动态角度动作：行为树把语音事件的
`wake_angle` 作为 `params_json.wake_angle_deg` 发送，动作执行器调用 Nav2
`/spin` 让车头朝向声源。页面不要自行换算角度或直接调用 `/spin`。

启用导航适配层后，睡眠、清洁、进食、排泄和充电等已配置行为会通过
`/waypoint_nav/task` 先导航到 YAML 中的精确地点名；随机行为则提交保留目标
`K`。导航成功后再执行原行为树 Stage 对应的平台动作。页面仍只发送
`ExecuteBehavior.Goal`，不要直接调用点位 Service 或发布 `/goal_pose`。点位与行为
范围见 [waypoint_nav 点位适配说明](NAV2_WAYPOINT_INTEGRATION.md)。

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
  "wake_frame_id": "microphone_array"
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
  "metadata_json": "{}",
  "timestamp": 1785312001.223
}
```

Result 字段说明：

| 字段 | 类型 | 页面含义 |
|---|---|---|
| `goal_id` | string | 与 Goal、Feedback 关联的唯一执行 ID |
| `behavior_id` | string | 上游业务 ID；用于跨 Tree/Action 日志关联 |
| `behavior_name` | string | 实际执行的行为名 |
| `status` | string | 业务终态；页面终态判断的第一依据 |
| `result` | string | 简短结果码；成功必须与 `status=SUCCESS` 联合判断 |
| `reason` | string | 面向日志或详情页的原因，不应用作程序分支键 |
| `duration_sec` | number | Action 端统计的整次执行耗时，单位秒 |
| `failed_action` | string | 失败的精确 `ACT_*`；无失败时为空字符串 |
| `interrupted_by` | string | 取消/抢占来源；没有时为空字符串 |
| `reward` | number | 当前执行结果的数值反馈 |
| `emotion_delta_json` | string | JSON 字符串；情绪变化数据 |
| `need_delta_json` | string | JSON 字符串；需求变化数据 |
| `metadata_json` | string | JSON 字符串；充电电量、导航或闭环接近等扩展结果 |
| `timestamp` | number | Result 产生时的 Unix epoch 秒 |

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

页面和 Action Server 都只使用以下 73 个行为。其他行为名会在 Goal 验证阶段
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

### 情绪表达（18）

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
expressCalmInPlaceWithHuman
expressJoyInPlaceWithHuman
expressExcitementInPlaceWithHuman
expressAnxietyInPlaceWithHuman
expressFearInPlaceWithHuman
expressCuriosityInPlaceWithHuman
```

### 主人定制视觉行为（3）

```text
unhappy
miss_owner
farewell_leave
```

这三个行为的第一个 Feedback 动作固定为 `ACT_APPROACH_VISUAL_TARGET`；视觉接近
完成后，第二个 Feedback 才分别报告下表中的定制表达动作。

### 直接指令（28）

```text
respond_owner_call
approach_voice_caller
respond_person_fall
respond_stop_gesture
sit_down
lie_down
stand_up
walk_to_random_point
go_out_to_play
go_home
approach_owner
back_up
stand_still
hold_position
quiet
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

这 28 个直接指令保持精确行为边界；`stand_still` 是本批唯一包含两个 Stage 的
新增指令：先站立，再保持站姿。

| `behavior_name` | `current_action` |
|---|---|
| `respond_owner_call` | `ACT_INTERACT_RESPOND_CALL` |
| `approach_voice_caller` | `ACT_INTERACT_APPROACH_VOICE_CALLER` |
| `respond_person_fall` | `ACT_PERCEPTION_RESPOND_PERSON_FALL` |
| `respond_stop_gesture` | `ACT_PERCEPTION_RESPOND_STOP_GESTURE` |
| `unhappy` | `ACT_APPROACH_VISUAL_TARGET` -> `ACT_OWNER_UNHAPPY` |
| `miss_owner` | `ACT_APPROACH_VISUAL_TARGET` -> `ACT_EXPRESS_MISS_YOU` |
| `farewell_leave` | `ACT_APPROACH_VISUAL_TARGET` -> `ACT_OWNER_GOING_OUT` |
| `sit_down` | `ACT_BASIC_SIT` |
| `lie_down` | `ACT_BASIC_LIE_DOWN` |
| `stand_up` | `ACT_BASIC_STAND` |
| `walk_to_random_point` | `ACT_NAV_WALK_RANDOM` |
| `go_out_to_play` | `ACT_NAV_GO_OUT_TO_PLAY` |
| `go_home` | `ACT_NAV_GO_HOME` |
| `approach_owner` | `ACT_INTERACT_APPROACH_OWNER_CLOSER` |
| `back_up` | `ACT_BASIC_BACK_UP` |
| `stand_still` | `ACT_BASIC_STAND` -> `ACT_CONTROL_HOLD_STANDING` |
| `hold_position` | `ACT_CONTROL_HOLD_POSITION` |
| `quiet` | `ACT_CONTROL_QUIET` |
| `wait_in_place` | `ACT_BASIC_WAIT` |
| `come_to_owner` | `ACT_INTERACT_APPROACH_OWNER` |
| `follow_owner` | `ACT_INTERACT_FOLLOW_OWNER` |
| `give_paw` | `ACT_INTERACT_GIVE_PAW` |
| `high_five` | `ACT_INTERACT_HIGH_FIVE` |
| `roll_over` | `ACT_TRICK_ROLL_OVER` |
| `spin_around` | `ACT_TRICK_SPIN` |
| `return_to_owner` | `ACT_INTERACT_RETURN_OWNER` |
| `drop_object` | `ACT_OBJECT_DROP` |
| `play_dead` | `ACT_TRICK_PLAY_DEAD` |
| `bring_object` | `ACT_OBJECT_BRING` |
| `fetch_object` | `ACT_OBJECT_FETCH` |
| `emergency_stop` | `ACT_SYSTEM_EMERGENCY_STOP` |

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

动作 ID 共 203 个，唯一目录为 `config/action_catalog.yaml`。其中多 Stage 行为的
每个 Stage 可能有多个候选动作，运行时按该 Stage 的 `selection_policy` 选择一个，
所以页面不能仅凭 `behavior_name` 预判最终 `current_action`。

仿真端必须记录未实现的 `ACT_*`，不能静默替换成相近动作，否则会再次产生
“行为名称正确但执行动作不一致”的问题。

当前 Feedback 在 Stage 完成后才发布。如果仿真动画需要在动作开始时同步触发，
现有接口还缺少 `unit_started` 事件；在接口扩展前，不要假设 Feedback 是动作
起始信号。

Go2/Lite3 真机可能已在 Feedback 到达前完成当前动作。平台 Topic、限速、
动作映射和急停联调见各自的 ROS2 动作集成说明。

对于带定点导航的行为，首个 Stage Feedback 会在 Nav2 到点后才出现。页面从
Goal 被接受到首个 Feedback 之间应保持“执行中/导航中”，不能因暂时没有
`current_action` 判定超时。页面 Goal 的 `timeout_sec` 也必须覆盖导航时间。

页面按精确 `ACT_*` 选择动画，不根据底盘速度反推动作。
不同底盘的真实动作由 Go2/Lite3 配置决定；代理动作不能当成真实物体操作。

`ACT_INTERACT_FOLLOW_OWNER` 不是开环代理动作。`follow_owner` 的
`timeout_sec=0` Goal 在 Go2 上持有外部 UWB 进程链，在 Lite3 上持有
`/go2/follow_uwb` 内层 Action；持续 Feedback 和真实取消 Result 都属同一 Goal。
页面只按 `follow_owner` 与 `ACT_INTERACT_FOLLOW_OWNER` 展示，无需直连 UWB。
页面直接发送有界 Goal 时仍可用 `config/action_catalog.yaml` 的 15 秒动作预算，
Goal 的 `timeout_sec` 应大于该值。直接发送 `timeout_sec=0` 的长期 Goal 时，
客户端还须按 `goal_id`/`behavior_id` 在 `/behavior/goal_lease` 续租；否则 Action
在初始 3 秒宽限后停车并返回 `FAILED`。

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
  显示实际 `ACT_*`；后续如需修正，必须先更新双方采用的新对照表。2026-09-18
  起它的 Stage 顺序为 `circle` → `action` → `exit` → `head_up`，最后一步是
  新增的 `ACT_RAISE_HEAD`（抬头，完成信号），仿真端需要认识这个新动作。

## 9. 联调检查清单

- [ ] `ros2 action info /execute_behavior` 能看到 Action Server。
- [ ] 页面生成唯一 `goal_id` 和 `behavior_id`。
- [ ] `params_json` 是字符串形式的合法 JSON。
- [ ] Goal、Feedback、Result 能按 `goal_id` 关联。
- [ ] 页面按 Result payload 而不是 transport state 判断成功。
- [ ] Feedback 按 Stage 到达，不要求 10Hz。
- [ ] `current_action` 原样匹配动画键，未知动作明确告警。
- [ ] 新页面主列表只使用第 6 节的 73 个直接行为。
- [ ] `emergency_stop` 映射为 `ACT_SYSTEM_EMERGENCY_STOP`。
- [ ] `sit_down` 映射为 `ACT_BASIC_SIT`。

## 10. 相关文件

- [ROS2 集成说明](ROS2.md)
- [Go2 ROS2 动作集成说明](GO2_ROS2_INTEGRATION.md)
- [Lite3 ROS2 动作集成说明](LITE3_ROS2_INTEGRATION.md)
- [waypoint_nav 点位适配说明](NAV2_WAYPOINT_INTEGRATION.md)
- [唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)
- [行为树动作对接契约](BEHAVIOR_TREE_ACTION_CONTRACT.md)
- [行为树运行映射](../config/behavior_tree_actions.yaml)
- [严格动作目录](../config/action_catalog.yaml)
- 正式接口：`marsdog_interfaces/action/ExecuteBehavior`
