# MarsDog Action Executor — ROS2 集成说明

> 严格行为树契约版本：2026-07-31。
>
> 仿真页面请同时阅读
> [SIMULATION_PAGE_INTEGRATION.md](SIMULATION_PAGE_INTEGRATION.md)。

## 1. 系统关系

```text
marsdog_interfaces
  ExecuteBehavior.action
          ▲
          │
marsdog_behavior                  marsdog_action_executor
Behavior Tree / Action Client ──► /execute_behavior Action Server
                                           │
                                           └─► ACT_* / controller / simulator
```

| 包 | 职责 |
|---|---|
| `marsdog_interfaces` | 正式 Action/Message 定义 |
| `marsdog_behavior` | 行为树，发送 Goal |
| `marsdog_action_executor` | 严格校验、Stage 选择、动作执行和结果返回 |

## 2. 严格行为注册

Action Server 只接受
[behavior_tree_actions.yaml](../config/behavior_tree_actions.yaml) 中的 73 个
直接行为名。

不支持：

- 旧 canonical 行为；
- `behavior_templates.yaml` 中的历史行为；
- `behaviors.yaml` 中的历史语音模板；
- 旧行为 alias；
- 名称自动改写；
- 新表之外的动作。

示例：

| 请求 | 结果 |
|---|---|
| `sit_down` | ACCEPT |
| `emergency_stop` | ACCEPT |
| `eatNormally` | ACCEPT |
| `defecate` | REJECT |
| `go_back` | REJECT |
| `expressJoy` | REJECT |
| `wagTailFast` | REJECT |
| `seek_food_or_water` | REJECT |

Goal 验证等价于：

```python
if goal_request.behavior_name in configured_behavior_tree_names:
    return GoalResponse.ACCEPT
return GoalResponse.REJECT
```

## 3. Action Server

| 属性 | 值 |
|---|---|
| Node | `action_executor_node` |
| Action | `/execute_behavior` |
| 类型 | `marsdog_interfaces/action/ExecuteBehavior` |
| 行为数量 | 73 |
| 运行时动作数量 | 182 |
| Feedback | 每个 Stage 完成后一次 |

执行流程：

```text
_on_goal
  精确检查 behavior_name
        │
        ▼
_on_accepted
  启动执行并发布 Debug Goal
        │
        ▼
_on_execute
  GoalParser
  -> BehaviorResolver exact validation
  -> ConfigLoader exact template
  -> for Stage:
       cancel / timeout
       candidate filter
       random_one
       unit execute
       Feedback
  -> ResultEvaluator
  -> Action Result + Debug Result
```

## 4. ExecuteBehavior.action

正式定义来自 `marsdog_interfaces`。

### Goal

```text
string  goal_id
string  behavior_id
string  behavior_name
int32   priority_level
string  params_json
float64 timeout_sec
```

### Result

```text
string  goal_id
string  behavior_id
string  behavior_name
string  status
string  result
string  reason
float64 reward
string  emotion_delta_json
string  need_delta_json
string  metadata_json
```

### Feedback

```text
string  goal_id
string  behavior_id
string  behavior_name
string  status
float64 progress
bool    safe_to_interrupt
string  current_action
string  message
```

约束：

- `behavior_name` 大小写敏感；
- `params_json` 必须是合法 JSON 字符串；
- `current_action` 是新表中的精确 `ACT_*`；
- Feedback 在 Stage 完成后发布，不是固定 10Hz；
- 正式 Feedback 不含 `current_stage`。
- `metadata_json` 用于充电实际电量等动态业务结果，例如
  `{"energyValue":88}`。

## 5. Debug Topic

| Topic | 类型 | 时机 |
|---|---|---|
| `/debug/execute_behavior/goal` | `std_msgs/msg/String` | Goal 接受并启动时 |
| `/debug/execute_behavior/feedback` | `std_msgs/msg/String` | 每个 Stage 完成后；长期 Goal 运行期间每 0.5 秒持续发布阶段、周期和安全中断状态 |
| `/debug/execute_behavior/result` | `std_msgs/msg/String` | 行为结束时 |

`String.data` 是 JSON。

Lite3 启用后还会在运动段期间向 `/cmd_vel` 发布
`geometry_msgs/msg/Twist`；Go2 启用后向 `/api/sport/request` 发布
`unitree_api/msg/Request`。两者默认运动刷新频率均为 10Hz，且都不是 Debug
Topic。

### Goal JSON

```json
{
  "goal_id": "sim-001",
  "behavior_id": "ui-001",
  "behavior_name": "sit_down",
  "priority_level": 5,
  "params": {},
  "timeout_sec": 10.0,
  "timestamp": 1785312000.123
}
```

### Feedback JSON

```json
{
  "goal_id": "sim-001",
  "behavior_id": "ui-001",
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

### Result JSON

```json
{
  "goal_id": "sim-001",
  "behavior_id": "ui-001",
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

Debug Topic 用于调试和当前仿真可视化。业务控制以 Action 为准。

## 6. params_json

无参数：

```json
{}
```

带目标：

```json
{
  "source": "behavior_tree",
  "target": {
    "target_type": "human",
    "identity": "owner",
    "visible": true,
    "distance_m": 1.2
  },
  "random_seed": 42
}
```

常用字段：

| 字段 | 类型 |
|---|---|
| `source` | string |
| `priority_level` | integer |
| `target` | object |
| `trigger_event` / `specific_event_type` | string，精确上游事件 |
| `command_key` / `command_id` | string，语音词库键与稳定指令 ID |
| `command_catalog_version` | string，语音词库版本 |
| `intent_source` / `dispatch_role` | string，识别来源与分发角色 |
| `voice_slots` | object，行为树审核后的语音槽位 |
| `object_category` | string |
| `sleep_depth` | string |
| `elimination_type` | string |
| `charger_known` | boolean |
| `charger_available` | boolean |
| `random_seed` | integer |
| `use_wake_angle` | boolean |
| `wake_angle_deg` | number（度） |
| `wake_confidence` | number |
| `wake_frame_id` | string，当前要求原始输入帧 `microphone_array` |

`params_json` 不参与旧行为名称转换，也不能覆盖 Goal 顶层的
`behavior_name`。语音身份字段只用于追踪和动作参数化；具体 Stage 与 `ACT_*` 仍由
Action 的行为配置决定。

### `approach_voice_caller` 强契约

该行为精确映射为
`ACT_INTERACT_APPROACH_VOICE_CALLER -> person_nav_approach`。除 `target` 外，
`params_json` 必须包含：

| 字段 | 约束 |
|---|---|
| `interaction_id` | 非空，必须与语音会话一致 |
| `wake_id` | 非空，与当前唤醒声纹事件关联 |
| `speaker_role` / `speaker_id` | `owner/owner` 或 `family/family_member_1..4` |
| `speaker_status` | 必须为 `matched` |
| `strict_target_lock` | 必须为 `true` |
| `allow_target_switch` | 必须为 `false` |
| `target.target_type` | 必须显式为 `human` |
| `target.vision_epoch` / `target.target_id` | 目标 ID 必须以 `<vision_epoch>:human:` 开头 |
| `stand_off_distance_m` | 有限且不少于 1.5 m |

Action 向 `/perception/vision/task` 发一次 `locate_person_once`，由 Vision
对目标框和原图 Header 做来源绑定，再调用 SLAM。只有响应 `ok=true`、
`status=0`、目标 ID 一致、人体点有效且 `navigation_required=true` 时，
Action 才向 `/navigate_to_pose` 发送一次地图目标；`false` 时直接完成。
Lite3 在 Nav2 前执行底盘导航预检，Nav2 终态后确认退出遥控并停稳。
取消受理不等于完成；Action 等 Nav2 真实 Result 后返回 CANCELED。
内层终态未知时返回 FAILED 并锁定恢复状态，拒绝后续非急停 Goal。

### 社交/动物/物体的先接近后动作

13 个有明确视觉目标的 Social / Exploration 行为、六个普通
`express*WithHuman`，以及三个主人定制行为，都会先执行
`ACT_APPROACH_VISUAL_TARGET`。`target.target_type` 必须与行为策略一致，
`target.target_id` 必须能唯一绑定到 `/perception/visual_event` 的
`human_candidates`、`active_target` 或 `tracked_objects` 中同一稳定轨迹。
上游带 `vision_epoch` 时必须完全一致；未带时仅允许绑定当前最新有效 epoch。

人体社交目标使用锁定人体框高度闭环；动物和物体使用
`range_valid=true` 的 `distance_m`。行为停止条件和速度上限在
`config/visual_target_approach.yaml` 中配置。接近失败、取消或
超时后，第 2 个互动/检查 Stage 不执行，Result 的 `metadata_json` 使用
`visual_target_approach` 返回绑定目标、有效策略和终态原因。

三个定制行为的精确表达动作分别是 `unhappy -> ACT_OWNER_UNHAPPY`、
`miss_owner -> ACT_EXPRESS_MISS_YOU`、`farewell_leave -> ACT_OWNER_GOING_OUT`。
其中 `farewell_leave` 的 3 秒到达保持会持续观察同一目标，主人走远时恢复受限
接近；换人或目标丢失不会继续盲走。
三者均要求 `target.identity=owner`；身份缺失或不匹配会在运动前失败。

## 7. 发送 Goal

坐下：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'sim-001', behavior_id: 'ui-001', \
    behavior_name: 'sit_down', priority_level: 5, \
    params_json: '{}', timeout_sec: 10.0}"
```

紧急停止：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'sim-stop-001', behavior_id: 'ui-stop-001', \
    behavior_name: 'emergency_stop', priority_level: 6, \
    params_json: '{}', timeout_sec: 2.0}"
```

监听 Debug Topic：

```bash
ros2 topic echo /debug/execute_behavior/goal
ros2 topic echo /debug/execute_behavior/feedback
ros2 topic echo /debug/execute_behavior/result
```

验证严格拒绝：

```bash
ros2 action send_goal \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'old-001', behavior_id: 'old-001', \
    behavior_name: 'wagTailFast', priority_level: 5, \
    params_json: '{}', timeout_sec: 10.0}"
```

该 Goal 应在 `_on_goal` 阶段被 REJECT。

## 8. 取消和终态

`_on_cancel` 接受取消请求，`InterruptManager` 根据当前动作的策略处理：

| 策略 | 行为 |
|---|---|
| `immediate` | 立即停止 |
| `safe_point` | 到安全点停止 |
| `non_interruptible` | 当前动作完成后停止 |

仿真页面必须读取 Result payload 的 `status` 和 `result` 判断业务终态，不要只
读取 ROS2 Action transport terminal state。

## 9. 编译和启动

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws

colcon build \
  --packages-select marsdog_interfaces marsdog_action_executor \
  --symlink-install

source install/setup.bash
ros2 run marsdog_action_executor action_executor_node
```

或：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py
```

所有 `config/*.yaml` 会安装到包 share 目录。

### 启用 Go2

先 source Unitree ROS2 工作空间，再互斥选择 Go2 后端：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=go2 \
  go2_enabled:=true \
  go2_request_topic:=/api/sport/request
```

Go2 只覆盖 `go2_sport.yaml` 中有实现的动作；未实现动作不会模拟成功。
接口、动作表、视觉安全边界和真机验收见
[Go2 ROS2 动作集成说明](GO2_ROS2_INTEGRATION.md)。

`respond_owner_call → ACT_INTERACT_RESPOND_CALL` 使用独立的动态
`wake_orientation` 路由。所选底盘启用时，默认把行为树传入的
`microphone_array` 原始 `wake_angle_deg` 在 Action 内按 offset/sign 标定一次，
转为 `base_link` 相对 yaw 后发送 Nav2 `/spin`，不属于上述固定 Twist
运动组。字段契约、校准参数和测试命令见
[唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)。

`approach_voice_caller` 不是固定运动组。它用 Vision + SLAM 取得一次地图目标，
再按需交给 Nav2 避障导航，不做实时人体跟随。Action Result 的
`metadata_json.target_approach` 回传阶段、原因和是否导航。外层
`timeout_sec` 与内部服务、导航超时均保持有限。

Social / Exploration 的视觉目标接近同样不是固定运动组或 Nav2 点位。它通过
`visual_target_approach` 路由直接闭环发布受限 Twist；只有到达并确认停车后才
执行原行为动作。详细链路见
[视觉目标接近说明](VISUAL_TARGET_APPROACH.md)。

### 充电完成结果

`recharge` / `restInPlace` 只有在行为执行成功时才在 Action Result 的
`metadata_json` 中回传 `energyValue`。尚未接入 BMS 时可通过 launch 参数指定
结算电量，默认按协议使用 `100`：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  recharge_result_energy_value:=88
```

失败、取消或超时结果不会携带成功充电电量；ROS Action 终态也会分别使用
`abort`、`canceled`，不再一律标记为 succeed。

### 启用 waypoint_nav 语义点位

YAML 精确地点名路由及保留随机目标 `K` 复用正式 Service 和状态 Topic：

```text
/waypoint_nav/task   marsdog_voice_interaction/srv/VoiceTask
/waypoint_nav/status std_msgs/msg/String JSON
```

随机行为不再由 Action 生成 Pose 或在固定点中抽选；每次调用
`/waypoint_nav/task` 并发送 `task_type="goto_place"`、`place="K"`，由
`waypoint_nav` 使用实时 `/map` 选点。

启动时必须同时启用所选底盘和导航。Lite3 示例：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=lite3 \
  lite3_enabled:=true \
  navigation_enabled:=true \
  waypoint_nav_service_name:=/waypoint_nav/task \
  waypoint_nav_status_topic:=/waypoint_nav/status \
  waypoint_nav_cancel_confirmation_timeout_sec:=5.0 \
  navigation_preempt_lock_wait_sec:=8.0 \
  navigation_result_timeout_sec:=300.0
```

当前 YAML 地点名、11 个首批行为、精确 `ACT_*` 到平台动作映射，以及取消/急停
规则见 [waypoint_nav 点位适配说明](NAV2_WAYPOINT_INTEGRATION.md)。

## 10. 验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
UV_CACHE_DIR=/tmp/marsdog-uv-cache \
uv run pytest -q
```

严格契约测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
UV_CACHE_DIR=/tmp/marsdog-uv-cache \
uv run pytest -q tests/test_behavior_tree_action_contract.py
```

测试覆盖：

- 73 个行为；
- 269 条动作记录；
- 182 个唯一动作；
- 运行时动作目录恰好等于新表引用集合；
- 旧行为名被拒绝；
- 旧动作不进入运行时目录。

## 11. 相关文件

- [仿真页面接入说明](SIMULATION_PAGE_INTEGRATION.md)
- [Go2 ROS2 动作集成说明](GO2_ROS2_INTEGRATION.md)
- [Lite3 ROS2 动作集成说明](LITE3_ROS2_INTEGRATION.md)
- [Go2 ROS2 动作集成说明](GO2_ROS2_INTEGRATION.md)
- [waypoint_nav 点位适配说明](NAV2_WAYPOINT_INTEGRATION.md)
- [唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)
- [视觉目标接近说明](VISUAL_TARGET_APPROACH.md)
- [行为树动作契约](BEHAVIOR_TREE_ACTION_CONTRACT.md)
- [行为动作映射说明](BEHAVIOR_ACTION_MAP.md)
- [行为配置](../config/behavior_tree_actions.yaml)
- [严格动作目录](../config/action_catalog.yaml)
