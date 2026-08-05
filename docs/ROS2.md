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
[behavior_tree_actions.yaml](../config/behavior_tree_actions.yaml) 中的 53 个
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
| 行为数量 | 53 |
| 运行时动作数量 | 188 |
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
| `/debug/execute_behavior/feedback` | `std_msgs/msg/String` | 每个 Stage 完成后 |
| `/debug/execute_behavior/result` | `std_msgs/msg/String` | 行为结束时 |

`String.data` 是 JSON。

AGV 启用后还会在运动段期间向 `/cmd_vel` 发布
`geometry_msgs/msg/Twist`，默认频率为 10Hz；它不是 Debug Topic。

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
| `object_category` | string |
| `sleep_depth` | string |
| `elimination_type` | string |
| `charger_known` | boolean |
| `charger_available` | boolean |
| `random_seed` | integer |
| `use_wake_angle` | boolean |
| `wake_angle_deg` | number（度） |
| `wake_confidence` | number |
| `wake_frame_id` | string，当前要求 `base_link` |

`params_json` 不参与旧行为名称转换。

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

### 启用 AGV

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  agv_enabled:=true \
  agv_cmd_vel_topic:=/cmd_vel \
  agv_publish_rate_hz:=10.0
```

AGV 默认关闭。开启后，只有 `controller_routes.yaml` 中明确路由为 `agv` 的
13 个直接指令动作会发布 Twist；其他动作等待后续导航功能。完整映射、安全
边界和联调命令见
[AGV ROS2 运动适配说明](AGV_ROS2_INTEGRATION.md)。

`respond_owner_call → ACT_INTERACT_RESPOND_CALL` 使用独立的动态
`wake_orientation` 路由。`agv_enabled:=true` 时默认把行为树传入的
`wake_angle_deg` 转为 Nav2 `/spin` 的相对 yaw，不属于上述 13 个固定 Twist
运动组。字段契约、校准参数和测试命令见
[唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)。

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

### 启用 Nav2 语义点位

Nav2 适配使用正式 Action：

```text
/navigate_to_pose
nav2_msgs/action/NavigateToPose
```

启动时必须同时启用 AGV 和导航：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  agv_enabled:=true \
  navigation_enabled:=true \
  agv_cmd_vel_topic:=/cmd_vel \
  navigation_action_name:=/navigate_to_pose \
  navigation_frame_id:=map \
  navigation_server_timeout_sec:=10.0 \
  navigation_result_timeout_sec:=300.0
```

当前 A–E 点位、11 个首批行为、精确 `ACT_*` 到到点运动组映射，以及取消/急停
规则见 [Nav2 语义点位适配说明](NAV2_WAYPOINT_INTEGRATION.md)。

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

- 53 个行为；
- 211 条动作记录；
- 188 个唯一动作；
- 运行时动作目录恰好等于新表引用集合；
- 旧行为名被拒绝；
- 旧动作不进入运行时目录。

## 11. 相关文件

- [仿真页面接入说明](SIMULATION_PAGE_INTEGRATION.md)
- [AGV ROS2 运动适配说明](AGV_ROS2_INTEGRATION.md)
- [Nav2 语义点位适配说明](NAV2_WAYPOINT_INTEGRATION.md)
- [唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)
- [行为树动作契约](BEHAVIOR_TREE_ACTION_CONTRACT.md)
- [行为动作映射说明](BEHAVIOR_ACTION_MAP.md)
- [行为配置](../config/behavior_tree_actions.yaml)
- [严格动作目录](../config/action_catalog.yaml)
