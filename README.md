# MarsDog Action Executor

> 独立项目交接、Action 契约、视觉跟随、AGV 与调试说明见
> [docs/HANDOFF.md](docs/HANDOFF.md)。

仿生机器狗行为执行器。接收上游行为树下发的 `behavior_name` 和
`params_json`，按新的行为动作对照表生成 Stage，选择精确的 `ACT_*` 并执行。

## 严格行为树契约

运行时唯一行为来源是
[config/behavior_tree_actions.yaml](config/behavior_tree_actions.yaml)：

- 53 个可执行行为；
- 211 条候选动作记录；
- 188 个唯一 `ACT_*`；
- 行为名和动作名均大小写敏感；
- 不加载旧行为模板；
- 不解析旧行为 alias；
- 不向运行时动作目录暴露新表之外的动作。

例如：

```text
sit_down       -> ACT_BASIC_SIT
emergency_stop -> ACT_SYSTEM_EMERGENCY_STOP
```

`defecate`、`go_back`、`expressJoy`、`wagTailFast`、
`seek_food_or_water` 等不在新表中的旧名称会被拒绝。

## 数据流

```text
ROS2 ExecuteBehavior.Goal
  behavior_name + params_json
        │
        ▼
GoalParser
  params_json -> ExecutionContext
        │
        ▼
BehaviorResolver
  精确校验 behavior_name 是否属于 53 个直接行为
        │
        ▼
ConfigLoader
  从 behavior_tree_actions.yaml 获取有序 Stage
  校验 action_catalog.yaml 恰好包含表引用的 188 个 ACT_*
        │
        ▼
StageExecutor
  每 Stage: candidates -> eligibility -> random_one -> execute
        │
        ├─ Feedback.current_action = 精确 ACT_* ID
        └─ Result.behavior_name = 原 behavior_name
```

## ROS2 接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/execute_behavior` | `marsdog_interfaces/action/ExecuteBehavior` | 正式行为调用 |
| `/debug/execute_behavior/goal` | `std_msgs/msg/String` JSON | Goal 可视化 |
| `/debug/execute_behavior/feedback` | `std_msgs/msg/String` JSON | Stage/动作/进度 |
| `/debug/execute_behavior/result` | `std_msgs/msg/String` JSON | 终态可视化 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 可选 AGV 运动输出 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 可选语义点位导航 |
| `/spin` | `nav2_msgs/action/Spin` | 按唤醒声源角度闭环转向 |

Feedback 在每个 Stage 执行完成后发布一次，不是固定 10Hz。
AGV Twist 在运动段执行期间默认按 10Hz 发布。

真机板端的 `/agv_pro_node` 位于 Domain 1。启动行为树、动作执行器、仿真桥和
rosbridge 前必须统一设置：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

只把动作执行器放到 Domain 1 会导致 Domain 0 的行为树无法发现
`/execute_behavior`。

## 主要文件

```text
config/
├── behavior_tree_actions.yaml         # 唯一行为定义
├── action_catalog.yaml                # 新表引用的 188 个动作元数据
├── agv_motion_groups.yaml             # 直接动作和到点 Stage 的 11 套 Twist 组
├── navigation_waypoints.yaml          # A-E 点位与 11 个行为的导航/Stage 映射
├── wake_orientation.yaml              # 唤醒角度校准和 Nav2 Spin 配置
├── controller_routes.yaml
└── safety_policies.yaml

marsdog_action_executor/
├── config_loader.py       # 严格加载并校验 53 behavior / 188 action
├── behavior_resolver.py   # 精确名称校验，无 alias
├── goal_parser.py
├── execution_context.py
├── stage_executor.py
├── adapters/agv_adapter.py
├── adapters/navigation_adapter.py
├── adapters/wake_orientation_adapter.py
├── result_evaluator.py
├── ros_node.py
└── units/

docs/
├── ARCHITECTURE.md
├── ROS2.md
├── AGV_ROS2_INTEGRATION.md
├── BEHAVIOR_ACTION_MAP.md
├── BEHAVIOR_TREE_ACTION_CONTRACT.md
└── SIMULATION_PAGE_INTEGRATION.md
```

## 快速验证

```bash
uv sync
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q
```

无 ROS2 的本地演示：

```bash
uv run python -m marsdog_action_executor.standalone_demo sit_down 42
```

ROS2 编译与启动：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build \
  --packages-select marsdog_interfaces marsdog_action_executor \
  --symlink-install
source install/setup.bash
ros2 run marsdog_action_executor action_executor_node
```

启用 AGV `/cmd_vel`：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  agv_enabled:=true \
  agv_cmd_vel_topic:=/cmd_vel
```

同时启用 Nav2 A–E 语义点位：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  agv_enabled:=true \
  navigation_enabled:=true \
  agv_cmd_vel_topic:=/cmd_vel \
  navigation_action_name:=/navigate_to_pose
```

`agv_enabled:=true` 时，`respond_owner_call` 默认会读取行为树传入的
`wake_angle_deg` 并调用 `/spin`。角度标定和联调见
[唤醒声源朝向说明](docs/WAKE_ORIENTATION_INTEGRATION.md)。

发送行为：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 't1', behavior_id: 'sim-t1', \
    behavior_name: 'sit_down', priority_level: 5, \
    params_json: '{}', timeout_sec: 10.0}"
```

## 文档

- [系统架构](docs/ARCHITECTURE.md)
- [ROS2 集成说明](docs/ROS2.md)
- [AGV ROS2 运动适配说明](docs/AGV_ROS2_INTEGRATION.md)
- [Nav2 语义点位适配说明](docs/NAV2_WAYPOINT_INTEGRATION.md)
- [唤醒声源朝向说明](docs/WAKE_ORIENTATION_INTEGRATION.md)
- [行为与动作映射说明](docs/BEHAVIOR_ACTION_MAP.md)
- [行为树动作契约](docs/BEHAVIOR_TREE_ACTION_CONTRACT.md)
- [仿真页面接入说明](docs/SIMULATION_PAGE_INTEGRATION.md)
