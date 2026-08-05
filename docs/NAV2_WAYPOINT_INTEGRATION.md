# Nav2 语义点位与行为动作序组适配说明

## 1. 目标

在不修改 53 个行为名称和 188 个精确 `ACT_*` 的前提下，将已有 A–E 定点
导航接入行为执行链：

```text
ExecuteBehavior.Goal
  behavior_name
      │
      ▼
navigation_waypoints.yaml
  behavior_name -> semantic waypoint
      │
      ▼
/navigate_to_pose (nav2_msgs/action/NavigateToPose)
      │ Nav2 succeeded
      ▼
原 behavior_tree_actions.yaml Stage 序列
      │ selected exact ACT_* remains unchanged
      ▼
exact ACT_* -> AGV Twist motion group -> /cmd_vel
```

导航失败、超时或取消时，不会继续执行到点后的 Stage 运动。

## 2. A–E 点位

坐标来自板端现有 `waypoints.yaml` 测试脚本：

| 点位 | 语义 | x | y | orientation z | orientation w |
|---|---|---:|---:|---:|---:|
| A | 睡觉/休息 | -1.085 | 3.348 | 0.951 | 0.309 |
| B | 清洁 | -2.395 | 2.572 | -0.844 | 0.537 |
| C | 吃饭/找食/查看食物 | -2.549 | -0.131 | -0.407 | 0.913 |
| D | 排泄点 | -1.382 | -1.121 | 0.465 | 0.886 |
| E | 充电点 | -0.133 | 0.695 | 0.473 | 0.881 |

唯一配置入口是
[`config/navigation_waypoints.yaml`](../config/navigation_waypoints.yaml)。
现场重新标点时只修改这里，不要把坐标写进 Python。

执行器不会自动发布 `/initialpose`。AMCL 初始位姿应在整机启动或定位恢复流程中
设置，不能在每次行为开始时重置。

## 3. 首批行为路由

| 行为 | 点位 | 启用物理代理的原 Stage |
|---|---|---|
| `sleepOnSide` | A | `prepare`, `sleep_pose`, `sleeping`, `wakeup` |
| `sleepNow` | A | 同上 |
| `restInPlace` | A | `recover` |
| `lickPaws` | B | `groom` |
| `eatNormally` | C | `prepare`, `eating`, `interaction`, `exit` |
| `eatExcitedly` | C | `prepare`, `eating`, `exit` |
| `seekFood` | C | `seek_food` |
| `seekFoodUrgently` | C | `seek_food` |
| `inspectDogFood` | C | `inspect` |
| `barkShortAlert` | D | `prepare`, `action`, `exit` |
| `recharge` | E | `recharge` |

`barkShortAlert` 名称与排泄动作语义不一致是上游新对照表的现状；本项目仍严格
按该表中的 `ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT` 等动作，将它路由到 D 点。

Stage 仍然从原候选列表选择精确 `ACT_*`，随后按该动作 ID 查找运动组。例如：

| 精确动作 | 小车代理 |
|---|---|
| `ACT_CIRCLE_AROUND` | `spin_360` |
| `ACT_YAWN` | `sit_gesture` |
| `ACT_SLEEP_ON_SIDE` | `stop` |
| `ACT_FLIP_BODY` | `roll_over_proxy` |
| `ACT_LICK_FOOD` | `wiggle` |
| `ACT_CHEW_OR_CARRY_FOOD` | `nudge` |
| `ACT_WALK_AWAY_OR_LIE_DOWN` | `retreat` |
| `ACT_SHAKE_OFF_WATER` | `wiggle` |
| `ACT_SQUAT_AND_ELIMINATE` | `stop` |

完整精确动作映射位于 `navigation_waypoints.yaml` 的
`action_motion_groups`。Feedback 和 Result 中的动作 ID 不会被运动组名替换。
配置校验要求每个已路由 Stage 的所有候选 `ACT_*` 都有映射，防止随机选中未
适配动作后静默退回 mock。

## 4. 启动条件

所有进程必须位于与底盘相同的 Domain 1：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

启用前确认：

```bash
ros2 action info /navigate_to_pose
ros2 node list
ros2 topic info /cmd_vel -v
```

应看到 Nav2 `NavigateToPose` Action Server、`/agv_pro_node`，并且定位和地图
已经就绪。停止 `teleop_twist_keyboard` 和手工 `ros2 topic pub`，避免多个
速度源抢占 `/cmd_vel`。

通过 launch 启动：

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

两个开关必须同时为 `true`。导航开启而 AGV Twist 关闭时，节点会拒绝启动，
避免出现“导航成功但到点动作静默不执行”。

## 5. 联调示例

睡觉：导航到 A，然后执行原睡眠 Stage 序组：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'nav-sleep-001', behavior_id: 'nav-sleep-001', \
    behavior_name: 'sleepNow', priority_level: 1, \
    params_json: '{}', timeout_sec: 120.0}"
```

普通吃饭：导航到 C，然后执行进食序组：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'nav-eat-001', behavior_id: 'nav-eat-001', \
    behavior_name: 'eatNormally', priority_level: 1, \
    params_json: '{}', timeout_sec: 120.0}"
```

日志应依次出现：

```text
Navigation: sleepNow -> waypoint A
Nav2 goal A: ...
Nav2 feedback: remaining=...
Nav2 goal A succeeded
Stage 1/4: prepare
AGV Twist -> /cmd_vel: ...
```

Goal 的 `timeout_sec` 同时限制导航等待，必须覆盖现场最长导航时间；否则执行器
会取消 Nav2 Goal，并返回 `navigation_failed` 或 `navigation_canceled`。

## 6. 取消、急停和速度源边界

- 普通 Action Cancel 会取消活动 Nav2 Goal，并发送零 Twist。
- `emergency_stop` 会取消 Nav2 Goal并立即发布冗余零 Twist。
- Nav2 运行期间不执行 Stage Twist；只有 Nav2 返回成功后才进入开环运动组。
- 当前是顺序控制，不实现 velocity mux。后续如果遥控、Nav2 和动作层需要并行
  保留，应引入 `twist_mux`，分别使用独立输入 Topic。
- E 点只导航到预充电位并停车，不包含最后厘米级自动对桩；对桩应由充电控制器
  单独完成。

## 7. 仿真页面

页面仍只发送 `/execute_behavior` Goal，不直接发送 `/goal_pose` 或
`/cmd_vel`。导航阶段暂不伪造 `current_action`，所以在首个 Stage Feedback
到达前，页面应保持 Goal 为“执行中/导航中”。到点后，Feedback 继续使用行为树
选中的精确 `ACT_*`。

如果页面需要实时距离，可另外订阅 Nav2 Feedback 或后续新增独立的导航状态
Topic；不能把 waypoint 名称伪装成 `ACT_*`。
