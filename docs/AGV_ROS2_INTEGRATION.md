# AGV ROS2 运动适配层联调说明

> 适用于差速或全向小车底盘。默认输出 Topic 为
> `geometry_msgs/msg/Twist /cmd_vel`。

## 1. 联动链路

```text
仿真页面 / 行为树
  ExecuteBehavior.Goal(behavior_name)
              │
              ▼
behavior_tree_actions.yaml
  behavior_name -> Stage -> exact ACT_*
              │
              ▼
controller_routes.yaml
  ACT_* -> agv
              │
              ▼
agv_motion_groups.yaml
  ACT_* -> motion_group -> Twist segments
              │ 10 Hz
              ▼
/cmd_vel geometry_msgs/msg/Twist
              │
              ▼
AGV base controller
```

行为树名称和 `ACT_*` 仍严格使用新对照表。AGV 层不增加别名，也不把未知动作
猜测成相近运动。

## 2. ROS Domain 与 RMW

动作执行器、行为树、仿真桥/rosbridge 和底盘驱动必须处于同一个
`ROS_DOMAIN_ID`，并使用兼容的 RMW。当前板端底盘节点 `/agv_pro_node`
运行在 Domain 1、`rmw_fastrtps_cpp`，所以启动整套 MarsDog 进程前统一执行：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

不能只给动作执行器设置 Domain 1：如果行为树仍在 Domain 0，它将无法发现
`/execute_behavior`。所有相关进程都应从上述环境启动。

验证：

```bash
ros2 node list
ros2 topic info /cmd_vel -v
```

同一终端应同时看到 `/action_executor_node`、`/behavior_tree_node` 和
`/agv_pro_node`，且 `/cmd_vel` 的 `Subscription count` 至少为 `1`。

## 3. 启用

AGV 默认关闭，防止节点启动后意外驱动车体。

通过 launch 启用：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  agv_enabled:=true \
  agv_cmd_vel_topic:=/cmd_vel \
  agv_publish_rate_hz:=10.0
```

通过 `ros2 run` 启用：

```bash
ros2 run marsdog_action_executor action_executor_node --ros-args \
  -p agv_enabled:=true \
  -p agv_cmd_vel_topic:=/cmd_vel \
  -p agv_publish_rate_hz:=10.0
```

参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `agv_enabled` | `false` | 是否真实发布 Twist |
| `agv_cmd_vel_topic` | `/cmd_vel` | 底盘速度 Topic |
| `agv_publish_rate_hz` | `10.0` | 运动段发布频率 |
| `agv_max_linear_x` | `0.25` | `linear.x` 绝对值上限，m/s |
| `agv_max_linear_y` | `0.0` | `linear.y` 绝对值上限，m/s |
| `agv_max_angular_z` | `1.2` | `angular.z` 绝对值上限，rad/s |

## 4. 与现有命令的关系

手工命令：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, \
    angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

AGV 适配器做的是同一件事，但由动作名称选择速度段，并自动完成：

- 以配置频率重复发布；
- 限制最大线速度和角速度；
- 多段动作按顺序执行；
- 每组结束连续发布至少 3 次零 Twist；
- 取消、`emergency_stop` 和正常退出且 ROS context 有效时发布零 Twist。

## 5. 当前映射范围

| 行为 | 精确动作 | AGV 运动组 |
|---|---|---|
| `sit_down` | `ACT_BASIC_SIT` | `sit_gesture`：左 30°、右 30°回正 |
| `lie_down` | `ACT_BASIC_LIE_DOWN` | `stop` |
| `stand_up` | `ACT_BASIC_STAND` | `stop` |
| `wait_in_place` | `ACT_BASIC_WAIT` | `wait_gesture`：右 90°、左 90°回正 |
| `come_to_owner` | `ACT_INTERACT_APPROACH_OWNER` | `forward` |
| `follow_owner` | `ACT_INTERACT_FOLLOW_OWNER` | `forward` |
| `give_paw` | `ACT_INTERACT_GIVE_PAW` | `wiggle` 替代动作 |
| `high_five` | `ACT_INTERACT_HIGH_FIVE` | `nudge` 替代动作 |
| `roll_over` | `ACT_TRICK_ROLL_OVER` | `roll_over_proxy` |
| `spin_around` | `ACT_TRICK_SPIN` | `spin_360` |
| `return_to_owner` | `ACT_INTERACT_RETURN_OWNER` | `return_owner_proxy`：掉头 180°后前进 |
| `play_dead` | `ACT_TRICK_PLAY_DEAD` | `play_dead_spin`：顺时针原地 360° |
| `emergency_stop` | `ACT_SYSTEM_EMERGENCY_STOP` | `stop` |

完整映射位于
[agv_motion_groups.yaml](../config/agv_motion_groups.yaml)，当前仅包含以上 13 个
精确动作，并额外提供到点 Stage 使用的 `retreat`，共 11 套运动组。

轮式 AGV 无法真实完成机器狗的侧翻动作，因此 `roll_over_proxy` 使用
左转—右转—左转的原地摆动组合表达“翻滚”。这只是底盘互动替代动作。
`give_paw` 和 `high_five` 同样使用底盘摆动/前后轻触表达。

角度使用 `角速度 × 持续时间` 的开环换算：

- 30°：`1.0 rad/s × 0.524s`；
- 90°：`1.0 rad/s × 1.571s`；
- 180°：`1.2 rad/s × 2.618s`；
- 360°：`1.2 rad/s × 5.236s`。

`sit_gesture` 和 `wait_gesture` 的“摆正”表示以反向等角度旋转恢复初始朝向，
随后发布零 Twist；它不是独立的转向舵机位置命令。实际角度受轮胎打滑、电机
响应和地面影响，真机应根据实测微调持续时间。

`return_owner_proxy` 在没有定位和主人目标坐标时只能表达“掉头返回”：先原地
左转约 180°，再以 `0.2m/s` 前进 `1.2s`。接入导航后应替换为目标点规划，
不能继续依赖这一开环代理动作。

除以上 13 个直接动作外，只有
`navigation_waypoints.yaml.action_motion_groups` 明确列出的动作会在 Nav2
成功到点后发布 `/cmd_vel`；其余动作仍不会驱动小车。当前 A–E 定点导航和到点
Stage 代理见 [Nav2 语义点位适配说明](NAV2_WAYPOINT_INTEGRATION.md)。

`ACT_INTERACT_RESPOND_CALL` 不使用固定 Twist 运动组。它走
`wake_orientation` 路由，根据每次唤醒事件的 `wake_angle_deg` 调用 Nav2
`/spin` 闭环旋转，详见
[唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)。

## 6. 联调命令

先确认 Topic：

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel
```

动作执行器会在每个运动段切换时打印实际 Twist 和
`matched_subscribers`。非零命令对应的值应至少为 `1`。如果为 `0`，检查
执行器和底盘驱动的 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、Topic 名称及
底盘驱动是否已经启动。

向前：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'agv-forward-001', behavior_id: 'agv-forward-001', \
    behavior_name: 'come_to_owner', priority_level: 5, \
    params_json: '{}', timeout_sec: 10.0}"
```

原地旋转：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'agv-spin-001', behavior_id: 'agv-spin-001', \
    behavior_name: 'spin_around', priority_level: 5, \
    params_json: '{}', timeout_sec: 10.0}"
```

翻滚替代运动：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'agv-roll-001', behavior_id: 'agv-roll-001', \
    behavior_name: 'roll_over', priority_level: 5, \
    params_json: '{}', timeout_sec: 10.0}"
```

紧急停止：

```bash
ros2 action send_goal \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'agv-stop-001', behavior_id: 'agv-stop-001', \
    behavior_name: 'emergency_stop', priority_level: 6, \
    params_json: '{}', timeout_sec: 2.0}"
```

## 7. 安全边界

- 当前运动是按时间执行的开环速度控制，不读取里程计，也不保证精确距离或角度。
- 首次真机联调应架空车轮或清空测试区域，并准备底盘硬件急停。
- `stand_up` 和 `lie_down` 当前映射为停车，执行它们时小车不动是预期行为。
- `return_owner_proxy` 包含真实前进段；首次测试必须确认掉头后的前方区域安全。
- 底盘必须配置 `/cmd_vel` 超时看门狗：在短时间收不到新速度命令时自动停车。
  进程被强制结束或 ROS context 已失效时，软件可能无法再发布最后一条零速度。
- 不要让 Nav2、遥控器和本节点同时直接发布同一个 `/cmd_vel`。如果存在多个
  速度源，应使用 velocity mux，并将本节点输出改为独立 Topic，例如
  `/marsdog_action/cmd_vel`。
- 障碍物检测、路径规划和目标跟随不应仅靠本适配器完成；这类动作应接入导航
  控制器，而不是继续增加开环时长。
- `linear.y` 默认限制为 0，适用于差速底盘。只有确认底盘支持横移后才能放开。

## 8. 修改运动组

修改一个 AGV 动作需要同步：

1. 在 `agv_motion_groups.yaml` 定义或调整 `motion_groups`；
2. 在同文件的 `action_motion_groups` 绑定精确 `ACT_*`；
3. 在 `controller_routes.yaml` 将该动作路由为 `agv`；
4. 运行 `pytest`，配置缺失、表外动作和路由不一致会导致校验失败。
# 语音会话注视追踪

动作节点订阅 `/behavior/attention_tracking` 和
`/perception/visual_event`。普通会话模式仅发布 `angular.z`，将实时人脸或人体
保持在画面中心；收到 `follow_owner` 模式后，还会根据人体框高度连续控制
`linear.x`。目标偏离中心过大时先转向、人体框达到目标高度时停车、视觉数据
超时或会话结束时立即停车。`ACT_INTERACT_FOLLOW_OWNER` 本身只做零速度交接，
不再执行固定的“前进并左右摆动”动作。
