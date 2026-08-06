# 动作系统项目交接说明

> 对接基线：2026-08-04 / 多项目契约 1.0.0

## 1. 本项目负责什么

动作系统是行为执行端：接收行为树的语义 `behavior_name`，严格解析为有序 Stage 和精确 `ACT_*`，执行仿真/控制器/AGV/Nav2，并通过 Action Feedback/Result 返回生命周期。同时提供语音会话期间的视觉居中和人体跟随闭环。

- 节点：`action_executor_node`
- 入口：`ros2 launch marsdog_action_executor action_executor.launch.py`

## 2. 对外接口

| 方向 | 接口 | 类型 | 作用 |
|---|---|---|---|
| 提供 | `/execute_behavior` | `marsdog_interfaces/action/ExecuteBehavior` | 正式业务控制 |
| 订阅 | `/behavior/attention_tracking` | String JSON, RELIABLE 10 | 会话级跟踪开关/模式 |
| 订阅 | `/perception/visual_event` | String JSON, BEST_EFFORT 5 | 闭环目标数据 |
| 发布 | `/cmd_vel` | `geometry_msgs/msg/Twist` | AGV 速度，启用时默认 10 Hz |
| 调用 | `/spin` | `nav2_msgs/action/Spin` | 可选唤醒声源转向 |
| 调用 | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 可选语义点位导航 |
| 发布 | `/debug/execute_behavior/*` | String JSON | 可视化/调试，不是业务依赖 |

正式 Action 定义只以 `/home/cat/xbb/marsdog_interfaces/action/ExecuteBehavior.action` 为准。本仓库 `action/ExecuteBehavior.action` 是兼容副本。

## 3. 行为执行契约

Action Server 只接受 `config/behavior_tree_actions.yaml` 中的精确行为名，大小写敏感，不自动 alias、不猜相近名称。Stage 的候选 `ACT_*` 来自该文件，动作元数据来自 `config/action_catalog.yaml`。

关键 Goal 字段：`goal_id`、`behavior_id`、`behavior_name`、`priority_level`、合法 JSON `params_json`、总 `timeout_sec`。

Feedback 在 Stage 完成后发布，`current_action` 是精确 `ACT_*`，`safe_to_interrupt` 供行为树执行 safe-point 抢占。

Result 的 `metadata_json` 用于动态业务结果。成功充电必须返回：

```json
{"energyValue": 88}
```

当前无 BMS 时可由 launch 参数 `recharge_result_energy_value` 模拟，默认 100。该值表示实际电量百分比。

## 4. 视觉居中与跟随

后台控制器输入：

- `/behavior/attention_tracking.enabled/mode/interaction_id`
- `/perception/visual_event.active_target`

模式：

- `face_body_centering`：只转向，不前进。
- `follow_owner`：转向并根据人体框高度前进。

当前默认参数：

| 参数 | 默认值 |
|---|---:|
| `attention_tracking_gain` | 0.8 |
| `attention_tracking_deadband` | 0.08 |
| `attention_tracking_activation_deadband` | 0.14 |
| `attention_tracking_smoothing_alpha` | 0.15 |
| `attention_tracking_max_angular_z` | 0.25 rad/s |
| `attention_tracking_max_angular_accel` | 0.25 rad/s² |
| `attention_tracking_visual_timeout_sec` | 0.8 s |
| `follow_target_height` | 0.68 |
| `follow_height_deadband` | 0.05 |
| `follow_activation_deadband` | 0.10 |
| `follow_linear_gain` | 0.75 |
| `follow_max_linear_x` | 0.25 m/s |
| `follow_max_linear_accel` | 0.35 m/s² |
| `follow_max_heading_error` | 0.30 |

上述跟随参数集中在 `config/attention_tracking.yaml`。launch 默认加载该文件，
也可以通过 `params_file:=/path/to/custom.yaml` 使用另一份 ROS 2 参数文件；
配置文件中的值优先于 launch 内置默认值。

`follow_owner` 在行为表中必须是零速度 `follow_handoff`，不能配置成前进一段再左右摆动。连续运动只由后台闭环产生。

安全条件：

- 只接受 `tracking_state=tracking` 的新鲜目标。
- 目标超时/丢失、`enabled=false`、节点取消时发布零速度。
- 水平误差太大时先旋转，暂不前进。
- 正式 Behavior 持有执行锁时暂停后台跟踪，防止两个链路同时写 `/cmd_vel`。

## 5. AGV、Nav2 与动作配置

| 文件 | 内容 |
|---|---|
| `config/behavior_tree_actions.yaml` | Behavior → Stage → `ACT_*`，唯一行为注册表 |
| `config/action_catalog.yaml` | `ACT_*` 元数据、中断策略、资源 |
| `config/controller_routes.yaml` | 动作到控制器路由 |
| `config/agv_motion_groups.yaml` | Twist 序列、速度限制、持续时间 |
| `config/navigation_waypoints.yaml` | Nav2 语义点位和行为路由 |
| `config/sound_config.yaml` | 语音指令犬叫与精确行为音频映射 |
| `config/sounds/` | 随包安装的 MP3/WAV 行为音频素材 |
| `config/wake_orientation.yaml` | 唤醒角度到 `/spin` 的转换 |
| `config/safety_policies.yaml` | 安全约束 |

启用 AGV：

```bash
source /opt/ros/humble/setup.bash
source /home/cat/ros2_ws/install/setup.bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  agv_enabled:=true \
  attention_tracking_enabled:=true
```

启用 Nav2 路由还需 `navigation_enabled:=true`，并保证 `/navigate_to_pose` 可用。唤醒转向使用 `/spin`，不使用时关闭 `wake_orientation_enabled`。

## 6. 可视化与调试

动作界面：

```bash
ros2 run marsdog_action_executor emotion_display
```

它订阅：

```text
/debug/execute_behavior/goal
/debug/execute_behavior/feedback
/debug/execute_behavior/result
```

界面使用 Qt 实时绘制暖色卡通电子狗脸，不依赖单张 PNG 才能显示；不同情绪会改变眼睛、
耳朵、视线、嘴型、颜色和动画节奏；喜悦/社交带上浮爱心，焦虑/恐惧带动态汗滴。
Feedback 同步更新 Stage、ACT 和底部进度条。业务
状态以 Action Result 为准，Debug Topic 只用于观测。界面会按 `goal_id` 忽略旧任务的
迟到 Result，并可通过 Feedback 恢复启动前已经开始的行为。程序默认全屏启动，按
`F11` 可在全屏和普通窗口之间切换，按 `Esc` 退出全屏。

跟随调试还应同时运行视觉 Viewer，观察 `track_id`、`body_center.x`、`bbox.h` 和最终 `/cmd_vel`。

## 7. 测试与验收

```bash
cd /home/cat/xbb/20260707_MarsDogAction
uv run pytest
ros2 action info /execute_behavior
ros2 topic info -v /cmd_vel
ros2 topic echo /debug/execute_behavior/feedback
```

至少回归：

- 表内行为接受，表外行为在 goal 阶段拒绝。
- 取消、超时、失败、成功的业务 Result 正确。
- 充电成功包含 `metadata_json.energyValue`。
- `follow_owner` Action 本身不产生固定移动。
- 目标丢失/会话结束后 1 秒内速度归零。
- Action 执行期间后台跟踪没有并发写速度。
- `/cmd_vel` 只有经过系统设计的发布者。

## 8. 修改接口时必须协同

- 新增 Behavior：行为树负责人提供名称/优先级/参数，动作负责人注册 Stage/ACT，并共同做 Action 冒烟测试。
- 修改 `ACT_*` 不应改变上游 Behavior 名；若必须改 Behavior 名属于跨项目 MAJOR 变更。
- 修改跟随参数不需要语音改代码，但必须保持 `mode` 语义和失效保护。
- 修改 Action 字段必须先改 `marsdog_interfaces`，再同步行为树 Client、本项目 Server、文档和联调脚本。
