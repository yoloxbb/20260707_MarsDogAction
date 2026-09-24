# 动作系统项目交接说明

> 对接基线：2026-08-13 / 多项目契约 1.1.0

## 1. 本项目负责什么

动作系统是行为执行端：接收行为树的语义 `behavior_name`，严格解析为有序 Stage 和精确 `ACT_*`，执行 Go2/Lite3 控制器与导航，并通过 Action Feedback/Result 返回生命周期。语音会话视觉居中走后台通道；UWB 跟随由长期 Action Goal 持有。

- 节点：`action_executor_node`
- 入口：`ros2 launch marsdog_action_executor action_executor.launch.py`

## 2. 对外接口

| 方向 | 接口 | 类型 | 作用 |
|---|---|---|---|
| 提供 | `/execute_behavior` | `marsdog_interfaces/action/ExecuteBehavior` | 正式业务控制 |
| 订阅 | `/behavior/attention_tracking` | String JSON, RELIABLE 10 | 会话级跟踪开关/模式 |
| 订阅 | `/behavior/goal_lease` | String JSON, RELIABLE 10 | 长期 Goal 按 `goal_id`/`behavior_id` 续租 |
| 订阅 | `/perception/visual_event` | String JSON, BEST_EFFORT 5 | 闭环目标数据 |
| 发布 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Lite3 速度输出及外部 UWB 链路 |
| 调用 | `/spin` | `nav2_msgs/action/Spin` | 可选唤醒声源转向 |
| 调用 | `/waypoint_nav/task` | `marsdog_voice_interaction/srv/VoiceTask` | YAML 精确地点名及保留随机目标 `K` |
| 订阅 | `/waypoint_nav/status` | String JSON | 点位运行态/终态 |
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

## 4. 视觉居中、UWB 跟随与语音呼叫者接近

后台控制器输入：

- `/behavior/attention_tracking.enabled/mode/interaction_id`
- `/perception/visual_event.active_target`（仅普通居中和目标接近使用）

模式：

- `face_body_centering`：只转向，不前进。
- `follow_owner`：由 `/execute_behavior` 长期 Goal 控制；Go2 使用外接 UWB 进程链，Lite3 使用 `go2_uwb_behavior` Action/Service。

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

普通视觉居中参数仍集中在 `config/attention_tracking.yaml`。旧的 `follow_*`
参数为启动兼容保留，但 UWB 模式不会读取人体框或产生视觉跟随速度。

UWB 配置集中在 `config/uwb_follow.yaml`。当前 Go2 运行路径启动外接 FTDI
UWB AOA 与 `go2_uwb_local_follow` 进程；Lite3 通过 Action/Service 控制
`go2_uwb_behavior`。Go2 可按下列外部管线核对依赖：

```bash
ros2 run uwb_aoa_pkg libAoa_robot_example \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AP2315SD-if00-port0
ros2 launch go2_uwb_local_follow local_follow.launch.py \
  enable_motion:=true cmd_vel_topic:=/cmd_vel
```

**Lite3 不使用上面这条管线。** 它由操作员单独启动 `go2_uwb_behavior` 的
`behavior_follow_roam.launch.py`，本执行器只发 Action Goal `/go2/follow_uwb`，
并用 Service `/go2/set_behavior`（`IDLE=0` / `STOP=2`）解锁和急停。
**不要**在 Lite3 上启动 `local_follow.launch.py`、`uwb_follow_only.launch.py`
或 `local_velocity_planner.launch.py`——它们都发 `/cmd_vel`，会和那条链打架。
那条链在跑时，Lite3 的 twist 类动作会以 `lite3_control_conflict` 响亮失败。
完整说明见 [UWB_FOLLOW_ROS2_INTERFACE.md](UWB_FOLLOW_ROS2_INTERFACE.md)。

`follow_owner` 是一个长期 `/execute_behavior` Goal（`timeout_sec=0`），不再由
`/behavior/attention_tracking` 开启。Tree 每次 tick 在 `/behavior/goal_lease`
按 `goal_id`/`behavior_id` 续租；语音 idle 只关闭会话级视觉居中，不停车。
Action 启动一次 UWB，持续发布 Feedback。取消受理仅是 `CANCEL_REQUESTED`；
Lite3 等内层 `/go2/follow_uwb` 真实 Result，Go2 确认进程退出和底盘停车，
然后才返回外层 `CANCELED`。目标丢失、控制器失败或租约过期返回 `FAILED`。
旧 Goal 未终结时拒绝替代 Goal；抢占后默认不自动跟随。详见
[长期 Goal 契约](UWB_FOLLOW_ROS2_INTERFACE.md)。

有界旧 Goal 仍受动作单元的 15 秒预算限制。长期 Goal 只有外层无截止时间，
内部反馈、目标新鲜度、漫游、姿态和停车确认仍有有限时限。

Go2 使用外部管线时，底盘驱动、RealSense 双目输入和关闭主动红外仍是
前置条件，且不应手工重复启动。

一次唤醒接近使用独立的严格链路：

```text
approach_voice_caller
  -> person_nav_approach
  -> ACT_INTERACT_APPROACH_VOICE_CALLER
  -> VisionTask.locate_person_once(target_id, stand_off_distance)
  -> navigation_required=true: Nav2 /navigate_to_pose 一次固定地图目标
  -> navigation_required=false: 已在安全距离，直接完成
```

Goal 必须携带同一语音会话的 `interaction_id`、
`wake_id`、已匹配的 `speaker_role` / `speaker_id`、
`strict_target_lock=true`、`allow_target_switch=false`，以及：

```json
{
  "interaction_id": "voice-session-1",
  "wake_id": "wake-1",
  "speaker_role": "owner",
  "speaker_id": "owner",
  "speaker_status": "matched",
  "strict_target_lock": true,
  "allow_target_switch": false,
  "target": {
    "target_type": "human",
    "vision_epoch": "vision-boot-uuid",
    "target_id": "vision-boot-uuid:human:17"
  },
  "stand_off_distance_m": 1.5
}
```

Vision 负责 bbox 原图来源校验和一次 SLAM 定位。Action 必须检查响应目标 ID、
`status=0`、人体点和地图目标；定位失败、目标失鲜、Nav2 拒绝或导航超时均不
回退到 bbox 或直接 `/cmd_vel`。`metadata_json.target_approach` 回传终态、
失败原因与 `navigation_required`。取消等待 Nav2 真实 Result；终态未知时进入
恢复锁并拒绝后续非急停 Goal。

安全条件：

- Vision 只对当前有效、带原图来源的目标执行单次定位。
- Nav2 接管避障；目标是定位时的固定地图位姿，不提供连续追踪。
- Lite3 导航前进入 Vision Mode，终态后退出遥控并确认底盘停稳。
- 正式 Behavior 持有执行锁时暂停后台跟踪，防止两个链路同时写 `/cmd_vel`。
- 取消/急停请求 Nav2 取消并停车；内层真实 Result 或恢复锁决定何时可接收替代 Goal。

视觉安全事件使用两个独立正式行为：

```text
respond_person_fall
  -> ACT_PERCEPTION_RESPOND_PERSON_FALL -> Go2/Lite3 -> stop

respond_stop_gesture
  -> ACT_PERCEPTION_RESPOND_STOP_GESTURE -> Go2/Lite3 -> stop
```

两者名称区分大小写，不能合并或用 alias 替换。当前映射执行停车；这是
安全停车代理，不代表已经实现真实机器狗的跌倒救援或手势反馈动作。

## 5. Go2、Lite3、导航与动作配置

| 文件 | 内容 |
|---|---|
| `config/behavior_tree_actions.yaml` | Behavior → Stage → `ACT_*`，唯一行为注册表 |
| `config/action_catalog.yaml` | `ACT_*` 元数据、中断策略、资源 |
| `config/controller_routes.yaml` | 动作到控制器路由 |
| `config/go2_sport.yaml` | Go2 SportMode 动作序列 |
| `config/lite3_actions.yaml` | Lite3 动作计划 |
| `config/navigation_waypoints.yaml` | waypoint_nav 精确地点名、随机目标 `K` 和行为路由 |
| `config/sound_config.yaml` | 语音指令犬叫、精确行为音频与阶段音频（`stage_sounds`）映射 |

19 个核心语音指令均有独立 Behavior。本批新增
`walk_to_random_point/go_out_to_play/go_home/approach_owner/back_up/`
`stand_still/hold_position/quiet`。其中前三项依赖 Nav2；`quiet` 只停止当前音频；
`hold_position` 保持当前姿态，不等价于 `emergency_stop`。GO_OUT 与 WALK 均向
`waypoint_nav` 发送保留目标 `K`，GO_HOME 使用配置的 YAML 精确地点名。
| `config/sounds/` | 随包安装的 MP3/WAV 行为音频素材 |
| `config/wake_orientation.yaml` | 唤醒角度到 `/spin` 的转换 |
| `config/safety_policies.yaml` | 安全约束 |

往 `config/sounds/` 加新素材后**必须重新 `colcon build`**：该目录在安装树里是真实
目录（不是符号链接），不重编译新文件不会出现，播放会静默失败。转录素材时本机
`ffmpeg` 没有 mp3 编码器，用 GStreamer 的 `lamemp3enc`，例如 96 kHz 单声道 WAV 转
44.1 kHz / 128 kbps（MP3 不支持 96 kHz，采样率必须显式降下来）：

```bash
gst-launch-1.0 -q \
  filesrc location="输入.wav" ! wavparse ! audioconvert ! audioresample \
  ! audio/x-raw,rate=44100,channels=1 \
  ! lamemp3enc target=bitrate bitrate=128 cbr=true \
  ! filesink location=config/sounds/输出.mp3
```

启用 Go2：

```bash
source /opt/ros/humble/setup.bash
source /home/cat/ros2_ws/install/setup.bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=go2 \
  go2_enabled:=true \
  attention_tracking_enabled:=true \
  uwb_follow_enabled:=true
```

启用导航路由还需 `navigation_enabled:=true`，并保证 `/waypoint_nav/task`、
`/waypoint_nav/status` 可用；随机导航也只通过该接口提交 `place="K"`。唤醒转向使用
`/spin`，不使用时关闭 `wake_orientation_enabled`。

唤醒角度输入帧固定为 `microphone_array`。Voice/BehaviorTree 透传原始角度，只有
Action 使用 `wake_angle_zero_offset_deg` 和 `wake_angle_direction_sign` 做一次安装
标定，输出 `base_link` 相对 yaw；不要在 Voice 或 BehaviorTree 重复校准。

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

界面使用 Qt 实时绘制暖色金毛风格电子狗脸，以金黄渐变毛色、长下垂耳、圆润脸颊、
奶油色口鼻和温暖棕色眼睛形成头部形象，不依赖单张 PNG 才能显示。默认 Normal Mode
只显示大尺寸狗脸、面向普通用户的简短状态文案，以及有实际进度时出现的细圆角光带；
Behavior、Stage、ACT、Goal ID、Feedback 和 Result 等工程字段默认不占据面部界面。

按 `D` 或 `F1` 可打开右侧暖色半透明 Debug Overlay，查看 Behavior、Emotion、
Intensity、Level、Stage、Action、精确进度、Goal ID、Action Status、Feedback、Result
和 ROS 连接状态。Normal Mode 与 Overlay 使用同一份实时状态，不存在两套事件逻辑。

内部将 `emotion`、`activity`、`system_state` 和 `intensity` 分开，再生成用户文案与
`follow/search/listening/success/failed/emergency` 等 face cue。不同情绪和活动会改变
眼睛、耳朵、视线、嘴型、颜色和动画节奏；喜悦/社交带上浮爱心，焦虑/恐惧带动态汗滴。
Intensity 同时影响速度、幅度、五官形状和特效数量。Emergency 会抑制气泡和爱心，
显示暖红脉冲边缘；终态仍由 Action Result 决定。

界面按 `goal_id` 忽略旧任务的迟到 Result，并可通过 Feedback 恢复启动前已经开始的
行为。程序默认全屏启动，按 `F11` 在全屏和普通窗口之间切换，按 `Esc` 退出全屏。

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
- `follow_owner` 的单个长期 Goal 只启动一次 Go2 UWB 进程组或 Lite3 内层 Action。
- 语音 idle 不停车；CancelGoal 后等真实 Result，再确认进程/内层 Action 终态与停车。
- `play_alone` 在同一 Goal 内完成至少两个“漫游、停稳、姿态”周期。
- Tree 停止续租后，Action 在租约到期时停车并返回失败。
- Action 执行期间后台跟踪没有并发写速度。
- `/cmd_vel` 只有经过系统设计的发布者。

部署时在共享 ROS 工作区重建两个包，在同一终端重新 source 并重启 Tree、Action；
运行中的 Python 节点不会热加载。重启后核对加载文件与 ROS 图：

```bash
source /opt/ros/humble/setup.bash
cd /home/cat/ros2_ws
colcon build --symlink-install --packages-select marsdog_behavior marsdog_action_executor
source install/setup.bash
hash -r
ros2 pkg prefix --share marsdog_behavior
ros2 pkg prefix --share marsdog_action_executor
python3 -c 'import inspect, marsdog_behavior.ros_node as t, marsdog_action_executor.ros_node as a; print(inspect.getfile(t)); print(inspect.getfile(a))'
ros2 action info /execute_behavior -c
ros2 action info /go2/follow_uwb -c
ros2 action info /go2/random_roam -c
ros2 topic info /behavior/goal_lease -v
ros2 topic info /cmd_vel -v
ros2 topic echo /go2/behavior_diagnostics
```

上述软件检查不能代替真机验证 UWB 失联、控制器反馈、漫游停稳、姿态和取消停车。

真机排障与已知陷阱（姿势恢复的入口清单、两棵 install 树、日志判据）见
[SESSION_ISSUES_2026-09-20.md](SESSION_ISSUES_2026-09-20.md)。

## 8. 修改接口时必须协同

- 新增 Behavior：行为树负责人提供名称/优先级/参数，动作负责人注册 Stage/ACT，并共同做 Action 冒烟测试。
- 修改 `ACT_*` 不应改变上游 Behavior 名；若必须改 Behavior 名属于跨项目 MAJOR 变更。
- 修改长期 Goal 续租参数须同步 Tree 与 Action，并保持 `goal_id`/`behavior_id` 身份匹配。
- 修改 Action 字段必须先改 `marsdog_interfaces`，再同步行为树 Client、本项目 Server、文档和联调脚本。
