# MarsDog Action Executor

> 独立项目交接、Action 契约、Go2/Lite3、UWB 跟随与调试说明见
> [docs/HANDOFF.md](docs/HANDOFF.md)。

仿生机器狗行为执行器。接收上游行为树下发的 `behavior_name` 和
`params_json`，按新的行为动作对照表生成 Stage，选择精确的 `ACT_*` 并执行。

## 严格行为树契约

运行时唯一行为来源是
[config/behavior_tree_actions.yaml](config/behavior_tree_actions.yaml)：

- 73 个可执行行为；
- 269 条候选动作记录；
- 182 个唯一 `ACT_*`；
- 行为名和动作名均大小写敏感；
- 不加载旧行为模板；
- 不解析旧行为 alias；
- 不向运行时动作目录暴露新表之外的动作。

例如：

```text
sit_down       -> ACT_BASIC_SIT
emergency_stop -> ACT_SYSTEM_EMERGENCY_STOP
respond_person_fall -> ACT_PERCEPTION_RESPOND_PERSON_FALL
respond_stop_gesture -> ACT_PERCEPTION_RESPOND_STOP_GESTURE
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
  精确校验 behavior_name 是否属于 73 个直接行为
        │
        ▼
ConfigLoader
  从 behavior_tree_actions.yaml 获取有序 Stage
  校验 action_catalog.yaml 恰好包含表引用的 182 个 ACT_*
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
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Lite3 速度输出及外部 UWB 链路 |
| `/api/sport/request` | `unitree_api/msg/Request` | 可选 Unitree Go2 高层动作输出 |
| `/waypoint_nav/task` | `marsdog_voice_interaction/srv/VoiceTask` | YAML 精确地点名及保留随机目标 `K` |
| `/waypoint_nav/status` | `std_msgs/msg/String` JSON | 点位运行态/终态 |
| `/spin` | `nav2_msgs/action/Spin` | 按唤醒声源角度闭环转向 |

Feedback 在每个 Stage 执行完成后发布一次，不是固定 10Hz。
启动行为树、动作执行器和底盘相关节点前，必须统一 ROS Domain 和 RMW；
板端当前配置示例：

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
├── action_catalog.yaml                # 新表引用的 182 个动作元数据
├── go2_sport.yaml                     # 独立 Go2 SportMode 动作与路由覆盖
├── lite3_actions.yaml                 # Lite3 动作计划与路由覆盖
├── navigation_waypoints.yaml          # YAML 地点名、随机目标 K 与行为/Stage 映射
├── wake_orientation.yaml              # 唤醒角度校准和 Nav2 Spin 配置
├── visual_target_approach.yaml         # 人/动物/物体接近距离和速度策略
├── controller_routes.yaml
└── safety_policies.yaml

marsdog_action_executor/
├── config_loader.py       # 严格加载并校验行为、动作与控制器路由
├── behavior_resolver.py   # 精确名称校验，无 alias
├── goal_parser.py
├── execution_context.py
├── stage_executor.py
├── adapters/velocity.py
├── adapters/go2_sport_backend.py
├── adapters/lite3_backend.py
├── adapters/navigation_adapter.py
├── adapters/waypoint_nav_adapter.py
├── adapters/wake_orientation_adapter.py
├── result_evaluator.py
├── ros_node.py
└── units/

docs/
├── ARCHITECTURE.md
├── ROS2.md
├── GO2_ROS2_INTEGRATION.md
├── LITE3_ROS2_INTEGRATION.md
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

启用 Go2：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=go2 \
  go2_enabled:=true \
  go2_request_topic:=/api/sport/request
```

启用 Lite3 并接入 waypoint_nav 固定地点与服务端随机目标：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=lite3 \
  lite3_enabled:=true \
  navigation_enabled:=true \
  waypoint_nav_service_name:=/waypoint_nav/task \
  waypoint_nav_status_topic:=/waypoint_nav/status
```

固定路由使用 `config/navigation_waypoints.yaml` 中与
`$HOME/.ros/waypoints.yaml` 完全一致的地点名称（当前为卧室、充电桩、
厨房、卫生间、客厅）。随机行为不再由 Action 在固定点中抽选，而是唯一调用
`/waypoint_nav/task` 并发送 `task_type="goto_place"`、`place="K"`；
`waypoint_nav` 根据实时 `/map` 生成安全目标。`K` 是保留入口，不写入
`waypoints.yaml`。

启用 Unitree Go2（需先 source 官方 `unitree_ros2` 工作空间）：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=go2 \
  go2_enabled:=true \
  go2_request_topic:=/api/sport/request \
  navigation_enabled:=true
```

`chassis_type` 只接受 `go2` 或 `lite3`。73 个行为引用的 182 个动作均有明确 Go2 路径：171 个 SportMode 映射，
其余使用 Nav2、UWB、视觉或唤醒专用适配器；`quiet` 是无身体动作的音频策略。
当前 Go2 `follow_owner` 使用外接 FTDI UWB 与 `go2_uwb_local_follow` 进程链路；
Lite3 使用 `/go2/follow_uwb` Action 和 `/go2/set_behavior` Service。
完整映射、视觉约束和真机验收步骤见
[Go2 ROS2 动作集成说明](docs/GO2_ROS2_INTEGRATION.md)。
“过来/靠近/回来”由 Tree 先绑定已确认的主人视觉目标，Action 再通过
Vision 单次定位与 Nav2 执行避障导航。Tree 不再需要额外开关。

启用底盘后，`respond_owner_call` 会读取行为树传入的
`microphone_array` 原始 `wake_angle_deg`；Action 是唯一安装标定所有者，经
offset/sign 转成 `base_link` 相对 yaw 后调用 `/spin`。角度标定和联调见
[唤醒声源朝向说明](docs/WAKE_ORIENTATION_INTEGRATION.md)。
当前线性四麦阵列存在前后镜像歧义：Action 在首次转向后等待新鲜的
`active_target` 人物，未确认时再原地检查后向镜像角。该搜索不选人也不移动
底盘位置，后续接近仍要求严格的 `vision_epoch + target_id`。

同一语音会话随后可下发精确行为 `approach_voice_caller`。其唯一动作
`ACT_INTERACT_APPROACH_VOICE_CALLER` 锁定 `vision_epoch + target_id`，向
`/perception/vision/task` 请求一次 `locate_person_once`。SLAM 成功且
`navigation_required=true` 时向 Nav2 `/navigate_to_pose` 发送一次地图目标；
`false` 时直接完成。Action 不再用 bbox 或直接 `/cmd_vel` 追逐唤醒者。
取消要等待 Nav2 真实 Result；终态无法确认时锁定恢复状态并拒绝替代运动 Goal。

Social / Exploration 中有明确人、动物或物体目标的 13 个行为、六类普通
`express*WithHuman`，以及 `unhappy`、`miss_owner`、`farewell_leave`，现在都采用两阶段
执行：先由 `ACT_APPROACH_VISUAL_TARGET -> visual_target_approach` 锁定 Tree
选中的目标并闭环接近，成功停车后才执行原来的互动/检查动作。停止距离由
`config/visual_target_approach.yaml` 按行为配置。三个主人定制行为还分别覆盖
接近速度、人体框贴近程度和到达保持时间：`unhappy` 缓慢靠近并安静停留，
`miss_owner` 较快贴近后执行兴奋摆动，`farewell_leave` 在 3 秒观察窗口内若主人
继续离开会恢复跟随同一视觉 Track。正式运行只接受
`range_valid=true + distance_m`；缺目标、错类型、目标歧义、epoch 不匹配、
距离无效、丢失、取消或超时都会清零底盘并终止，后续动作不会继续。视觉短暂
失效后需连续 3 个新有效快照才恢复移动；保持停止期间会合并重复停车请求，并用
距离/框高迟滞抑制阈值附近的反复启停。
无具体目标的 `exploreRoom` 仍执行房间探索，不进入该接近阶段。
Social 目标身份为 `unknown` 时，绑定 Vision 已确定选择的新鲜 `active_target`
稳定 Track，不从多人候选中随机选人；active target 无效时继续安全拒绝。

六类普通 `express*WithHuman` 使用上述通用视觉目标接近，不复用语音唤醒专用的
`ACT_INTERACT_APPROACH_VOICE_CALLER`。只有接近成功并停车后才执行人物互动表达。
`express*Alone` 在启用导航后按配置概率保持原地，或将保留目标 `K`
交给 `waypoint_nav` 生成实时随机点，到达后再执行表达 Stage，要求
所选底盘的 `*_enabled:=true` 和 `navigation_enabled:=true`。

语音会话 WAITING 阶段使用六个 `express*InPlaceWithHuman`。这些行为必须携带
`"mobility_policy": "in_place"`，计划会在导航、声音和 Controller 调用前完整校验。
缺少策略返回 `inplace_policy_required`，混入目标接近、Nav2 或其他前置移动能力
返回 `inplace_policy_violation`。每个行为直接复用对应普通 `*WithHuman` 的
`expression` Stage 和动作候选，但省略 `target_approach`；表达动作自身的小幅
对应平台的表达动作仍会执行，完成、失败和取消出口都会停车。

视觉安全事件使用两个独立、大小写敏感的正式行为：`respond_person_fall` 与
`respond_stop_gesture`。它们分别映射
`ACT_PERCEPTION_RESPOND_PERSON_FALL`、
`ACT_PERCEPTION_RESPOND_STOP_GESTURE`；Go2 和 Lite3 映射均执行停车动作。

主人定制动作的精确链路是：

```text
unhappy        -> ACT_APPROACH_VISUAL_TARGET -> ACT_OWNER_UNHAPPY
miss_owner     -> ACT_APPROACH_VISUAL_TARGET -> ACT_EXPRESS_MISS_YOU
farewell_leave -> ACT_APPROACH_VISUAL_TARGET -> ACT_OWNER_GOING_OUT
```

三个行为都要求 Tree 在 `params_json.target.identity` 中明确传入 `owner`；普通
人体、陌生人或家庭成员目标会在任何非零运动前失败并保持零速度。
如果 Tree 输出 `current owner visual target unavailable`，表示 Goal 还没有
进入 Action，此时调整接近速度无效。先在板端查询同一份 Vision 快照：

```bash
ros2 service call /perception/vision/task \
  marsdog_vision_interaction/srv/VisionTask \
  "{task_id: 'owner-check', task_type: 'query_targets', \
params_json: '{\"target_types\":[\"human\"],\"min_confidence\":0.5,\"max_age_ms\":500}'}"
```

可用的主人目标必须在 `targets[]` 中同时带有非空
`vision_epoch`/`target_id`、`identity=owner`，并且为新鲜人体候选。
`identity=family_member_*` 不等于主人；`unknown` 说明人脸尚未确认。如果
Service 不可用，Tree 只会回退到 0.5 秒内的
`/perception/visual_event.human_candidates[]`，旧快照不会触发移动。

上述主人视觉门槛只用于 `unhappy`、`miss_owner`、`farewell_leave` 社交行为。
`approach_owner`、`come_to_owner`、`return_to_owner` 由 Tree 直接下发；Action
在执行时从 `/perception/visual_event.active_target` 绑定新鲜人体轨迹，允许
`identity=unknown`，再调用 `locate_person_once` 并按需导航。没有可用人体轨迹时
返回 `visual_target_unavailable`，不会发送 Nav2 Goal。

Go2 的身体动作走独立 SportMode 映射，Lite3 走对应动作计划。没有口部、尾部、面部或物体夹持能力的语义使用配置中明确
标记的 `proxy` 身体表达，不把它解释为真实咬取、张嘴或搬运动作。三个主人行为
在视觉接近完成后均不再盲目前进，且最终发送停止。当前自动化测试不等于四足
姿态、步态稳定性或真机安全验收。

Lite3 模式覆盖全部 182 个 `ACT_*`：169 个进入 Lite3 backend，13 个保留专用
闭环控制器（其中 2 个是永久拒绝的 `ACT_BONE_DANCE` / `ACT_JUMP_PAW`）。Lite3
原生的打招呼和已确认不影响机背载荷的扭身体动作分别用于趴下社交和站立社交
fallback；其余不具备的生物动作和物体操作统一映射到
目标姿态闭环或可观察、限幅的俯仰、偏航和机身高度代理。睡觉、趴下、起床和起立
先读取新鲜 `/robot_status`，仅在方向确定时发送一次起立/趴下 toggle，并连续确认
目标稳定状态；超时绝不盲目重发。趴下态下执行 `require: stand` 的步骤时，backend
先在入口处起立并确认稳定站立再继续，避免一次趴下把后续所有站立动作卡死。
睡觉行为到点后先原地转一圈（`ACT_CIRCLE_AROUND`，开环 0.5 rad/s × 12.6 s）再
趴下；上厕所到点后绕点走 3 圈（`ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT`，半径
0.2 m、位移开环 15.7 s），最后以新增的 `ACT_RAISE_HEAD` 抬头作为完成信号。两个
转圈都是开环计时，因为 `/robot_status` 不携带 yaw。绕点走圈是第一个由后端持续
向前驱动的计划，因此额外声明 `minimum_forward_clearance_m` 门禁——后向超声检查
不覆盖前进方向。俯仰轴是死区轴（`|value| ≤ 6553` 视为 0），原来的 7500 只比死区边高
947，真机上操作员**完全看不见**低头和抬头，所以抬头与 9 个低头动作统一抬到 20000。
同一轮把俯仰的保持时间也调长了（低头 1.0 → 2.0 s，抬头 2.5 s）：幅度看得见之后
1 s 仍然读不出一个动作。偏航代理是一份被 `plan_id` 共享的物理计划，6 个定义引用
同一个锚点、加载器要求逐字节相同，所以扭腰只能整族一起从 0.8 s 改到 1.6 s（一次
影响 114 个动作），随之有 17 个单元超时从 3.0/4.0 s 抬到 5.0 s——保持、模式切换
和稳定等待共用单元预算，趴下态的入口起立还会再加 2.2 s。
注意「超出死区多少」不是可见性判据：偏航 10500 同样只高 947，却清晰可见，俯仰这条
轴更钝。结果在
`metadata_json.lite3_action` 中标记
`fidelity=proxy`、`semantic_effect=proxy_motion`；不会据此更新为虚假的坐下
或物体持有状态。坐下没有可靠的 Lite3 稳定状态，仍使用机身降低代理。由于
机背安装有算力卡和语音设备，横滚轴 `0x21010131` 同样在配置校验和运行时永久
禁止。起立/趴下 toggle 仍禁止作为普通 `simple` 步骤，只能由目标姿态闭环或入口期
姿势恢复调用；
APP 语音、软急停、翻身和跳跃指令不会进入行为映射。太空步作为无界步态被永久
封锁，它不会自行停止，会让 `gait_state` 卡住并拖垮之后的所有动作。后退动作要求
新鲜状态和后方超声净空，运行中持续检查并在异常时冗余发布零速度。所有未验收
共享物理计划默认关闭，必须显式打开总门后才可在场测试。

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
- [Go2 ROS2 动作集成说明](docs/GO2_ROS2_INTEGRATION.md)
- [Lite3 ROS2 动作集成说明](docs/LITE3_ROS2_INTEGRATION.md)
- [Go2 ROS2 动作集成说明](docs/GO2_ROS2_INTEGRATION.md)
- [Lite3 ROS2 动作集成说明](docs/LITE3_ROS2_INTEGRATION.md)
- [waypoint_nav 点位适配说明](docs/NAV2_WAYPOINT_INTEGRATION.md)
- [唤醒声源朝向说明](docs/WAKE_ORIENTATION_INTEGRATION.md)
- [视觉目标接近说明](docs/VISUAL_TARGET_APPROACH.md)
- [行为与动作映射说明](docs/BEHAVIOR_ACTION_MAP.md)
- [行为树动作契约](docs/BEHAVIOR_TREE_ACTION_CONTRACT.md)
- [仿真页面接入说明](docs/SIMULATION_PAGE_INTEGRATION.md)

Lite3 新增 [`play_alone`：UWB 随机点后随机姿态动作](docs/PLAY_ALONE_LITE3.md)。
