# play_alone：UWB 随机点后玩耍

上游链路：`自己去玩吧` → `EVT_VOICE_COMMAND_PLAY_ALONE` /
`CMD_PLAY_ALONE` → `command_play_alone` → `play_alone`。
Tree 负责 AudioEvent v2 的 specific_command、匹配事件、命令 ID、允许触发校验，
以及 Lv1 / sub_priority=1、立即中断、长期 Goal（`timeout_sec=0`）。
Action 接收 `/execute_behavior` 的精确行为名 `play_alone`，不直接订阅语音事件。

同一个 Behavior Goal 内重复以下完整周期，直到取消或失败：

1. `uwb_roam` 必需阶段：`ACT_UWB_RANDOM_ROAM` → Lite3 专用 `uwb_roam` 客户端。
   先完成 Lite3 站立/控制权预检，再发送 `/go2/random_roam`。
   Goal 为 `random_seed=0, timeout_sec=30, min_radius=0.5, max_radius=2.0`；
   剩余预算不足时缩短 timeout，低于可安全发送的预算则拒绝。
   圆心由服务端冻结在调用时 UWB 位置，不使用地图点位 `K`。
2. 仅 `STATUS_SUCCEEDED(4)` **且** `SUCCESS(0)`，并且 Lite3
   `finish_navigation()` 确认停稳后，进入 `play` 必需阶段。
3. 均匀随机选一个：`ACT_GUARD_DOOR`（扭腰）、
   `ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD`（低头）、`ACT_STRETCH`（降低身体后恢复）。
   均复用现有 Lite3 有界姿态计划、状态检查及轴复位，每次只做一个，预算 5 秒。
   这些是身体姿态表达，不是抓玩具或厂家跳跃动作。

每次漫游单元预算 40 秒，玩耍单元预算 5 秒；长期 Goal 无外层截止时间。每个周期完成后发送包含周期、阶段和 `safe_to_interrupt` 的 Feedback，再开始下一次漫游。
取消先发给本次 Goal，**取消确认不是终态**：等待真实 Result 后才交还运动控制权。
若下游终态未知，保留控制权故障锁并拒绝替代 Goal；人工确认停车、恢复下游后
重启 Action。急停入口仍可用。所有非成功业务码均不执行玩耍。

启动沿用 [UWB 接口部署](UWB_FOLLOW_ROS2_INTERFACE.md) 和用户提供的
`go2_follow_develop/src/go2_uwb_behavior/docs/上层对接与部署.md`。
本功能仅在 Lite3 且 `uwb_follow_enabled=true` 时注册；其他底盘或缺少服务端时明确失败。
无需 Action 自己启动 UWB、相机或规划器。
当前下游配置是 `publish_idle_velocity: false`；旧部署文档中“空闲也持续发零速”
与该配置已有差异。联调应核对实际参数和 `/cmd_vel` 发布者，不同时启动旧跟随链。
漫游遵守下游 STOP 锁存，不自动发 IDLE 解锁。Tree 每次 tick 在 `/behavior/goal_lease` 按 `goal_id`/`behavior_id` 续租；语音 idle 不取消此 Goal，续租过期则失败并停车。

软件回归覆盖成功双条件、所有失败业务码、Goal 拒绝、取消确认不释放、晚到的
Goal 接受、停稳失败和终态传输异常。尚需实机验证 UWB 到点、姿态表现、
取消停车以及多周期长期 Goal。运行中的节点不会热加载新 Python，部署需重建并重启。

## 可选：UWB 目标点 → Action → Nav2

默认仍是上文的 `/go2/random_roam`。显式设置 `uwb_roam_execution_mode:=nav2`
或 `config/uwb_follow.yaml` 的 `roam_execution_mode: nav2` 后使用下面的链路。
同一个 Behavior Goal 内重复以下完整周期，直到取消或失败：

1. `ACT_UWB_RANDOM_ROAM` 仍路由到 `uwb_roam`。Action 确认上游 IDLE、
   完成 Lite3 站立/控制权预检，调用 `/go2/generate_navigation_goal`。
   请求包含唯一 `request_id`、`random_seed=0, min_radius=0.5, max_radius=2.0`。
   上游冻结 UWB 圆心，按原圆环和障碍净空约束生成一次固定 `map` 位姿，
   返回 `navigation_goal` 和 `center_pose`；不使用地图点位 K。
2. Action 检查本次请求 ID、成功业务码、位姿、时间戳和采样半径，
   最多发送一个 Nav2 `/navigate_to_pose` Goal。生成成功仅表示有目标，不能执行玩耍。
3. 只有 Nav2 真实 `STATUS_SUCCEEDED(4)` 且未发生取消/围栏失败，
   并且 `finish_navigation()` 确认底盘停稳后，才进入 `play` 必需阶段。
4. 均匀随机选一个 `ACT_GUARD_DOOR`、`ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD`
   或 `ACT_STRETCH`。这些是既有 Lite3 有界身体姿态表达，每次只做一个，预算 5 秒。

漫游单元预算 40 秒：目标生成等待最多 5 秒，Nav2 内部导航最多 30 秒，
为结束停稳预留时间；取消后还需等待真实 Result，确认窗口可延长实际收尾时间。
长期 Goal 无外层截止时间，Tree 按 `goal_id` / `behavior_id` 续租，语音 idle 不取消它。

取消确认不是终态。取消先请求 Nav2 停止，真实 Result 前不交还控制权。
晚到的 Goal 接受必须取消并保留对应 Result；终态未知、传输异常或停稳失败时
保留故障锁并拒绝替代运动 Goal。人工确认停车并恢复下游后重启 Action。
所有目标生成或导航失败均不执行玩耍。

Nav2 使用自己的代价地图和规划器。目标生成仍检查圆环和障碍净空；
Action 按 Nav2 的地图位姿反馈检查相对冻结圆心的 6 米范围，越界请求取消并等待终态。
此反馈检查存在反馈/制动延迟，不等同于旧局部链的逐速度预测围栏。

## 启动与重建

先在 `/home/cat/robot_ws/go2_follow_develop` 构建新服务：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select go2_uwb_behavior
source install/setup.bash
ros2 launch go2_uwb_behavior navigation_targets.launch.py
```

此模式不会创建 `/cmd_vel` 或名义速度发布者，`/go2/navigation_target` 仅供显示；
Action 只消费本次服务响应，不会因显示话题的一条旧目标而自动运动。
UWB 串口、双目相机、Nav2 与地图定位仍需按部署要求启动。
Lite3 底盘使用 `use_bridge:=false use_action:=false`，由 Action 管理程序/遥控模式。
必须存在有效 `map←odom` TF，Nav2 必须处于 active 且提供 `/navigate_to_pose`。

再在 `/home/cat/ros2_ws` 构建 Action，构建及运行终端都需 source 新的 UWB 安装空间：

```bash
source /opt/ros/humble/setup.bash
source /home/cat/robot_ws/go2_follow_develop/install/setup.bash
source /home/cat/ros2_ws/install/setup.bash
colcon build --symlink-install --packages-select marsdog_action_executor
source /home/cat/ros2_ws/install/setup.bash
```

重启 Action；运行中的 Python 节点不会热加载源码。
`uwb_follow_enabled=true` 仍用于注册 Lite3 的 UWB 适配器。

## 兼容旧链路

`uwb_roam_execution_mode:=legacy` 可明确选择旧 `/go2/random_roam` 运动 Action。
此时上游使用 `behavior_follow_roam.launch.py` 的原运动模式；不能搭配 target-only 节点。
持续 `/go2/follow_uwb` 也使用原运动模式。目标生产者名为 `/uwb_navigation_target_node`，
运动节点名为 `/uwb_behavior_controller_node`；两者共享服务和感知话题，不能同时启动。
切换需停止并重启上游；不会静默退回旧速度链。

## 软件验证与实机验收

软件测试覆盖目标生成、请求关联、输入超时、坐标和时效、单个 Nav2 Goal、
生成失败不导航、取消确认不释放、晚接受 Goal、终态未知锁定、围栏与停稳失败。
目标模式节点测试检查没有底盘速度发布者。
仍需实机验证 UWB 目标、Nav2 到点、姿态表现、取消制动及长期多周期体验。
