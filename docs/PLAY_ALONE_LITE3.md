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
