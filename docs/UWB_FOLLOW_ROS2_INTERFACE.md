# UWB 跟随：go2_uwb_behavior 接口对接

## 1. 背景

追随功能在 2026-09-20 换了实现。新包 `go2_uwb_behavior`（C++）自己拥有整条回路：
UWB 输入、规划、以及 `/cmd_vel` 输出。它对上层只暴露三个接口：

| 接口 | 类型 | 用途 |
|---|---|---|
| `/go2/follow_uwb` | Action `FollowUwb` | 跟着 UWB 信标走 |
| `/go2/random_roam` | Action `RandomRoam` | 原地玩耍，走到随机点后**自动结束** |
| `/go2/set_behavior` | Service `SetBehavior` | 只接受 `IDLE=0` / `STOP=2` |

两个 Action **全局互斥**：已有任务在跑时再发一个会直接 `accepted=false`。

## 2. 本执行器怎么接

按 `chassis_type` 分流，两条链路互不影响：

| 底盘 | 追随实现 | 位置 |
|---|---|---|
| **lite3** | **本文件的 Action/Service 客户端** | `adapters/uwb_follow_action_adapter.py` |
| go2 | 外接 FTDI + `local_follow` 进程管线 | `adapters/uwb_follow_adapter.py` |

**lite3 上本执行器只当客户端，一个进程都不起。** 旧的两个进程
（`uwb_aoa_pkg`、`go2_uwb_local_follow/local_follow.launch.py`）在 lite3 上不再使用。

### 长期 Goal 的建立与结束

Tree 发送一个 `follow_owner` `/execute_behavior` Goal，`timeout_sec=0`。
`goal_id`/`behavior_id` 是 Goal 身份，`interaction_id` 只记录来源。Action 在同一个
Goal 内启动一次控制器，持续发送阶段和 `safe_to_interrupt` Feedback，直到取消、
控制器失败、UWB 目标失鲜、Tree 续租失效或节点退出。语音 idle 只关闭会话级
`/behavior/attention_tracking`，不会停止跟随。

Lite3 先确认 `/go2/follow_uwb` 和 `/go2/set_behavior` 可用，再调用
`prepare_navigation(allow_uwb_chain=True)`，发送一次内层 `FollowUwb` Goal
（`timeout_sec=0`）。取消先请求内层取消并发送 `STOP`，**等待内层真实 Result**，
确认停稳后才返回外层 `CANCELED`。若 Result 或停车不能确认，锁定恢复状态并拒绝
替代 Goal。内层 UWB 输入超时、反馈失鲜和服务健康检查仍有有限时限。

Go2 只启动一次 FTDI AOA 与本地规划进程组；Action 持续检查子进程及
`/uwb/target_point` 新鲜度（默认 3 秒）。结束时确认两个进程退出并补发零速。
默认不自动恢复。Node 的 Goal 锁在真实 Result 后释放；`CANCEL_REQUESTED`
仍持有运动控制权。

Tree 每次 tick 在 `/behavior/goal_lease` 发送
`{"schema_version":1,"goal_id":"...","behavior_id":"..."}`。
Action 按两个 ID 匹配，首次续租宽限 3 秒，此后 2 秒未收到续租即失败并停车。
Tree 被强杀时即使没有 CancelGoal，租约也会过期。控制器终态未知时保留
`recovery_required`，人工确认后重启 Action。

### 结果码

`code` 是权威，`status` 只看 `STATUS_SUCCEEDED`：

| `code` | 含义 | 本执行器 |
|---|---|---|
| 0 `SUCCESS` | 正常结束 | 长期跟随意外结束则外层失败；有界旧 Stage 才可成功 |
| 1 `CANCELED` | 被取消 | 取消路径等真实 Result 后返回外层 `CANCELED` |
| 2 `PREEMPTED_BY_MODE` | 被别的模式抢占 | 本 Goal 主动 STOP 后可作为取消终态；其他情况失败 |
| 3 `NOT_READY` | 就绪窗口内感知链没起来 | 失败——通常意味着 T1-T4 没起齐 |
| 4 `TIMEOUT` | 到达 goal 自己的 `timeout_sec` | 失败，`last_error` 带码 |
| 5 `INPUT_TIMEOUT` | UWB 输入持续失效超过 3 s | 失败，`last_error` 带码 |
| 6 `STOP_UNCONFIRMED` | 停车没确认 | 失败——**安全相关，必须让操作员看见** |

有界旧 Goal 保留既有 Stage 时长语义。长期 Goal 的内层 `timeout_sec=0`，由外层
取消、续租失效或控制器故障结束；取消后仍须等真实内层 Result。

## 3. `/cmd_vel` 独占与 `lite3_control_conflict`

新链在行为运行时发布 `/cmd_vel`；当前部署配置的空闲发布行为须现场核查
（`publish_idle_velocity`）。本执行器的 Lite3 twist 类动作（`ACT_CIRCLE_AROUND`、
`ACT_BASIC_BACK_UP`、`ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT` 等）也发 `/cmd_vel`。
两者并存会让速度命令互相覆盖。

处理方式是**检测到冲突就响亮失败**，不静默抽搝：

- `Lite3ChassisBackend` 的所有运动出口都过 `_motion_ready()`，其中
  `_control_ownership_ready()` 会数节点名。`/uwb_behavior_controller_node` 在列。
- 于是新链在跑时，twist 动作失败并给出：

  ```text
  lite3_control_conflict:active_nodes=/uwb_behavior_controller_node=1
  ```

- **唯一的例外是跟随自己**：跟随就是要把 `/cmd_vel` 交给这条链，所以跟随适配器调
  `prepare_navigation(allow_uwb_chain=True)`。这个让位是**显式参数、默认 False**——
  其它所有动作一律继续拒绝，否则「跟随的准备步骤被它依赖的那个节点挡死」。
- 豁免只摘掉这一个名字：报告里若还有别的竞争节点（如 `/lite3_twist_bridge`），
  仍然照常失败。

**已知边界**：所有权检查是**进入时**的闸门（`_motion_ready`）。`_runtime_motion_ready`
是每 tick 的廉价检查，不做所有权检查。所以「动作已经在跑、链才启动」这种情况抓不到。

### 抢占与运动所有权

Tree 对 `follow_owner` 和 `play_alone` 持续仲裁。Lv0 急停可立即发停车；
Lv1 跌倒或停止手势先请求取消旧 Goal，再等待真实 Result，随后发送替代 Goal。
Action 在旧 Goal 占锁或取消待决时拒绝任何普通替代 Goal。原会话跟随优先级门
已移除，抢占后不会自动重新启动跟随。

### 不要启动的东西

**不要**单独启动 `local_follow.launch.py`、`uwb_follow_only.launch.py`、
`local_velocity_planner.launch.py` —— 它们的默认输出话题已经被改成 `/cmd_vel`，
会和行为节点打架（`enable_motion:=true` 时尤其危险）。

## 4. 部署：四个终端

| 终端 | 命令 |
|---|---|
| T1 | `ros2 launch <lite3_bringup> lite3_bringup.launch.py` |
| T2 | realsense 相机（先关 emitter） |
| T3 | `ros2 launch uwb_aoa_pkg uwb_source.launch.py` |
| T4 | `ros2 launch go2_uwb_behavior behavior_follow_roam.launch.py enable_motion:=true` |

T4 自己会拉起 6 个节点：`uwb_behavior_controller_node`、`uwb_target_adapter_node`、
`stereo_obstacle_projector_node`、`disparity_node`、`rolling_obstacle_map_node`、
`local_velocity_planner_node`。

首次实机建议把就绪窗口从 2.0 放宽到 4.0 ——从「收到 Goal」到「障碍点云真正新鲜」要串起
投影建订阅 → 双目出视差（实测约 4 Hz）→ 投影 → 滚动地图 2 帧确认，首帧链路可能贴近
2 秒而误报 `NOT_READY`。本仓侧的对应参数是 `config/uwb_follow.yaml` 的
`ros_interface.ready_timeout_sec`（默认已是 4.0，指我们等 Action/Service 出现的时间）。

```bash
ros2 action list | grep go2        # 三个接口都在
ros2 action feedback /go2/follow_uwb
```

### 跟随时会频繁切模式

`lite3_twist_bridge` 的逻辑是：`/cmd_vel` 非零 → 切 Vision Mode；连续归零超过
`idle_timeout`（默认 1 秒）→ 退回 Joystick Mode。所以主人站着不动超过 1 秒，狗会退回
遥控器模式，主人再走又切回来。这是既有链路的行为，不是本次引入的。不想看到频繁切换：

```bash
ros2 param set /lite3_twist_bridge idle_timeout 10.0
```

## 5. 时间预算（对方默认参数）

| 环节 | 数值 |
|---|---|
| 感知链输出 | 双目视差实测约 4 Hz |
| 障碍点云超时 | 0.70 s 未刷新 → 立即零速 |
| 任务启动就绪窗口 | 2.0 s（首跑建议 4.0） |
| 输入中断恢复 | 3.0 s 内恢复自动继续，否则 `INPUT_TIMEOUT` |
| 停车确认 | 里程计连续 0.3 s 低于阈值；最多等 2 s |

感知链**只在 Action 活动期间**计算；空闲时门控关掉，不跑双目匹配。但相机驱动、UWB、
底盘节点是常驻的。

## 6. 排障

| 现象 | 原因 |
|---|---|
| `uwb_follow_action_unavailable:/go2/follow_uwb` | T4 没起，或 DDS 域不一致 |
| `uwb_set_behavior_unavailable:/go2/set_behavior` | 同上；这一条会让跟随**直接拒绝启动**（没有刹车就不交底盘） |
| `lite3_control_conflict:...` | 新链在跑而你在跑 twist 动作。要么关掉 T4，要么改用跟随 |
| `uwb_follow_failed:NOT_READY` | 就绪窗口太短，按 §4 放宽 `readiness_timeout_sec` |
| `uwb_follow_failed:INPUT_TIMEOUT` | UWB 输入断了超过 3 s：看 T3 的信标与串口 |
| `uwb_follow_no_time_budget` | 仅旧的有界 Stage 缺少有限预算；长期 Goal 不走这个 Stage |
| 日志 `Rejected goal while previous Result is pending` | 旧 Goal 尚未终结；等待真实 Result 后重试 |
| `uwb_follow_disabled` | `uwb_follow_enabled:=false` |

## 7. 相关

- `config/uwb_follow.yaml` —— 两种机制的选择与参数都在这里
- `marsdog_action_executor/adapters/uwb_follow_action_adapter.py` —— 客户端实现
- 上游文档：`go2_uwb_behavior/docs/上层对接与部署.md`（部署细节与验收清单）
- `docs/LITE3_ROS2_INTEGRATION.md` —— Lite3 底盘整体接入
