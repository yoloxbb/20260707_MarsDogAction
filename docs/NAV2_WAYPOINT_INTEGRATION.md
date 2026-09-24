# waypoint_nav 点位与行为动作序组适配说明

## 1. 目标

在保留精确 Behavior 与 `ACT_*` 边界的前提下，将网页保存到
`$HOME/.ros/waypoints.yaml` 的定点与 waypoint_nav 实时随机目标接入行为执行链：

```text
ExecuteBehavior.Goal
  behavior_name
      │
      ▼
navigation_waypoints.yaml
  behavior_name -> semantic waypoint
      │
      ▼
/waypoint_nav/task (marsdog_voice_interaction/srv/VoiceTask)
      │ matching /waypoint_nav/status reaches SUCCEEDED/NAV2_SUCCEEDED
      ▼
原 behavior_tree_actions.yaml Stage 序列
      │ selected exact ACT_* remains unchanged
      ▼
exact ACT_* -> Go2/Lite3 platform action
```

Action 是生产路径中 `waypoint_nav` 的唯一调用方。固定语义路由发送地点
文件中的精确名称；随机导航默认发送保留目标 `K`，由 waypoint_nav 根据实时
`/map` 选点（当前由 `random_navigation_fixed_pool` 临时覆盖为固定点位抽签，
见 §2）。两者统一走上述 Service/状态 Topic，Action 不直连
`/navigate_to_pose`。导航失败、超时、协议错误或取消时，不会继续执行到点后的
Stage 运动。

## 2. 地点文件与 Action 映射

当前工作站的 `$HOME/.ros/waypoints.yaml` 中实际标记为：

| YAML ID | 精确地点名 | x | y | orientation z | orientation w |
|---|---|---:|---:|---:|---:|
| `A` | 客厅 | 2.375 | 0.935 | -0.576048 | 0.817416 |
| `B` | 厨房 | 0.375 | -0.015 | -0.461810 | 0.886979 |
| `C` | 卧室 | 1.325 | -3.765 | 0.336186 | 0.941795 |
| `D` | 卫生间 | 3.175 | -2.565 | -0.525731 | 0.850651 |
| `E` | 充电桩 | 2.025 | -1.365 | 0.160182 | 0.987087 |

网页和板端必须使用同一份文件。BehaviorTree 加载其中实际标记的 ID/名称清单
用于语音目标确认和仲裁；waypoint_nav 加载同一文件中的 Pose。地点删除或改名后，
上游清单和本项目 `waypoint_nav.places` 都必须同步，不得用默认 A–E 推测。

Action 执行器的唯一配置入口是
[`config/navigation_waypoints.yaml`](../config/navigation_waypoints.yaml)。其
`waypoint_nav.places` 保存固定路由要发送的 YAML 精确名称；当前映射为：

| Action 内部语义槽 | 发送的 `place` | 当前服务端 `matched_id` |
|---|---|---|
| A（睡眠/home） | 卧室 | `C` |
| B（充电） | 充电桩 | `E` |
| C（食物） | 厨房 | `B` |
| D（排泄） | 卫生间 | `D` |
| E（清洁） | 客厅 | `A` |

waypoint_nav 允许 `place` 是精确 ID 或精确名称，并在受理结果中返回规范化的
`matched_id + place`。Action 会校验请求值至少命中返回的其中一项，不再错误要求
`matched_id == 请求名称`。根目录 [`waypoints.yaml`](../waypoints.yaml) 和
`ros2_pose.py` 只是历史手工工具，不参与生产下发。

随机配置是 `random_navigation_place: K`。`K` 与字符串 `"11"` 是 waypoint_nav
的同一保留入口，不是第十一张地点卡，也不得写入 `waypoints.yaml`。服务端
受理后统一返回 `matched_id="11"`、`place="随机点位"`，Action 客户端已显式允许
这一规范化。

> **临时覆盖（2026-09-20）**：`navigation_waypoints.yaml` 现启用
> `random_navigation_fixed_pool: [A, B, C, D, E]`。池非空时随机导航**不再**发送
> `K`，而是每次导航从池中随机抽一个点位，经 `waypoint_nav.places` 解析成与固定
> 路由完全相同的地点名后再下发。因此「随便走走」的落点必定是五个已标定点之一，
> 不再依赖服务端在 `/map` 上选点。池里的 ID 必须同时存在于 `waypoints` 和
> `waypoint_nav.places`，否则启动即报错。把列表清空
> （`random_navigation_fixed_pool: []`）即恢复由 waypoint_nav 生成随机目标的行为；
> `random_navigation_place: K` 保持不变，校验不接受把它改成固定点位。

执行器不会自动发布 `/initialpose`。AMCL 初始位姿应在整机启动或定位恢复流程中
设置，不能在每次行为开始时重置。

## 3. 首批行为路由

| 行为 | 点位 | 启用物理代理的原 Stage |
|---|---|---|
| `go_home` | A（演示 home） | `navigation` |
| `sleepOnSide` | A | `circle`, `prepare`, `sleep_pose`, `sleeping`, `wakeup` |
| `sleepNow` | A | 同上 |
| `restInPlace` | B | `recover` |
| `recharge` | B | `recharge` |
| `eatNormally` | C | `lower_head`, `head_up`, `lie_down`, `stand_up`, `lower_head_again`, `waist_twist` |
| `eatExcitedly` | C | 同上 |
| `seekFood` | C | 同上 |
| `seekFoodUrgently` | C | 同上 |
| `inspectDogFood` | C | 同上（它的 `target_approach` 不在此列） |
| `barkShortAlert` | D | `circle`, `action`, `exit`, `head_up` |
| `lickPaws` | E | `groom` |

`barkShortAlert` 名称与排泄动作语义不一致是上游新对照表的现状；本项目仍严格
按该表中的 `ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT` 等动作，将它路由到 D 点。

### 3.1 睡觉：到点后原地转一圈再趴下

`sleepOnSide` / `sleepNow` 的 `circle` Stage 在导航完成后立刻执行，是整条链的第
一步，只有一个候选 `ACT_CIRCLE_AROUND`。它刻意不放进 `prepare`：`prepare` 是
`random_one`，放进去就只有 1/6 的概率会转，达不到「到点、转一圈、再趴下」的固定
顺序。`prepare` 里原有的 `ACT_CIRCLE_AROUND` 已随之移除，候选总数不变。

Lite3 没有原生的「转一整圈」动作，因此 `ACT_CIRCLE_AROUND` 走移动模式角速度通道
（`0x21010135` 在原地模式下是偏航角姿态指令，在移动模式下是左右转弯的角速度），
以 1.0 rad/s 开环持续 6.3 s。开环是刻意的取舍：`/robot_status` 不携带 yaw，转
角只能计时而非测量，落点接近但不精确等于一圈。该动作在真机上仍未验收，首次使用
需要有人监护并保证周围净空——已有的后向超声门禁不覆盖旋转。

### 3.2 上厕所：到点后绕点走 3 圈，最后抬头表示完成

`barkShortAlert` 的 Stage 顺序是
`circle` → `action` → `exit` → `head_up`：

- `circle` 只有一个候选 `ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT`，导航完成后立刻执行；
- `action` 是蹲下排泄，`exit` 是原有的随机收尾三项；
- `head_up` 只有一个候选 `ACT_RAISE_HEAD`，是**最后一个**动作，代表执行完成。
  它刻意不并进 `exit`：并进去就只有 1/3 的概率会抬头，就不能当完成信号了。

和前两者不同，这里是**绕点走圈**而不是原地自转：狗一边前进一边转弯，移动模式下
`0x21010130`（前后速度）与 `0x21010135`（转弯角速度）合成半径
`linear_x / angular_z` 的圆。第一版是 `0.20 / 0.40`，即半径 0.5 m、3 圈 47.1 s，
看起来像慢吞吞地晃而不是绕圈；2026-09-18 按「圈速快 3 倍、半径 0.2 m」改成
`0.24 / 1.20`——半径正好 0.2 m，3 圈 = 3·2π/1.20 = 15.7 s。这两个数正好顶到
`limits`，而 `limits` 是**运行时钳位**：计划超限不会失败，只会被静默夹住，半径和
圈速一起被改掉，所以抬 `limits` 和抬计划必须同时做（另见
[LITE3_ROS2_INTEGRATION.md](LITE3_ROS2_INTEGRATION.md) §4.4，launch 里的参数默认值
也会覆盖配置）。0.2 m 半径意味着狗几乎是拧着走、内侧腿基本不迈，验收要在开阔地面
做。同样是开环计时；此外这是**第一个由后端自己持续向前驱动**的计划，因此它额外
声明了 `minimum_forward_clearance_m: 0.60`——后向超声门禁不覆盖前进方向，该阈值是
可选项，只对本计划生效。狗绕圈时朝向会扫过整圈，所以这条门禁实际要求四周净空。

`ACT_RAISE_HEAD` 是 2026-09-18 新增的动作（`0x21010130` 取负值；协议里正值是
低头），Lite3 与 Go2 都有对应代理：Lite3 走抬头俯仰，Go2 挂到已有的
`alert_pose`（本来就是负俯仰）。它的俯仰取值和全部 9 个低头动作一起从 7500 抬到 **20000**：7500 只占俯仰
轴可用行程的 3.6%（死区 6553），真机上根本看不见，详见
[LITE3_ROS2_INTEGRATION.md](LITE3_ROS2_INTEGRATION.md) §4.3。保持时间也一起调过：
低头代理 1.0 → **2.0 s**（20000 的幅度看得见了，但 1 s 一闪而过读不出来），抬头
**2.5 s**（它是完成信号，要比它所镜像的低头更慢、更像一个刻意动作）。这两档时长
把 14 个俯仰动作的单元超时顶到 5.0 s（`ACT_LOWER_HEAD_FLOP_EARS_WAG_TAIL` 与
`ACT_OWNER_UNHAPPY` 到 6.0 s），因为保持、模式切换和稳定等待共用同一个单元预算，
而趴下态的入口起立还会再花 2.2 s；`ACT_EXPRESS_MISS_YOU` 原先自带 4.0 s 的短预算，
在同一轮里一并抬到 5.0 s，否则起立恢复会把它刚做出来的手势吃掉。

**时长注意**：这三个动作加起来远超上游原有的 60 s goal 预算（该预算同时覆盖
导航），因此 `MarsDogTree/config/behaviors.yaml` 的 `barkShortAlert.timeout_sec`
已从 60.0 抬到 **180.0**。转圈缩短到 15.7 s 后这个预算对转圈有大冗余，但导航本身
在十几到几十秒量级，不要因此把它压回小值。

`walk_to_random_point` 与 `go_out_to_play` 都走随机导航，分别保留
`ACT_NAV_WALK_RANDOM` 和 `ACT_NAV_GO_OUT_TO_PLAY`；目标从
`random_navigation_fixed_pool` 抽签（池为空时回到保留随机目标 `K`）。可通行区域与障碍判定属于 waypoint_nav/
Nav2，Action 不再维护随机候选固定点或多边形。导航适配器
未启用时，这三个行为因 `requires_controller=true` 失败，不会模拟成功。

Stage 仍然从原候选列表选择精确 `ACT_*`。`navigation_waypoints.yaml` 的
`stage_actions` 声明哪些动作属于导航后的 Stage；动作会以原 `ACT_*` ID 交给
当前 Go2 或 Lite3 后端的 `execute_step`，由对应平台的动作配置执行。
Feedback 和 Result 继续保留原动作 ID。配置校验要求每个已路由 Stage 的
所有候选动作都在 `stage_actions` 中，避免随机选中未接入的动作。

睡眠行为的 `sleeping` Stage 显式声明 `motion_state: stationary`。执行器在进入
该状态时立即发送停止命令，并在该 Stage 内保持静止；三个睡眠中
动作仍作为精确 `ACT_*` 出现在 Feedback/Result 中。进入后续 `wakeup` Stage 时，
运动状态自动恢复为 `active`，允许执行起身代理动作。

### 3.3 进食：到点后的六步仪式

五个进食行为（`eatNormally`、`eatExcitedly`、`seekFood`、`seekFoodUrgently`、
`inspectDogFood`）在到达 C 点厨房后执行同一套固定仪式，每一步都是单候选
`random_one`，顺序即操作员看到的顺序：

```text
lower_head       低头   ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD
head_up          抬头   ACT_RAISE_HEAD
lie_down         趴下   ACT_BASIC_LIE_DOWN
stand_up         站立   ACT_BASIC_STAND
lower_head_again 低头   ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD
waist_twist      扭腰   ACT_TWIST_WAIST_LEFT_RIGHT
```

原表里 `prepare` / `eating` / `interaction` / `exit` / `seek_food` / `inspect`
这些阶段被整体替换，只剩 `inspectDogFood` 的 `target_approach` 留在行为定义里
（order 1，`failure_policy: abort`），路由里不列出——导航本身就是它的「到达点位」。
两侧低头共用俯仰代理族（`low_semantic_pitch`）的锚点动作，语义上正是「低头闻食盆」；
扭腰是这一次唯一新增的 `ACT_*`，Lite3 用左右各 1.6 s 的偏航代理，
Go2 用 `waist_twist` 组（`move` z ±0.45）。

**预算**：Lite3 上这套仪式本身约 20 s（含低头 2.0 s ×2、抬头 2.5 s、扭腰 3.2 s
与各步的模式切换／姿势恢复），而上游没有给进食行为单独的 `timeout_sec`，走的是
`_default_timeout_for_level(3)` = 30 s，并且同一个预算还要覆盖导航。要在真机上跑
完整流程，必须在 `MarsDogTree/config/event_intent_map.yaml` 的饥饿事件（或它的
`dog_food` / `no_dog_food` 路由项）上给出足够的 `timeout_sec`（建议 180 s，与
`barkShortAlert` 一致）。

## 4. 启动条件

所有进程必须位于与底盘相同的 Domain 1：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

启用前确认：

```bash
ros2 service type /waypoint_nav/task
ros2 topic info /waypoint_nav/status -v
ros2 node list
ros2 topic info /cmd_vel -v
```

Service 类型必须是 `marsdog_voice_interaction/srv/VoiceTask`，状态类型必须是
`std_msgs/msg/String`。所有点位导航都要求 `waypoint_nav`、其下游 Nav2、定位、
地图和所选底盘节点正常。停止 `teleop_twist_keyboard` 和手工
`ros2 topic pub`，避免多个速度源抢占 `/cmd_vel`。

通过 launch 启动：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=lite3 \
  lite3_enabled:=true \
  navigation_enabled:=true \
  waypoint_nav_service_name:=/waypoint_nav/task \
  waypoint_nav_status_topic:=/waypoint_nav/status \
  waypoint_nav_cancel_confirmation_timeout_sec:=5.0 \
  waypoint_nav_terminal_retention_sec:=86400.0 \
  navigation_preempt_lock_wait_sec:=8.0 \
  navigation_result_timeout_sec:=300.0
```

底盘与导航开关必须同时为 `true`。导航开启而底盘未启用时，节点会拒绝启动，
避免出现“导航成功但到点动作静默不执行”。

### 情绪类移动契约

- `expressCalm/Joy/Curiosity/Excitement/Anxiety/FearAlone`：触发移动时发送
  `place="K"`（池非空时为抽中的固定点位地点名），由 `waypoint_nav` 在实时地图中
  选点或直接解析该固定点，到达后执行表达动作。六个 Alone 行为另有
  `random_navigation_in_place_probability`（当前 0.5）决定是否原地不动。
- 对应的 `*WithHuman` 不进入随机导航。它们先执行必需的
  `approach_human` Stage，以 Goal 中不可变的 `vision_epoch + target_id` 锁定
  人员，保持实时视觉闭环并停在安全距离，成功后才进入 `expression`。
- 人员目标缺失、失鲜、切换 epoch、测距无效、取消或超时都会发布零速并让
  `WithHuman` 失败，不会静默降级成 `Alone`。

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
waypoint task=action:nav-sleep-001:waypoint state=RUNNING code=QUEUED
waypoint task=action:nav-sleep-001:waypoint state=RUNNING code=DISPATCHED
waypoint task=action:nav-sleep-001:waypoint state=RUNNING code=NAV2_ACCEPTED
waypoint task=action:nav-sleep-001:waypoint state=SUCCEEDED code=NAV2_SUCCEEDED
Stage 1/4: prepare
Go2/Lite3 stage action -> platform controller
```

Goal 的 `timeout_sec` 同时限制导航等待，必须覆盖现场最长导航时间；否则执行器
会向 `waypoint_nav` 发送带 `target_task_id` 的取消请求，并等待原任务终态；受理
取消本身不等于已停止。`K` 随机路径使用相同的定向取消和终态确认逻辑。

## 6. 取消、急停和速度源边界

- 普通 Action Cancel 只取消当前 Action 对应的 `target_task_id`；目标不匹配不得
  影响其他任务。
- 取消 Service 返回受理后，Action 仍等待该任务的 `INTERRUPTED`/`FAILED` 等真实
  终态，才释放执行锁。5 秒未见终态时发 `query` 恢复可能漏掉的状态；替代 Goal
  最多等待锁 8 秒，超时后失败关闭。
- `emergency_stop` 请求取消活动导航并立即发布冗余零 Twist，但行为 Result 同样
  不把“取消已受理”误报成“导航已停止”。
- 固定导航期间不执行 Stage 动作；只有匹配 task_id 的
  `SUCCEEDED/NAV2_SUCCEEDED` 才进入对应平台的动作 Stage。
- 重复 task_id 在 Action 进程内不会二次下发；服务端仍须持久化终态 24 小时，
  并在节点重启后通过 `query` 返回运行态或终态。完整终态过期后仍须保留
  task_id tombstone；旧 ID 返回 `TASK_ID_RETIRED`，不得重新导航。
- 当前是顺序控制，不实现 velocity mux。后续如果遥控、Nav2 和动作层需要并行
  保留，应引入 `twist_mux`，分别使用独立输入 Topic。
- “充电桩”只导航到预充电位并停车，不包含最后厘米级自动对桩；对桩应由充电控制器
  单独完成。

### 6.1 Action 与 waypoint_nav 的 JSON 契约

三类请求继续复用 `marsdog_voice_interaction/srv/VoiceTask`。`task_id` 是请求自身
ID；`target_task_id` 只出现在 `cancel/query` 的 `params_json` 中：

```jsonc
// goto_place params_json
{
  "protocol_version": "1.0",
  "client_id": "marsdog_action_executor",
  "place": "厨房",
  "timeout_sec": 120.0,
  "preempt": false,
  "terminal_retention_sec": 86400
}

// cancel params_json
{
  "protocol_version": "1.0",
  "client_id": "marsdog_action_executor",
  "target_task_id": "action:goal-001:waypoint"
}

// query params_json
{
  "protocol_version": "1.0",
  "client_id": "marsdog_action_executor",
  "target_task_id": "action:goal-001:waypoint"
}
```

随机导航的 `goto_place` 请求只将上例的 `place` 改为字符串 `"K"`。
其他协议字段、task ID、超时、取消和查询逻辑完全相同。服务端的规范化受理
结果为：

```json
{
  "accepted": true,
  "matched_id": "11",
  "place": "随机点位",
  "message": "accepted"
}
```

`goto_place` 受理响应沿用：

```json
{
  "accepted": true,
  "matched_id": "B",
  "place": "厨房",
  "message": "accepted"
}
```

`cancel` 受理响应必须回显目标，目标不存在或不匹配时必须 `success=false`，且不能
影响当前导航：

```json
{
  "accepted": true,
  "target_task_id": "action:goal-001:waypoint",
  "message": "cancel accepted"
}
```

`query` 成功响应的 `result_json` 为以下二者之一。`status` 的结构与状态 Topic
完全相同；无记录只表示该 ID 不在持久化存储中，不能伪造导航成功：

```json
{"found": false, "target_task_id": "action:goal-001:waypoint"}
```

```json
{
  "found": true,
  "target_task_id": "action:goal-001:waypoint",
  "status": {
    "task_id": "action:goal-001:waypoint",
    "task_type": "goto_place",
    "state": "SUCCEEDED",
    "code": "NAV2_SUCCEEDED",
    "matched_id": "B",
    "place": "厨房",
    "message": "arrived",
    "safe_to_interrupt": false,
    "sequence": 3
  }
}
```

状态组合如下；只有最后一列的唯一成功组合允许 Tree 继续到点后的 Stage：

| `state` | 允许的 `code` | 终态 | Action 结果 |
|---|---|---:|---|
| `RUNNING` | `QUEUED` | 否 | 尚未下发，可原子撤回时允许取消 |
| `RUNNING` | `DISPATCHED` | 否 | UUID 与发送意图已落盘，Nav2 是否收到尚未确定 |
| `RUNNING` | `NAV2_ACCEPTED` | 否 | Nav2 已接受，继续等待 |
| `RUNNING` | `RECOVERY_REQUIRED` | 否 | 重启恢复尚未确认，阻塞新导航 |
| `SUCCEEDED` | `NAV2_SUCCEEDED` | 是 | 成功，继续 Stage |
| `FAILED` | `GOAL_REJECTED`, `NAV2_EXCEPTION`, `INTERNAL_ERROR`, `TIMEOUT`, `NAV2_FAILED`, `NAV2_UNKNOWN` | 是 | 失败 |
| `INTERRUPTED` | `CLIENT_CANCELLED`, `PREEMPTED`, `NAV2_CANCELED` | 是 | 取消/失败，不继续 Stage |

`sequence` 可选是为了兼容现有板端；新服务端应提供单任务严格递增序号。Action
发现序号缺口会通过 `query` 恢复；倒退、同序号内容冲突、未知状态、非法组合、
`matched_id/place` 冲突均按协议失败，并定向取消本任务。`safe_to_interrupt` 缺失时
按 `false` 处理。`DISPATCHED`、`RECOVERY_REQUIRED` 和所有终态都必须为
`false`；`QUEUED` 只有在服务端可原子确认从未下发时才可为 `true`。

服务端需按 `task_id` 幂等。完整运行态和终态记录默认保留 24 小时并落盘原子
更新；仅保存在内存不满足本契约。完整记录过期后仍保留不自动过期的轻量
task_id tombstone，重复 `goto_place` 返回 `TASK_ID_RETIRED`，避免旧 ID 再次触发
导航。若 Action 重启后重发同一 task_id，服务端拒绝重复请求，Action 随后用
`query` 恢复仍在保留期内的真实状态。

服务端节点级阻塞模式命名为 `RECOVERY_BLOCKED`，对应未决任务对外发布
`RUNNING/RECOVERY_REQUIRED`。恢复通道得到 Nav2 `STATUS_UNKNOWN` 时不得推导任务
已经停止，必须继续阻塞新导航。只有板端完成“发送前落盘、发送窗口崩溃、按 UUID
恢复/取消”的崩溃窗口测试后，才能声明重启恢复可用。

取消规则按下发边界处理：`QUEUED` 任务若能原子确认从未下发，可直接写入取消
终态；从 `DISPATCHED` 开始必须核对 Nav2 真实结果。下发与取消竞争时，如果 Nav2
成功先发生，终态必须是 `SUCCEEDED/NAV2_SUCCEEDED`，不能改写为取消。取消响应
只表示请求已受理，Action 仍以原任务的 Topic/query 终态释放执行锁。

## 7. 仿真页面

页面仍只发送 `/execute_behavior` Goal，不直接调用 `/waypoint_nav/task`、
`/goal_pose` 或 `/cmd_vel`。导航阶段 Feedback 的 `current_action` 为
`WAYPOINT_NAV/<code>`，并只接受本任务的状态；到点后继续使用行为树选中的精确
`ACT_*`。`safe_to_interrupt=true` 只透传服务端明确标记为可安全取消的运行阶段；
旧服务端未提供该字段时一律为 `false`。

页面如需更多导航细节，可展示 Action Feedback，不应绕过 Action 直接消费其他
任务的状态，也不能把 waypoint 名称伪装成 `ACT_*`。
