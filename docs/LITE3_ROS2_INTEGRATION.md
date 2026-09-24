# Lite3 ROS2 动作集成

## 1. 执行链

```text
behavior_name -> Stage -> ACT_* -> Lite3ChassisBackend
                                  -> /cmd_vel
                                  -> /simple_cmd
                                  <- /robot_status
```

Lite3 不复用 Go2 SportMode。Nav2、视觉目标接近、唤醒角旋转和
UWB 跟随继续使用平台无关的闭环适配器。

## 2. 安全边界

- 禁止 APP 语音通道 `0x21010C0A`；
- 禁止软急停 `0x21020C0E`，`emergency_stop` 只取消控制器并冗余发布零速；
- 禁止起立/趴下 toggle 作为普通 `simple` 映射；它只能由读取新鲜状态、固定方向、
  单次发送并确认终态的 `target_posture` 闭环调用；
- 禁止翻身、扭身跳、后空翻和向前跳进入行为映射；
- 禁止横滚轴 `0x21010131`；机背算力卡和语音设备必须保持水平安全边界；
- `/robot_status` 缺失、过期、低电量、失控保护或忙态时拒绝动作；
- `/simple_cmd` 没有 `motion_sender` 订阅者时动作失败；
- 不依赖只代表下发确认的 `/lite3/action_result`；
- 简单动作观察到动作态和终态后才成功；
- 未配置动作保持 `unsupported`，不回退为 mock success。

配置默认关闭后端、开启安全逻辑代理、关闭所有未验收运动：

```yaml
enabled: false
allow_proxies: true
allow_unverified: false
accepted_unverified_actions:
  - ACT_BASIC_LIE_DOWN
  - ACT_BASIC_STAND
  - ACT_CIRCLE_AROUND
  - ACT_EXPRESS_MISS_YOU
  - ACT_GETUP_STRETCH
  - ACT_GUARD_DOOR
  - ACT_INTERACT_GIVE_PAW
  - ACT_RAISE_HEAD
  - ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT
  - ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD
  - ACT_STRETCH
  - ACT_TRICK_SPIN
```

白名单项同时充当共享物理 `plan_id`；多个 `ACT_*` 只有在 sequence 完全相同时才
复用同一个 plan。由于 `allow_unverified` 默认仍为 `false`，配置白名单本身不会让
未验收动作自动运行。

## 3. 速度所有权

动作执行器、Nav2 和 UWB 都以 `/cmd_vel` 为最终速度入口。Lite3 补流桥和
独立 `/lite3/action` 入口必须关闭：

```bash
ros2 launch transfer lite3_bringup.launch.py \
  use_bridge:=false use_action:=false
```

外部运动开始前，后端发送 `0x21000C03` 进入 Vision Mode；结束、取消或异常退出时
发布五条零 Twist，再发送 `0x21000C02` 交回 Joystick Mode。随后必须等待新的
`/robot_status` 连续处于 `basic_state=6, gait_state=0, motion_state=0`，才允许执行
行为的第一个 Stage；默认等待 5 个观测、总超时 2 秒。超时保持失败关闭，不绕过
忙态、过渡态或低电量门禁。

Nav2 和 UWB 不能同时运行。部署端还应确认没有额外节点绕过 Action 抢发
`/cmd_vel`。Lite3 后端会在每次自动运动预检时查询 ROS 图；发现根命名空间下
存在 `/lite3_twist_bridge`、`/lite3_action` 或 `/uwb_behavior_controller_node`
时，拒绝自动运动并返回 `lite3_control_conflict`。零速停止出口仍然可用。

`/uwb_behavior_controller_node` 是 2026-09-20 起的新条目：它从启动起就以 20 Hz
持续发 `/cmd_vel`（空闲时为零），会把 `ACT_CIRCLE_AROUND` 这类开环 twist 打散。
**跟随自己例外**——跟随就是要把 `/cmd_vel` 交给这条链，所以 UWB 跟随适配器调
`prepare_navigation(allow_uwb_chain=True)` 显式让位；该参数默认 `False`，其它所有
动作一律照常拒绝。豁免只摘掉这一个名字，报告里若还有别的竞争节点仍然失败。

所有权检查是**进入时**的闸门。`_runtime_motion_ready` 是每 tick 的廉价检查，
不做所有权检查，因此「动作已经在跑、链才启动」这种情况不会被拦下。

接口细节、四终端部署与排障见
[UWB_FOLLOW_ROS2_INTERFACE.md](UWB_FOLLOW_ROS2_INTERFACE.md)。

## 4. 动作等级

| 等级 | 默认 | 说明 |
|---|---|---|
| `exact + verified` | 开 | 已验收的零速保持、后退和安全停止 |
| `proxy + verified` | 开 | 已验收的限幅代理运动；结果标记为 `proxy_motion` |
| `exact/proxy + unverified` | 关 | 必须同时打开总门并将精确 `ACT_*` 或共享物理 plan ID 加入验收白名单 |
| 危险简单指令 | 永久关 | 配置校验与运行时双重拒绝 |

182 个 `ACT_*` 均有明确路由：169 个进入 Lite3 backend，13 个继续使用导航、
视觉接近、UWB 跟随、唤醒朝向或安静控制器（其中 2 个是永久拒绝的
`ACT_BONE_DANCE` / `ACT_JUMP_PAW`）。Lite3 不具备的生物动作和物体操作
按语义使用站/趴目标姿态闭环或可观察的限幅姿态代理，不能据此声称已坐下、
抓取、携带、释放或完成充电。

| Lite3 计划 | ACT 数 | 实际效果 |
|---|---:|---|
| 目标姿态闭环 | 23 | 17 个趴下语义、6 个起立/起床语义；单次 toggle 后确认稳定终态 |
| 零速控制语义 | 6 | 等待、保持和安全停止，保持零运动 |
| 可观察姿态代理 | 152 | 俯仰、偏航或机身高度，自动回正；禁止横滚 |
| Lite3 原生动作代理 | 4 | 趴下时打招呼、站立时扭身体 |
| 有界 Twist | 6 | 睡前原地转圈、上厕所绕点走圈，以及精确后退与代理旋转/后退 |
| 专用闭环控制器 | 13 | 导航、视觉目标接近、UWB 跟随、声源朝向或安静控制；其中 2 个永久拒绝 |

配置里的每个代理都保留精确 `ACT_*` ID；结果同时报告 `executed_plan`、
`fidelity`、`semantic_effect`、实际 `physical_posture` 和终态 `/robot_status`。

### 4.1 入口期姿势恢复

`require: stand` 曾是纯前置条件，这会让趴下的狗再也起不来。2026-09-18 真机上的
表现是：平静行为（`expressCalmAlone` 的 5 个候选里 `ACT_SPLOOT` 是趴下类，其余
4 个需要站立）随机选中趴下类之后，`basic_state` 停在 1，之后每一次站立类动作都
以 `lite3_posture_mismatch:require=stand,basic_state=1` 失败——而且没有任何一步
会把狗重新站起来，只能人工介入。

现在 backend 在**步骤入口**发现「狗在趴下态 + 本步要求站立」时，先发一次
`0x21010202` 起立并确认稳定站立，再执行本步：

- 只恢复**趴下态**，且只针对 `require: stand`；反向（站着要求 `lie`）仍然拒绝。
- 恢复受该步骤自己的 deadline 约束，不会超出本步预算。
- 其它前置条件失败（低电量、失控保护、步态忙等）照常原样上报，不会被起立掩盖。
- **流式运行期的检查保持严格**：动作执行途中姿势变化仍然中止该动作，不会中途站
  起来。

**导航预检也算一个入口。** `prepare_navigation`（把 `/cmd_vel` 交给 Nav2 或 UWB 链之前
的那道闸门）同样接了这套恢复。2026-09-20 真机上 `expressCalmAlone` 走随机导航路线时
卡的就是这里：行为在预检就被判失败，**一个 step 都没跑到**，于是步骤入口的恢复永远
没机会执行，狗保持趴下、每次选中该行为都重复同一条 `lite3_posture_mismatch`。恢复
**不放进 `_motion_ready`**：那个检查还被 `publish_velocity` 的每 tick 流式路径共用，
那里必须保持严格（见上面最后一条）。

`posture_recovery_timeout_sec`（`config/lite3_actions.yaml`，默认 6.0）是这一次
起立的预算；真机实测起立 2.2 s。

### 4.2 前进方向净空门禁

`minimum_backward_clearance_m` 一直是配置里的默认值，因为每个后退计划都需要它；
前进方向此前只由 Nav2 驱动，不在后端职责内。上厕所的绕点走圈是第一个由后端自己
**持续向前**驱动的计划，因此 `_clearance_ready` 增加了对应的前进分支，由计划的
`minimum_forward_clearance_m` 显式开启，缺失时视为 0（不改变任何既有计划）。
入口与运行中都会检查，触发时返回 `lite3_forward_clearance`。

未验收动作需要双重授权：

```bash
lite3_allow_proxies:=true
lite3_allow_unverified:=true
```

并且 `config/lite3_actions.yaml:accepted_unverified_actions` 必须包含当前精确
`ACT_*` 或其共享 `plan_id`。共享 plan 只复用完全相同的物理 sequence；全局开关
本身不会放开未列入白名单的计划。

### 4.3 俯仰轴的取值与可见性

`0x21010130`（俯仰）、`0x21010135`（偏航）、`0x21010102`（机身高度）都是**死区轴**：
motion controller 把 `|value| ≤ dead_zone` 当作 0（俯仰 6553、偏航 9553、机身高度
20000），所以取值下界不是 0 而是死区边。

2026-09-18 真机上暴露的问题：低头代理用 7500，只比死区边高出 947——操作员在真狗上
**完全看不到**这个低头，也看不到镜像的抬头，而上厕所流程本身判的是 success。现在四处
俯仰定义（`ACT_EXPRESS_MISS_YOU`、`ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD`、
`low_semantic_pitch` 分组共 9 个低头动作，以及 `ACT_RAISE_HEAD`）统一改为 20000。
保持时间同一轮也一起调长，因为幅度看得见之后 1 s 的保持仍然读不出动作：低头代理
1.0 → **2.0 s**，`ACT_RAISE_HEAD` 1.0 → **2.5 s**（它是完成信号，要比它镜像的低头
更慢、更像一个刻意动作）。

时长和单元预算是一回事，不能只改一头。一个姿态步骤依次要付：入口起立恢复（真机实测
2.2 s）、两次模式切换及其稳定时间、一个控制周期、保持本身、最后是稳定等待；其中恢复
和稳定等待都被该步骤的 deadline 截断，所以**预算不够不会报错，只会让 deadline 落在保持
中途**，操作员看到的依然是变短的动作。因此这一轮把 10 个俯仰动作的单元超时抬到 5.0 s
（`ACT_LOWER_HEAD_FLOP_EARS_WAG_TAIL`、`ACT_OWNER_UNHAPPY` 抬到 6.0 s），
`ACT_EXPRESS_MISS_YOU` 本来就自带 4.0 s 的短预算（2.0 + 0.25 + 2.2 = 4.45 已经不敷），
一并抬到 5.0 s。同样的道理适用于下面 4.4 的偏航族：整族保持时间 0.8 → 1.6 s 的那一轮，
有 17 个原本 3.0/4.0 s 的单元超时必须跟着抬到 5.0 s。
`tests/test_lite3_backend.py:test_every_pose_hold_fits_inside_its_unit_budget`
守这条不变式。

**不要把「超出死区多少」当成可见性判据。** 偏航 10500 同样是超出死区 947，操作员的
描述是「明显扭转，幅度清晰」；俯仰 7500 则完全看不见。俯仰这条轴就是更钝。所以这次
改动只动俯仰四处，偏航 10500 与机身高度 −21000 保持不变。

20000 这个新值**尚未在真机上验收**，而且机背还装着算力卡和语音设备（横滚轴
`0x21010131` 就是因此被永久禁止的），俯仰每往上一档都要有人监护重测。

### 4.4 速度钳位是 `limits`，而 launch 默认值会盖过它

`config/lite3_actions.yaml:limits` 不是文档而是运行时钳位：`publish_velocity` 会把每个
Twist 夹到这三个值。超限的计划**不会失败**，只会静默按钳位值跑——对曲线计划就是半径
和圈速同时变，且不产生任何错误或日志。

更隐蔽的是 `launch/action_executor.launch.py` 把同样的数字写死成
`lite3_max_linear_x` / `lite3_max_angular_z` 参数的默认值，而参数**优先于**从 `limits`
派生的声明值。两处必须一起改，只改一处等于钳位还是旧值。`tests/test_lite3_backend.py`
里有两条测试守这件事：所有 `twist` 计划段不得超出 `limits`，以及 launch 默认值必须
等于 `limits`。

当前值 0.24 / 1.20 是为 0.2 m 的上厕所绕圈（`v = ω · r`）抬上去的；这份配置里没有
别的计划超过 0.20 m/s 或 0.50 rad/s。

### 4.5 偏航代理是整族共享的，改一个等于改 114 个

偏航代理（`0x21010135`，取值 10500）不是逐个定义的动作，而是一份被 `plan_id`
共享的物理计划：`ACT_GUARD_DOOR` 是它的锚点，另外 5 个定义（`scan_semantic_yaw`、
`quiet_attention_yaw`、`safe_body_yaw_fallback` 三个分组，加 `ACT_WAG_TAIL` 和
`ACT_TRICK_ROLL_OVER`）都通过 `plan_id: ACT_GUARD_DOOR` 引用它，加载器要求这 6 处与
锚点**逐字节完全相同**。所以**没有办法只把某一个动作的偏航做长**——平静行为的扭腰
读起来像抽一下而不是一个动作，只能整族一起改：0.8 → **1.6 s**（2026-09-18），
一次性影响全部 114 个 `ACT_*`，而 `ACT_TWIST_WAIST_LEFT_RIGHT`（进食仪式的收尾扭腰）
刻意不加入这一族：它要左右各扭一次，方向相反，只能自带一份独立计划。同轮还有 17 个原本 3.0/4.0 s 的单元超时随之抬到
5.0 s，理由见 §4.3。

### 4.6 进食仪式的收尾：左右各扭一次腰

`ACT_TWIST_WAIST_LEFT_RIGHT` 是 2026-09-18 进食仪式改写时新增的唯一动作，也是
`lite3_actions.yaml` 里第一个多步 `pose` 计划：

```yaml
ACT_TWIST_WAIST_LEFT_RIGHT:
  sequence:
    - {type: pose, cmd_code: 0x21010135, value: 10500, require: stand, duration_sec: 1.6}
    - {type: pose, cmd_code: 0x21010135, value: -10500, require: stand, duration_sec: 1.6}
```

两个方向的幅度都沿用真机验证过的偏航取值 10500（§4.5），每侧 1.6 s 与该族一致；
不能共享 `plan_id` 的原因见 §4.5——锚点是单向字节相同的序列，而这里要一正一负。
预算按「两步各自付模式切换与控制周期、入口起立恢复只付一次」算：
1.6 + 1.6 + 2×0.25 + 2.2 = **5.9 s**，单元 `timeout_sec` 取 **8.0**，
`tests/test_lite3_backend.py::test_every_pose_hold_fits_inside_its_unit_budget`
把这条不变式钉住（该测试同时覆盖所有全 `pose` 计划，不只单步的）。
Go2 对应使用 `waist_twist` 动作组。

## 5. 启动

```bash
source /home/cat/Lite3_ROS/install/setup.bash
source /home/cat/ros2_ws/install/setup.bash

ros2 launch transfer lite3_bringup.launch.py \
  use_bridge:=false use_action:=false

ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=lite3 \
  lite3_enabled:=true \
  navigation_enabled:=true
```

不使用 Nav2 时设 `navigation_enabled:=false`。UWB 首次启动必须先按 Lite3 文档以
`enable_motion:=false` 空跑验证。

## 6. 验收顺序

1. 确认 `/robot_status` 约 100 Hz，电量不低于 25%。
2. 确认 `/simple_cmd` 和 `/cmd_vel` 只有预期消费者。
3. 单独验证 `lie_down` 与 `stand_up`：目标已满足时不发 toggle；需要切换时只发一
   次 `0x21010202`，并观察过渡态及连续稳定终态。超时不得自动重发。
4. 验证 `hold_position`、`wait_in_place`、代理元数据和停止出口。
5. 验证 Nav2 到点后先出现稳定状态，再启动第一个行为 Stage；失败结果应包含
   `lite3_navigation_settle_timeout` 或具体状态门禁原因。
6. 验证低速 `back_up`，检查后方超声、模式切换、运行时状态看门狗和最终零速。
7. 原始 toggle、翻滚和跳跃指令不进入通用行为映射验收。
8. 姿态轴如需引入，先用独立计划逐轴验收，再将单个 `ACT_*` 加入验收白名单。
9. 姿态轴采用有界的 ≥20 Hz 连续发送并最终发 0；不得无限循环，也不能改成单帧完成判定。
10. 验证入口期姿势恢复：先把狗趴下，再触发一个 `require: stand` 的动作，应看到
    先发一次 `0x21010202`、确认站立后再执行；同时确认站着要求趴下的动作仍被拒
    绝，低电量时不会被起立掩盖。**导航路线要单独测一遍**：趴下后触发一个在
    `random_navigation_behaviors` 里的行为（如 `expressCalmAlone`），应看到起立发生在
    进入 Vision Mode 之前，而不是 `Chassis rejected navigation preflight`。
11. 睡前转圈（`ACT_CIRCLE_AROUND`）目前是**未验收**的开环动作：`/robot_status`
    不带 yaw，转圈靠计时（1.0 rad/s × 6.3 s ≈ 一圈），落点会有偏差。首次上真狗
    必须在有人监护、周围净空的前提下单独验收，再保留在
    `accepted_unverified_actions` 白名单里。现有后向超声门禁不覆盖旋转。
12. 上厕所绕点走圈（`ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT`，0.24 m/s × 1.20 rad/s
    = 半径 0.2 m，3 圈 15.7 s）同样是**未验收**的开环动作。0.2 m 半径意味着狗几乎是
    拧着走、内侧腿基本不迈，绊脚风险高于第一版的 0.5 m，首次上真狗必须在开阔地面、
    有人监护下单独验收，并确认 `minimum_forward_clearance_m: 0.60` 的刹停距离够。
    抬头 `ACT_RAISE_HEAD` 需确认负俯仰方向正确、回正到位、保持 2.5 s 足够读成一个
    完成信号，而且操作员在正常距离上**看得见**抬头的幅度——第一版 7500 就是看不见
    才改的。低头代理保持 2.0 s，同一轴同一幅度，验收时可一起看。
13. 上游 `barkShortAlert.timeout_sec` 必须 ≥ 180 s（`MarsDogTree/config/behaviors.yaml`）。
    该预算同时覆盖导航与全部 Stage。转圈从 47.1 s 缩到 15.7 s 后它对转圈有大冗余，
    但导航本身在十几到几十秒量级，不要因此把它压回 60 s。
14. 进食仪式（`eatNormally` 等五个行为，§3.3 / `NAV2_WAYPOINT_INTEGRATION.md`）首次
    上真狗按步验收：低头 2.0 s 与抬头 2.5 s 是否看得见、趴下与站立是否到位、
    收尾的左右各一次扭腰（`ACT_TWIST_WAIST_LEFT_RIGHT`，±10500 × 1.6 s）方向与
    幅度是否读得出，以及整条仪式在导航之后能否在 goal 预算内跑完（Lite3 上仪式本身
    约 20 s，上游必须给出覆盖导航 + 仪式的 `timeout_sec`）。

软件测试不等于四足姿态、步态稳定性或真机安全验收。

## 7. 关键文件

- `config/lite3_actions.yaml`：动作计划、代理分组和安全门；
- `marsdog_action_executor/adapters/lite3_backend.py`：状态机和 ROS 传输；
- `config/lite3_actions.yaml`：主人靠近命令走单次视觉定位和 Nav2 路由；
- `tests/test_lite3_backend.py`：无 ROS、无真机测试。
