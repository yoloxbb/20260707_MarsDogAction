# 随机点到达后的短组合

`walk_to_random_point` 和 `go_out_to_play` 仍先向 waypoint_nav 提交 `place="K"`。
收到真实成功终态并停稳后，执行 `navigation` 确认 Stage，再从新增的
`arrival_expression` Stage 随机选一套组合。导航失败、取消、超时或终态未知时
不会进入表达阶段。

六种 `express*Alone` 在现有 `expression` 候选池中增加组合。原有候选与各自
0.5 的原地概率保留；抽中原地时也可以执行这些组合。`*WithHuman`、
`*InPlaceWithHuman` 和 UWB 长期 `play_alone` 的动作池不变。

## 完整动作组合

每个 `ACT_ROAM_*` 是一套硬件后端执行的完整序列，仍由 `random_one` 选中一套，
内部按顺序执行。任一段失败或取消即停止，不把缺失的控制器作为模拟成功。
组合没有食物、动物或人物目标条件。

| Action | 动作顺序 | 单组合预算 |
|---|---|---:|
| `ACT_ROAM_LOOK_DOWN_UP` | 低头 → 抬头 | 20 秒 |
| `ACT_ROAM_SCAN_HEAD_UP` | 向左观察 → 向右观察 → 抬头 | 20 秒 |
| `ACT_ROAM_STRETCH_SCAN` | 舒展 → 向左观察 → 向右观察 | 20 秒 |
| `ACT_ROAM_TWIST_STRETCH` | 左扭腰 → 右扭腰 → 舒展 | 20 秒 |
| `ACT_ROAM_LIE_STAND` | 趴下 → 停留 1 秒 → 站立 | 20 秒 |
| `ACT_ROAM_LOWER_HEAD_PAUSE` | 短低头 → 还原 → 静止 1.2 秒 | 12 秒 |
| `ACT_ROAM_OBSERVE_PAUSE` | 短左观察 → 短右观察 → 静止 1.2 秒 | 12 秒 |
| `ACT_ROAM_HEAD_UP_TWIST` | 抬头 → 左扭腰 → 右扭腰 | 20 秒 |

上述预算是上限，包含控制模式切换、姿态确认与入口恢复时间；实际动作时长更短。
每个成功组合以站立结束。取消趴起组合时不会为了完成组合再发送站立动作。

| Behavior | 增加的组合 |
|---|---|
| `walk_to_random_point` | 低头抬头、左右观察抬头、舒展观察、趴起 |
| `go_out_to_play` | 扭腰舒展、抬头扭腰、舒展观察、左右观察抬头 |
| `expressCalmAlone` | 低头抬头、舒展观察、趴起 |
| `expressJoyAlone` | 扭腰舒展、舒展观察、抬头扭腰 |
| `expressCuriosityAlone` | 左右观察抬头、低头抬头、舒展观察 |
| `expressExcitementAlone` | 抬头扭腰、扭腰舒展、左右观察抬头 |
| `expressAnxietyAlone` / `expressFearAlone` | 短低头停留、短观察停留 |

## 平台执行

Lite3 的低头/抬头使用机身俯仰，观察/扭腰使用原地偏航姿态，舒展使用机身
降低后还原。每段姿态回零并确认站立空闲后再执行下一段；不发送前进、后退或
整圈旋转速度。趴起使用现有 `target_posture`，分别确认连续 5 个新鲜的趴下和
站立状态；方向不明的起趴 toggle 不重试。

Go2 使用原地 Euler 俯仰和偏航姿态；舒展是俯仰身体表达代理。趴起使用
`stand_down` / `stand_up` 和有限时间等待，没有 Lite3 的状态反馈确认。
Euler 中途失败或取消时会归零姿态，再发送冗余 StopMove。

这些身体表达不声称执行独立头部运动、嗅闻、尾巴动作或生物动作。新 Lite3 计划
仍标记 `verified: false`，加入各自物理计划白名单，但 `allow_unverified` 默认仍为
`false`；沿用现有代理/未验收开关。默认随机池未增加需要周围空间确认的整圈旋转。

## 熟悉玩具的到点表达

`inspectFamiliarPlayItem` 在 `target_approach` 成功并停稳后，先执行必需的
`arrival_expression`，随机选择低头抬身、左右观察抬身或降低机身后左右观察之一，
再进入原来的 `inspect` 玩具动作池。组合复用完整 `ACT_ROAM_*` 硬件序列，
本行为每套上限 12 秒，包含姿态回零和站立空闲确认；仍与导航、玩具动作共享
上游 Goal 的总预算，不自动延长总时限。接近或组合失败、取消、超时会中止后续阶段。
Lite3 使用机身俯仰、偏航和高度变化，组合期间只发布零速度；不声称具备真实
嗅闻、咬取或飞盘拾取能力。原六个玩具候选及其语义 ID 保留。

## Tree 总预算与生效

Tree 的 `EVT_VOICE_COMMAND_WALK` / `EVT_VOICE_COMMAND_GO_OUT` 路由和对应
`behaviors.yaml` 预算均为 70 秒。六种情绪仅在 `routes.solo` 设置 70 秒，
`IntentMapper.map_emotion_event()` 将选定路由的预算透传到 Action Goal。
人物与语音等待情绪继续使用原预算。

随机导航仍最多 45 秒，导航确认最多 1 秒，组合最多 20 秒，其余时间留给流程开销。
这些阶段共享一个 Goal 的总截止时间，不会各自重置总预算。

重建并重启 Action、Tree 后生效；软件测试覆盖执行顺序、失败、取消、硬件门控和
预算透传，尚不代表新增组合已完成真机验收。
