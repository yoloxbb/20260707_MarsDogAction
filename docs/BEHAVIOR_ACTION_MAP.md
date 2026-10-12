# 行为与动作映射说明

运行时完整映射只维护在
[behavior_tree_actions.yaml](../config/behavior_tree_actions.yaml)。

## 规模

| 项目 | 数量 |
|---|---:|
| 直接行为 | 77 |
| Stage 候选记录 | 312 |
| 唯一 `ACT_*` | 191 |

所有名称精确匹配且大小写敏感。没有旧行为名、alias 或旧动作回退。

## 映射规则

- 多 STEP 行为按 `stages` 顺序依次执行；
- 每个 Stage 使用 `random_one`；
- 单动作行为也使用一个只含一个候选的 Stage；
- 表中候选重复项原样保留；
- “随机行为 N 选 1”的 N 与实际动作行数不一致时，以列出的动作行为准；
- `current_action` 和 `executed_units` 返回候选中的原始 `unit_id`。

## 直接行为

### 生理、资源与休息

```text
eatNormally
seekFood
eatExcitedly
seekFoodUrgently
barkShortAlert
lickPaws
sleepOnSide
sleepNow
restInPlace
recharge
```

### 动物互动

```text
testAnimalBoundary
greetAnimal
inviteAnimalToPlay
```

### 人类与资源互动

```text
seekHumanInteraction
seekInteraction
inviteHumanToPlay
```

### 探索和物体

```text
exploreRoom
inspectObject
inspectFamiliarPlayItem
inspectTrashCan
inspectDeliveryBox
inspectTissuePaper
inspectDoor
inspectDogFood
```

除无目标漫游 `exploreRoom` 外，上述动物、人类和物体相关的 13 个行为都先执行：

```text
target_approach
  ACT_APPROACH_VISUAL_TARGET -> visual_nav_approach

原 interaction / greet / invite / inspect Stage
  原 ACT_* 候选随机选择
```

接近 Stage 是必需阶段且失败策略为 `abort`。目标未锁定或距离无效时不会继续
原动作。具体目标字段、停止距离和零速度出口见
[视觉目标接近说明](VISUAL_TARGET_APPROACH.md)。

### 情绪表达

```text
expressCalmWithHuman
expressCalmAlone
expressJoyWithHuman
expressJoyAlone
expressCuriosityWithHuman
expressCuriosityAlone
expressExcitementWithHuman
expressExcitementAlone
expressAnxietyWithHuman
expressAnxietyAlone
expressFearWithHuman
expressFearAlone
expressCalmInPlaceWithHuman
expressJoyInPlaceWithHuman
expressExcitementInPlaceWithHuman
expressAnxietyInPlaceWithHuman
expressFearInPlaceWithHuman
expressCuriosityInPlaceWithHuman
```

其中六个 `*InPlaceWithHuman` 仅用于语音会话 WAITING 阶段，要求
`mobility_policy: in_place`，每个行为只有一个 `expression` Stage，动作候选与
对应普通 `*WithHuman` 的表达 Stage 完全一致，但没有目标接近 Stage。普通 `*WithHuman` 则先通过
`ACT_APPROACH_VISUAL_TARGET` 完成通用视觉人物接近，成功停车后才执行人物表达；
它们不复用语音唤醒专用的 `ACT_INTERACT_APPROACH_VOICE_CALLER`。

### 主人定制视觉行为

```text
unhappy        -> ACT_APPROACH_VISUAL_TARGET -> ACT_OWNER_UNHAPPY
miss_owner     -> ACT_APPROACH_VISUAL_TARGET -> ACT_EXPRESS_MISS_YOU
farewell_leave -> ACT_APPROACH_VISUAL_TARGET -> ACT_OWNER_GOING_OUT
```

三个行为都要求 `target.target_type=human` 且 `target.identity=owner`，先锁定 Tree
选中的同一主人 Track；其他人不会被当作主人回退执行。
`unhappy` 以 0.45 增益较缓靠近；`miss_owner` 使用较快接近和摆动/轻微侧靠
代理；`farewell_leave` 到达后持续观察 3 秒，主人再次走远时恢复跟随同一 Track。
视觉阶段失败会终止行为，定制表达不会执行。

### 直接指令

```text
respond_owner_call
approach_voice_caller
respond_person_fall
respond_stop_gesture
respond_wave_gesture
sit_down
lie_down
stand_up
walk_to_random_point
go_out_to_play
go_home
approach_owner
back_up
stand_still
hold_position
quiet
wait_in_place
come_to_owner
follow_owner
give_paw
high_five
roll_over
spin_around
return_to_owner
drop_object
play_dead
bring_object
fetch_object
emergency_stop
```

其中 `respond_owner_call` 的动作名称仍严格为
`ACT_INTERACT_RESPOND_CALL`，但底盘转角不是固定运动组：它读取本次唤醒事件
的 `wake_angle_deg` 并调用 Nav2 `/spin`。详见
[唤醒声源朝向说明](WAKE_ORIENTATION_INTEGRATION.md)。

`follow_owner` 的精确动作名称仍是 `ACT_INTERACT_FOLLOW_OWNER`，但控制器路由已
从视觉框跟随改为 `uwb_follow`。`timeout_sec=0` 时由同一个长期 Behavior Goal
启动并持有 UWB 跟随，持续运动与取消终态均由此 Goal 负责。

`approach_voice_caller` 的精确映射是：

```text
approach_voice_caller
  target_approach
    ACT_INTERACT_APPROACH_VOICE_CALLER
```

该 Task 不允许随机换人；`vision_epoch + target_id` 必须在整个闭环内保持一致。
正式距离只接受 `range_valid=true` 的米制值，所有目标无效/失鲜/取消/超时出口
都会先发布零 Twist。

新增核心指令保持独立完成语义：`stand_up` 站起后即完成，`stand_still` 还会执行
站立保持；`hold_position` 不改变当前姿态；`quiet` 只停止声音。
`walk_to_random_point/go_out_to_play/go_home` 由 Nav2 先完成目标导航，成功后才
完成对应 Stage；控制器缺失、导航失败、取消或超时均返回失败。

`walk_to_random_point` / `go_out_to_play` 在导航确认后增加 `arrival_expression`，
随机执行一套 2–3 段短组合；六种 `express*Alone` 也增加按情绪区分的组合候选。
完整顺序、平台实际动作和总预算见 [随机到点表现](RANDOM_ARRIVAL_EXPRESSIONS.md)。

Go2 模式通过 `go2_sport.yaml` 在运行时覆盖已实现动作：坐/趴/站、后退、保持、
握手/击掌代理、转圈和装死身体姿态使用官方高层 SportMode；`come_to_owner`、
`approach_owner`、`return_to_owner` 在 Action 侧绑定当前新鲜人体轨迹后执行单次定位与按需 Nav2 导航；视觉身份 `unknown` 可用。
无等价高层接口的动作按配置使用明确标注的代理或失败关闭。
详见 [Go2 ROS2 动作集成](GO2_ROS2_INTEGRATION.md)。

视觉安全事件保持两个不同的公共语义名，且都只执行零速度：

```text
respond_person_fall
  ACT_PERCEPTION_RESPOND_PERSON_FALL -> Go2/Lite3 停车

respond_stop_gesture
  ACT_PERCEPTION_RESPOND_STOP_GESTURE -> Go2/Lite3 停车
```

不得将二者合并成一个行为，也不得映射到任何包含移动段的既有动作。

## 示例

### eatNormally（进食仪式）

到达食盆点位（`navigation_waypoints.yaml` 的 `behavior_routes` 里四个进食行为都走
航点 C 厨房）之后，执行固定的六步仪式，每一步都是单候选 `random_one`，所以操作员
看到的就是这个顺序：**低头 → 抬头 → 趴下 → 站立 → 再低头 → 左右扭腰**。

```text
lower_head        ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD
head_up           ACT_RAISE_HEAD
lie_down          ACT_BASIC_LIE_DOWN
stand_up          ACT_BASIC_STAND
lower_head_again  ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD
waist_twist       ACT_TWIST_WAIST_LEFT_RIGHT
```

`seekFood` / `seekFoodUrgently` / `eatExcitedly` 使用**完全相同**的六个阶段；
`inspectDogFood` 在此之上保留原来的 `target_approach`（`ACT_APPROACH_VISUAL_TARGET`，
order 1），先通过单次地图定位与 Nav2 到达视觉选中的狗粮，
再执行这六步；它不再绑定固定厨房 C。餐盆旁的低头用的是俯仰代理族（`low_semantic_pitch`）的锚点动作，既让仪式
语义贴近「低头闻食盆」，也让该族的 `plan_id` 引用继续有效。

为了这一套仪式，原表里 23 个只为旧进食流程存在的动作（`ACT_LICK_FOOD`、
`ACT_CHEW_OR_CARRY_FOOD`、`ACT_BURP`、`ACT_SHAKE_HEAD`、`ACT_PAW_AT_BOWL_FOR_FOOD`
等）已从 `action_catalog.yaml`、`controller_routes.yaml`、
`navigation_waypoints.yaml`、`go2_sport.yaml` 和 `lite3_actions.yaml` 一并删除；
新增的只有 `ACT_TWIST_WAIST_LEFT_RIGHT`。上游预算注意：整个仪式加导航需要一个
覆盖它们的 goal 预算（Lite3 上仪式本身约 20 s）。

### sit_down

```text
action
  ACT_BASIC_SIT
```

### emergency_stop

```text
action
  ACT_SYSTEM_EMERGENCY_STOP
```

## 数据注意事项

- 原表中的 `expressCuriosiexpressCuriosityWithHumanty` 按明显录入错误整理为
  `expressCuriosityWithHuman`。错误字符串不是可执行行为名。
- 原表连续出现两次 `expressFearWithHuman`。第一组焦虑动作整理为
  `expressAnxietyAlone`，第二组保留为 `expressFearWithHuman`。
- `barkShortAlert` 的候选是排泄相关动作。当前严格按新表保留，等待上游确认
  是否需要统一改成另一个行为名。2026-09-18 按产品要求调整了它的编排：
  `circle`（绕点走 3 圈）→ `action`（蹲下排泄）→ `exit` → `head_up`（抬头，
  完成信号）。动作本身仍来自原表，只新增了 `ACT_RAISE_HEAD` 作为完成标记。

## 熟悉玩具的到点动作

`inspectFamiliarPlayItem` 先接近视觉目标并停稳，随机执行低头抬身、左右观察抬身
或降低机身后左右观察，再选择原玩具互动动作。新增组合最多 12 秒，与接近和
互动共享 Goal 总预算；动作 ID、目标绑定及其他探索行为保持现有契约。
详见 [熟悉玩具的到点表达](RANDOM_ARRIVAL_EXPRESSIONS.md#熟悉玩具的到点表达)。

## 动作元数据

`config/action_catalog.yaml` 只包含新表引用的 191 个动作。

`ConfigLoader` 会拒绝缺失动作以及新表未引用的多余动作，不存在旧动作回退。


### 语音“这是啥？”：`inspect_toy`

`inspect_toy` 新增独立的玩具获取策略和四个必需 Stage：
`ACT_APPROACH_VISUAL_TARGET` → `ACT_ROAM_LOOK_DOWN_UP` → `ACT_ROAM_SCAN_HEAD_UP`
→ `ACT_ROAM_STRETCH_SCAN`。获取阶段启动持续检测，按置信度选择球、飞盘或拉环，
锁定完整 ID 后调用 `locate_target_once` 和一次 Nav2 Goal；只有确认到达才执行三段组合。
详见 [INSPECT_TOY.md](INSPECT_TOY.md)。

`respond_wake_in_place` 由 Tree 在靠近成功或确认不靠近后内部生成。
独立 `expression` Stage 以 `random_one` 混合选择单动作和完整短动作组，
不含人物接近；实际动作池与原地约束见 [唤醒后的随机表现](WAKE_RESPONSE.md)。


### 视觉招手靠近后左右扭腰

`EVT_VISION_WAVE` → `respond_wave_gesture` 的两个必选 Stage：
`target_approach: ACT_APPROACH_VISUAL_TARGET` →
`expression: ACT_TWIST_WAIST_LEFT_RIGHT`。任意被跟踪的人均可招手触发，使用同帧招手者的
`vision_epoch + target_id`，Vision 定位后由 Nav2 靠近至人前至少 1.5m；
第一阶段失败即终止。详细参数见 [行为树动作对接契约](BEHAVIOR_TREE_ACTION_CONTRACT.md#视觉招手直驱respond_wave_gesture)。
