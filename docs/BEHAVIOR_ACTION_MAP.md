# 行为与动作映射说明

运行时完整映射只维护在
[behavior_tree_actions.yaml](../config/behavior_tree_actions.yaml)。

## 规模

| 项目 | 数量 |
|---|---:|
| 直接行为 | 53 |
| Stage 候选记录 | 211 |
| 唯一 `ACT_*` | 188 |

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
```

### 直接指令

```text
respond_owner_call
sit_down
lie_down
stand_up
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

## 示例

### eatNormally

```text
prepare
  ACT_LOWER_HEAD_AND_APPROACH_BOWL

eating (3 选 1)
  ACT_LICK_FOOD
  ACT_CHEW_OR_CARRY_FOOD
  ACT_SCRATCH_FOOD

interaction (3 选 1)
  ACT_PAUSE_AND_LOOK_AT_OWNER
  ACT_CHANGE_POSTURE
  ACT_BURP

exit (4 选 1)
  ACT_LICK_LIPS_OR_NOSE
  ACT_SHAKE_HEAD
  ACT_WALK_AWAY_OR_LIE_DOWN
  ACT_SNIFF_GROUND_FOR_CRUMBS
```

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
  是否需要统一改成另一个行为名。

## 动作元数据

`config/action_catalog.yaml` 只包含新表引用的 188 个动作。

`ConfigLoader` 会拒绝缺失动作以及新表未引用的多余动作，不存在旧动作回退。
