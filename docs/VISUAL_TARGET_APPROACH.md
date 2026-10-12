# 视觉目标定位后由 Nav2 接近

具有明确人、动物或物体目标的行为使用以下链路：

```text
Tree /execute_behavior: behavior_name + params_json.target
  -> target_approach: ACT_APPROACH_VISUAL_TARGET -> visual_nav_approach
  -> 绑定唯一稳定的 vision_epoch + target_id
  -> VisionTask.locate_target_once
  -> 原图 ROI + 对齐深度 + TF + 可达地图目标搜索
  -> Action 发送一次 /navigate_to_pose Goal
  -> 等待真实 Nav2 Result，确认底盘收稳
  -> 原 expression / interact / greet / invite / inspect 阶段
```

Tree 保持现有物体/人物分流。Action 复用“过去”的 Nav2 传输与底盘控制权钩子，
接近过程的速度、路径规划和避障由 Nav2 管理。视觉适配器只发布停止指令。
定位或规划成功不是导航到达；接近失败会阻止后续动作。

## 目标绑定与单次定位

Goal 的 `target` 至少携带 `target_type` 和 `target_id`。兼容旧语义目标时，
Action 在当前视觉快照中唯一绑定轨迹，再将完整稳定 ID 交给 Vision。
上游已经指定完整 ID 时禁止通过标签切换到另一个轨迹。
人体读取 `/perception/visual_event`；动物和物体保留 `set_object_detection`
租约及对应 session 的 `/perception/vision/object_detections` v2，用于定位前获取新鲜轨迹。
Action 分别检查这两个 topic 的 `sequence`，同一 topic 的重复或乱序消息会被拒绝；
切换检测 session 不清空序号记录。两路保留原 `vision_epoch` 和 `target_id`，
Vision 重启后两路一起拒绝旧 epoch，并在新 epoch 内重新计数。
Vision 使用物体推理自身的原图尺寸、header 和 source_sequence，不使用更新的人体帧。
仅当前原图中实际检测到的目标允许生成导航点；预测旧框、失鲜、人脸、切分双目画面拒绝定位。

一次成功定位冻结地图目标，随后只执行一个 Nav2 Goal，不循环发点或追随移动目标。
定位成功返回 `navigation_required=false` 时已在安全距离，直接完成接近阶段。

## 距离和预算

| 目标 | 行为 | 地图目标 stand-off |
|---|---|---:|
| 人 | 社交、普通 WithHuman 情绪、主人定制行为 | 至少 1.5 m |
| 动物 | testAnimalBoundary | 1.5 m |
| 动物 | greetAnimal / inviteAnimalToPlay | 1.2 m |
| 物体 | 七个 inspect* 行为 | 0.9 m |

上述距离是未传参时的默认值，不是所有视觉目标统一固定为 1.5 米。
Tree 可以在每个 `/execute_behavior` Goal 的 `params_json` 顶层传入
`stand_off_distance_m`，覆盖该次视觉导航的距离，单位米。此值不写回行为配置，
也不影响下一次未传参的 Goal。例如接近当前视觉选中的玩具：

```json
{
  "target": {
    "target_type": "object",
    "vision_epoch": "实际视觉 epoch",
    "target_id": "实际视觉 epoch:object:实际轨迹 ID"
  },
  "stand_off_distance_m": 0.2
}
```

对应行为可用 `inspectFamiliarPlayItem`。必须使用实际新鲜视觉目标的 ID。
Tree 的 `NEED_EXPLORATION_TRIGGERED` 七个物体分流路由均配置 `0.2 m`：
玩具、垃圾桶、快递箱、纸巾、门、狗粮和陌生物体。
`empty -> exploreRoom` 没有视觉接近阶段。
其他调用未传此字段时仍采用 Action 的 `0.9 m` 默认值。
Action 将距离转为 VisionTask `locate_target_once.params_json.stand_off_distance`，
Vision 再传入 `person_3d_localization/LocateFromBbox.stand_off_distance`；
SLAM 按本次距离生成地图目标，Action 使用该目标导航。
`come_to_owner`（“过来”）沿现有 `person_nav_approach` 路由，未传距离仍为 1.5 米，
也使用相同的上层字段 `stand_off_distance_m`。

显式传入值必须是有限数值，且至少达到
`max(行为 minimum_safe_distance_m, 目标类型最低距离)`。
类型最低距离为人 1.5 米、动物 0.8 米、物体 0.2 米；例如
`testAnimalBoundary` 的行为最低距离为 1.0 米，不能传 0.9 米。
过小距离返回 `unsafe_stand_off_distance`，空值、布尔值、非数值及 NaN/Inf
返回 `invalid_stand_off_distance`，均不会请求定位或启动导航。
本参数只控制到已选目标的停靠半径，不会切换目标。

Vision 强制人体 body 几何、motion_eligible=true、置信度至少 0.5；动物至少 0.4，
物体至少 0.25。目标总年龄最多 0.35 秒。定位源图年龄单独由 Vision 的
`person_localization.max_source_age_sec` 控制；当前部署为 1.0 秒，以覆盖相机传输与
模型推理延迟，未配置时仍为 0.35 秒。原图 Header 不变，SLAM 继续匹配历史深度和 TF。
停止距离是地图目标搜索参数，
实际到达精度由 Nav2 配置与底盘决定。旧 bbox 高度、速度增益和迟滞配置仅保留作兼容测试参数。

`ACT_APPROACH_VISUAL_TARGET` 上限 160 秒，包含定位、导航、收尾，仍受外层 Goal 剩余预算限制。
提高这个动作上限不会自动增加 Tree 的行为预算；当前探索事件没有单独设置
`timeout_sec`，部署时仍使用 Tree 按优先级给出的默认预算。
定位、深度、TF、可达目标或 Nav2 不可用时失败，不回退到视觉速度接近。

`inspectFamiliarPlayItem` 在 `target_approach` 成功并停稳后，先执行必需的
`arrival_expression`，随机选择低头抬身、左右观察抬身或降低机身后左右观察之一，
再进入原来的 `inspect` 玩具动作池。组合复用完整 `ACT_ROAM_*` 硬件序列，
本行为每套上限 12 秒，包含姿态回零和站立空闲确认；仍与导航、玩具动作共享
上游 Goal 的总预算，不自动延长总时限。接近或组合失败、取消、超时会中止后续阶段。
Lite3 使用机身俯仰、偏航和高度变化，组合期间只发布零速度；不声称具备真实
嗅闻、咬取或飞盘拾取能力。原六个玩具候选及其语义 ID 保留。

`exploreRoom` 没有具体目标，仍直接执行探索动作。
`inspectDogFood` 到视觉选中的狗粮目标后执行原六步仪式，不再先导航到固定厨房。
进食行为的固定厨房导航保持原配置。

## 生命周期与部署

取消请求传递给当前 Nav2 Goal，只有真实终态才能释放底盘控制权。
发送接受状态或 Result 不明时保留关联 Goal 和 recovery_required，禁止普通替代行为，
也不执行 Lite3 finish_navigation 模式切换。人工确认停车与下游恢复后重启 Action。
普通成功/失败/已确认取消会收尾并关闭同一个物体检测 session。

Vision 与 Action 必须一起重建并重启；Vision 需要新增 locate_target_once，
旧版本 Vision 会明确失败。环境准备见 [视觉地图导航部署](VISUAL_NAV2_APPROACH.md)。
测试覆盖目标关联、原图 ROI、一次 Nav2 Goal、取消、终态不明、接近失败阻止表达；
深度、障碍物绕行、实机到点及制动仍需现场验收。
