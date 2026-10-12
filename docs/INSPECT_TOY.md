# “这是啥？”语音直驱玩具探索

语音标准词“这是啥？”和“这是什么？”通过确定性词库发布一个
`EVT_VOICE_COMMAND_INSPECT_TOY`，`command_id=CMD_INSPECT_TOY`。
Tree 按 `command_inspect_toy → inspect_toy` 路由为 Lv1 外部直驱行为，
总超时 210 秒，冷却 0.2 秒。它不依赖情绪阈值，也不要求 Tree 预先识别玩具。
`ACT_INSPECT_TOY` 仅为 Voice 描述元数据；Tree 向 `/execute_behavior` 发送的
行为名是 `inspect_toy`。

Action 通过 `/perception/vision/task` 的 `set_object_detection` 租用持续检测，
订阅 `/perception/vision/object_detections` 的 schema v2 结果。
允许的目标为 `dog toy ball`、`dog frisbee toy`、`dog tug ring toy`。
在 5 秒获取窗口内，选择同一有效结果中置信度最高的新鲜玩具；置信度相同按
稳定 ID 排序。锁定完整 `vision_epoch:object:track_id` 后不再换目标。
靠近期间使用同一检测 session，每 10 秒续租 30 秒；靠近结束或取消时释放。

通过 `locate_target_once → /person_3d_localization/locate_from_bbox → Nav2`
导航到玩具旁，默认期望距离为 0.6 米、最小允许距离为 0.4 米（相对 base_footprint 到目标点）。
优先检查期望距离上的所有方向，再搜索更远候选；障碍限制下实际目标可以更远。
视觉接近使用 navigate_to_visual_target.xml 和 visual_goal_checker，位置到达容差为
0.05 米；普通导航仍使用原来的 general_goal_checker。确认真实导航 Result 和
底盘收尾后，依次执行三个完整动作组合：

1. 低头观察 → 抬头 → 恢复中立姿态。
2. 左看 → 右看 → 抬头 → 恢复中立姿态。
3. 降低身体并恢复 → 左右观察 → 恢复中立姿态。

动作复用 Lite3 已配置的身体 pitch、yaw 和 body-height 命令；每段允许 12 秒。
它们是身体姿态表达，不具备实际叼取或抓取玩具的能力。检测无目标、定位失败、
导航失败、超时或取消时，不执行后续组合；续租失败会取消当前接近任务。
取消导航仍等待下游真实 Result，未知终态遵循 Action 已有恢复锁。

本机使用的视觉项目是 `/home/cat/marsdog_ws/src/MarsDogVisionInteraction`；
其现有持续检测和 `locate_target_once` 接口支持此流程，无需改为常开检测。
语音和 Action 源码分别在 `/home/cat/xbb/MarsDogVoiceInteraction`、
`/home/cat/xbb/20260707_MarsDogAction`。本机 `/home/cat/ros2_ws/src` 的旧视觉链接
指向另一个 checkout，部署时应确认 Vision 的安装来源与上述项目一致。

重新构建并重启 Voice、Tree 和 Action 后生效。Lite3 的姿态组合沿用现有硬件
开关：Action 启动参数需要 `chassis_type:=lite3 lite3_allow_unverified:=true`。
默认配置 `allow_unverified: false` 会拒绝这些姿态，三个 `ACT_ROAM_*` 已在现有
`accepted_unverified_actions` 白名单中；此改动保留该开关与白名单检查。

软件测试覆盖词库发布、Tree 路由、玩具筛选与锁定、租约释放、导航失败和取消，
以及 Lite3 三段组合的实际命令序列。相机推理、深度定位、实机靠近和动作效果需
部署重启后验收，测试中的模拟 Nav2 Result 不代表硬件已到达。
