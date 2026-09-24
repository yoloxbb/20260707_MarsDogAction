# 视觉目标先接近后动作

## 执行链

针对小车，具有明确人、动物或物体目标的行为使用统一两阶段结构：

```text
Tree /execute_behavior Goal
  behavior_name + params_json.target
    -> target_approach Stage
       ACT_APPROACH_VISUAL_TARGET
         -> controller_routes.yaml: visual_target_approach
         -> 人体读取 /perception/visual_event
         -> 动物/物体通过 VisionTask.set_object_detection 独占租约
         -> /perception/vision/object_detections v2 稳定轨迹与公制距离
         -> 受限 /cmd_vel 闭环接近并停车
    -> 原 interact / greet / invite / inspect Stage
```

第一个 Stage 必须成功，第二个 Stage 才会执行。它不调用 Nav2，也不复用
`ACT_INTERACT_APPROACH_VOICE_CALLER` 的语音会话严格契约。

## 行为与距离

| 目标 | 行为 | 停止条件 | 最小安全距离 |
|---|---|---:|---:|
| 人 | `seekHumanInteraction`、`seekInteraction`、`inviteHumanToPlay` | 人体框目标高度 0.80、到达阈值 0.76（归一化） | 模型框代理，不声明米制距离 |
| 人物情绪 | 六个普通 `express*WithHuman` | 人体框目标高度 0.80、到达阈值 0.76（归一化） | 模型框代理，不声明米制距离 |
| 主人不开心陪伴 | `unhappy` | 框高 0.82、阈值 0.79；线速度上限 0.30 m/s（0.45 增益实际更缓）；到达保持 1.5 s | 0.80 m 配置硬下限；框高模式不声明实测米制距离 |
| 想念主人 | `miss_owner` | 框高 0.84、阈值 0.81；线速度上限 0.30 m/s | 0.80 m 配置硬下限；框高模式不声明实测米制距离 |
| 主人离开 | `farewell_leave` | 框高 0.76、阈值 0.72；到达后持续观察 3.0 s | 0.80 m 配置硬下限；框高模式不声明实测米制距离 |
| 动物边界观察 | `testAnimalBoundary` | 1.50 m | 1.00 m |
| 动物互动 | `greetAnimal`、`inviteAnimalToPlay` | 1.20 m | 0.80 m |
| 物体 | 七个 `inspect*` 行为 | 0.90 m | 0.55 m |

米制目标在“期望距离 + 0.12 m deadband”内进入到达保持阶段，因此动物边界
观察实际约在 1.62 m 内停车，问候和邀玩约在 1.32 m 内停车。最低安全距离是
独立硬约束；检测距离达到该值时立即停车，不会为了追求期望距离继续前进。

`exploreRoom` 是没有具体视觉目标的空间探索，因此不加入接近 Stage。
六个熟悉物体行为必须以自己的具体名称发送给 Action；不得覆盖为未注册的
`inspectKnownObject`。

`farewell_leave` 的 3 秒不是盲走计时器：视觉目标处于到达区时底盘保持零速；
若同一主人 Track 在窗口内走远，Controller 会取消到达计时并恢复受限接近；目标
丢失、换 ID 或失鲜则停车并失败。到达保持结束后才执行
`ACT_OWNER_GOING_OUT` 的短暂关注代理。

为避免四足底盘在检测边界附近反复起步，目标失效时仍会立即发布一组冗余停止，
但保持停止期间不再按每帧重复发送。之后必须连续收到 3 个新的有效快照才恢复
运动；同一快照被控制循环重复读取不计数。到达确认另有迟滞区：米制距离需比进入
阈值再远 0.10 m，人体框高需比进入阈值再缩小 0.03，才会退出停止区。

`unhappy`、`miss_owner` 的贴近程度与速度来自每行为策略。定制表达 Stage 本身不
再向前推进，只用低速侧移或原地摆动代理低头贴靠、摇尾和蹭靠，最后归零。
轮式底盘代理不等于独立头部、尾部或四足姿态执行器的真机动作验收。
三个主人行为还要求 Goal 明确提供 `target.identity=owner`；身份缺失或不是主人时
返回 `visual_target_identity_mismatch`，不会退化为任意可见人体。

## 目标绑定

Goal 至少需要：

```json
{
  "target": {
    "target_type": "human | animal | object",
    "target_id": "Tree 选中的目标 ID"
  }
}
```

如果 Tree 同时提供 `vision_epoch`，Action 要求它与当前视觉流完全一致。普通
Need 目标暂时没有统一携带 epoch 时，Action 只在最新有效快照内通过
`target_id`、`track_id`、`identity` 或标签找到候选，并要求最终能固定到唯一的
稳定 `target_id/track_id`。同名多目标无法唯一绑定时直接失败，不会临时选择另
一个目标。

Goal 到达瞬间若选定目标只是 `temporarily_lost`，Controller 保持零 Twist，并在
`acquire_timeout_sec` 内等待同一稳定目标或同一唯一身份恢复为 `tracking`。重捕获
后才开始转向和前进；epoch 改变、目标歧义、取消或等待超时则失败。

普通 Social 的 `check_person` 兼容结果可能只携带身份名和人数：已知身份只绑定
视觉生产者当前选择的同身份 `active_target`；身份为 `unknown` 时也只绑定该
`active_target` 的稳定 Track，不从 `human_candidates` 随机选人。因此多人场景
仍保持 Vision 已有的确定性选择；`active_target` 失鲜、无稳定 ID 或非 tracking
时直接失败并保持零速度。

## 运动安全

- 人体社交策略显式使用锁定人体的模型框高度闭环，不把框高伪装成米制距离；
- 动物和物体只消费当前 Action session 所有的物体检测 v2 流，并必须提供
  有限正数 `distance_m` 且 `range_valid=true`；
- 动物策略使用 0.40 的专用置信度门槛，物体检查使用 0.35，人物仍使用全局
  0.50；Action 启动 Vision 连续物体检测租约时也显式请求 0.35，避免生产者和
  消费者门槛不一致；
- 连续检测从空闲状态启动时允许最多等待 5 秒首帧；等待期间底盘始终保持零速度；
- 目标短暂丢失时立即发布一组冗余停车，保持停止期间合并重复请求；最多等待策略
  指定的丢失时限，并只允许相同 Track 连续稳定 3 个新快照后恢复，不会盲走或
  改追其他目标；
- 动物/物体成功、失败、取消和抢占出口都关闭相同 session 的检测租约；
- 横向偏差较大时只原地对准，不向前；
- 目标缺失、类型不符、歧义、低置信、非 tracking、失鲜或 epoch 改变时停车；
- 取消、抢占、超时、异常、完成和急停均发布冗余零 Twist；
- 运动命令仍受执行 generation、视觉 revision、总超时和视觉新鲜度共同约束；
- 底盘必须保留 `/cmd_vel` 超时看门狗作为硬件侧最后防线。

配置入口是 `config/visual_target_approach.yaml`。物理运行需
所选底盘的 `go2_enabled:=true` 或 `lite3_enabled:=true`；底盘未启用时该 Controller 不安装，行为会明确失败，
不会用 mock 动作冒充接近。

## 距离与人体框排查

动物或物体行为返回 `metric_distance_unavailable` 时，表示目标已经绑定成功，
但 Vision 没有提供有效米制距离。先确认 RealSense 同时开启深度和对齐深度；
不能使用 `enable_depth:=false`：

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true enable_sync:=true

ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once
ros2 topic echo /perception/visual_event --once
```

人体社交行为要求当前轨迹包含有效 `bbox`，并在日志元数据中显示
`distance_source=bbox_height`；即使深度值存在，该策略也不会让深度提前判定到达。
没有人体检测框、框过期或锁定目标丢失时保持零速度。动物和物体行为仍要求
`range_valid: true` 与有限正数 `distance_m`，否则返回
`metric_distance_unavailable`。
