# MarsDog Action Executor 架构

## 1. 职责边界

本包是行为执行器，不负责行为决策。上游行为树选择一个
`behavior_name`；本包根据新动作对照表选择并执行对应 `ACT_*`。

```text
marsdog_behavior
  Behavior Tree / Action Client
           │ ExecuteBehavior
           ▼
marsdog_action_executor
  strict name validation
  stage selection
  action execution
           │ ACT_*
           ▼
controller / simulator
```

## 2. 严格配置边界

### 行为

唯一运行时行为源：

```text
config/behavior_tree_actions.yaml
```

加载后必须恰好得到新表中的 73 个行为。其他 YAML 中存在的历史行为不会进入
运行时注册表。

### 动作

唯一运行时动作目录：

```text
config/action_catalog.yaml
```

该文件只包含 `behavior_tree_actions.yaml` 引用的 182 个唯一动作（2026-09-18 进食仪式改写前是 204）。
`ConfigLoader` 会校验动作目录和引用集合一致；出现表外动作或缺少引用动作都会
阻止节点启动。

### 不支持的兼容机制

- 不解析旧 behavior alias；
- 不加载 `behavior_templates.yaml`；
- 不加载 `behaviors.yaml`；
- 不从情绪池动态生成候选动作；
- 不接受新对照表之外的行为；
- 不暴露新对照表之外的动作。

## 3. 请求处理管道

```text
ExecuteBehavior.Goal
        │
        ▼
GoalParser
  params_json -> ExecutionContext
        │
        ▼
BehaviorResolver
  requested_behavior_name ∈ configured 54?
        │ yes
        ▼
ConfigLoader.get_behavior_template(name)
        │
        ▼
for stage in template.stages
  cancel / timeout check
        │
        ▼
StageExecutor
  candidates
    -> EligibilityChecker
    -> random_one
    -> controller route
       -> Go2/Lite3 backend / UnitExecutor
        │
        ├─ PostureManager update
        ├─ InterruptManager update
        └─ executed_units append
        │
        ▼
ResultEvaluator
  all_required_stages_completed
```

名称校验是精确匹配。无法匹配时：

```text
is_valid = false
error_reason = "unsupported_behavior: '<name>'"
```

## 4. 核心模块

| 模块 | 职责 |
|---|---|
| `goal_parser.py` | ROS Goal / JSON 转 `ExecutionContext` |
| `behavior_resolver.py` | 严格校验 73 个直接行为名 |
| `config_loader.py` | 加载新表、校验严格动作目录、启动校验 |
| `eligibility_checker.py` | 条件、姿态和安全过滤 |
| `stage_executor.py` | Stage 候选选择和执行 |
| `adapters/velocity.py` | Go2/Lite3 共用的速度命令类型与 Twist 桥 |
| `adapters/go2_sport_backend.py` | Go2 `unitree_api/Request` SportMode 动作和速度出口 |
| `adapters/lite3_backend.py` | Lite3 动作计划和速度出口 |
| `adapters/navigation_adapter.py` | 点位导航与平台 Stage 动作执行 |
| `adapters/target_approach_adapter.py` | 强目标锁定、视觉契约校验和语音呼叫者闭环接近 |
| `adapters/visual_target_approach_adapter.py` | 将 Tree 选定的人/动物/物体绑定到实时轨迹，按行为距离闭环接近 |
| `interrupt_manager.py` | immediate / safe_point / non_interruptible |
| `posture_manager.py` | 执行后的姿态状态更新 |
| `result_evaluator.py` | 行为终态评估 |
| `ros_node.py` | `/execute_behavior` ROS2 Action Server |
| `debug_publishers.py` | Goal / Feedback / Result 调试 JSON |

## 5. 主要数据结构

### ExecutionContext

| 字段 | 含义 |
|---|---|
| `requested_behavior_name` | 上游原始名称 |
| `resolved_behavior_name` | 严格模式下与 requested 相同 |
| `params` | 原始 JSON 参数 |
| `priority_level` | 0–6 |
| `target` | 可选目标 |
| `current_stage` | 当前 Stage |
| `current_unit` | 当前 `ACT_*` |
| `completed_stages` | 已完成 Stage |
| `executed_units` | 已成功执行动作 |
| `is_valid` / `error_reason` | 请求校验状态 |

### Behavior Template

```yaml
behavior_name:
  behavior_name: behavior_name
  success_condition: all_required_stages_completed
  stages:
    - stage_id: action
      order: 1
      selection_policy: random_one
      required: true
      candidates:
        - {unit_id: ACT_EXAMPLE}
```

## 6. 选择与执行

新表中的所有 Stage 使用 `random_one`：

1. 读取候选动作；
2. 过滤不满足执行条件的候选；
3. 对剩余候选均匀随机选择一个；
4. 按动作目录元数据创建 UnitExecutor；
5. 成功后将精确 ID 写入 `executed_units`。

动作 ID 不做改名、归一化或相近动作替换。

## 7. 底盘隔离与控制器路由

`chassis_type` 只接受 `go2` 或 `lite3`。两种底盘共享上层
`behavior_name -> Stage -> ACT_*` 契约及导航、视觉闭环、唤醒转向和 UWB
编排；身体动作由各自配置实现：

```text
go2   -> controller_routes.yaml + go2_sport.yaml -> /api/sport/request
lite3 -> controller_routes.yaml + lite3_actions.yaml -> /simple_cmd + /cmd_vel
```

基础路由只登记共享控制器，`_default: unsupported`。Go2、Lite3 的
`ACT_*` 映射在加载时覆盖基础路由；未配置的物理动作失败关闭。
导航 Stage 按 `navigation_waypoints.yaml.stage_actions` 保留语义动作 ID，
到点后调用所选底盘的 `execute_step`，不使用轮式代理动作组。

### Go2 路由

`Go2ChassisBackend` 按 `go2_sport.yaml` 把精确动作转换为官方高层 SportMode
请求。持续 `Move` 默认以 10 Hz 刷新，结束、失败、取消和急停均冗余发送
`StopMove`。过来、靠近和返回主人不是开环 SportMode 序列，而是覆盖到
`visual_target_approach`，由同一 Vision Track 闭环生成 Move/StopMove。

Go2 的 182 个动作有效路由为：171 个 `go2`、4 个
`visual_target_approach`、3 个 `behavior_mobility`，以及 UWB、唤醒转向、语音
目标接近和无身体动作的 `quiet`。SportMode 没有等价能力的翻滚、张嘴、叼取、
面部和生物动作使用 `fidelity: proxy` 的保守身体表达；这只表示动作序列执行
完成，不声称真实物体操作或生物动作完成。Go2 的部署、完整映射和验收边界见
[GO2_ROS2_INTEGRATION.md](GO2_ROS2_INTEGRATION.md)。

### 追随路由

`follow_owner` 的 `ACT_INTERACT_FOLLOW_OWNER` 按底盘走两条实现：Go2 走
外接 FTDI + `local_follow` 进程管线，Lite3 则是
`go2_uwb_behavior` 的 **Action/Service 客户端**（`/go2/follow_uwb` +
`/go2/set_behavior`）——那条链自己拥有 `/cmd_vel`，所以本执行器在 Lite3 上
一个进程都不起。两条实现共享同一套公开方法面，`ros_node` 按 `chassis_type`
选一个注册为 `uwb_follow` 路由，上层调用点不区分底盘。

那条链从启动起就以 20 Hz 持续发 `/cmd_vel`（空闲时为零），会和 Lite3 自己的
twist 动作打架。Lite3 后端因此把它计入 `lite3_control_conflict` 的所有权检查；
**跟随适配器是唯一被允许与它并存的调用方**，通过
`prepare_navigation(allow_uwb_chain=True)` 显式让位，其它动作一律照常失败。
接口细节、四终端部署与排障见
[UWB_FOLLOW_ROS2_INTERFACE.md](UWB_FOLLOW_ROS2_INTERFACE.md)。

`follow_owner` 由 Tree 作为长期 `/execute_behavior` Goal 派发，
`timeout_sec=0` 只取消外层截止时间；内层健康检查与停车确认仍有有限时限。
Go2 或 Lite3 跟随只启动一次，Goal 期间持续反馈。语音 idle 不取消该 Goal。
Tree 用 `/behavior/goal_lease` 按 `goal_id`/`behavior_id` 续租；续租失效则
Action 停车并返回 `FAILED`。抢占先取消旧 Goal，等真实 Result 后才派发新 Goal；
Action 在此期间保持执行锁。Lv0 急停可直接制动，Lv1 安全行为由 Tree 仲裁，
抢占后不自动恢复跟随。

启用语义点位导航后，`navigation_waypoints.yaml` 先按 `behavior_name` 选择
`$HOME/.ros/waypoints.yaml` 中的精确地点名并调用 `/waypoint_nav/task`；只有匹配
task_id 的真实成功终态到达后，
StageExecutor 才从原行为树候选中选择精确
`ACT_*`。到点后按该动作 ID 调用 Go2 SportMode 或 Lite3 动作计划。
导航与 Stage 动作严格串行，取消按 `target_task_id` 作用于
当前点位任务；随机导航向同一任务接口发送保留目标 `K`，并使用同一取消
协议。所有
路径都经过所选底盘停止出口。

Stage 可通过 `motion_state` 声明底盘约束。默认值 `active` 允许按动作映射执行；
`stationary` 是强制不变量：Stage 一进入就发布零 Twist，且该 Stage 的任何动作
都只能保持零速度。`sleepOnSide` 和 `sleepNow` 的 `sleeping` Stage 使用此约束，
下一 Stage 会恢复默认 `active`，使睡觉状态与底盘运动状态保持一致。

`sleepOnSide` / `sleepNow` 的 Stage 顺序固定为
`circle` → `prepare` → `sleep_pose` → `sleeping` → `wakeup`：到点后先原地转一圈，
再铺垫、趴下、保持、起立。`circle` 只有一个候选且排在第一位，不能并进
`prepare`——`prepare` 是 `random_one`，并进去就只有 1/6 的概率会转。呼噜音频挂在
`sleep_pose` 和 `sleeping` 这**两个**「已经趴下」的 Stage 上（见下），所以狗一趴下就
开始打呼、保持姿势时继续打，铺垫和起立都是安静的。2026-09-20 之前只挂了
`sleep_pose`，但该 Stage 在 Lite3 上只有约 3.4 s，被趴下动作本身的舵机噪音盖住，
操作员听不到，因此扩到了 `sleeping`。

`barkShortAlert` 的 Stage 顺序固定为
`circle` → `action` → `exit` → `head_up`：到点后绕点走 3 圈，再蹲下排泄、随机
收尾，最后以新增的 `ACT_RAISE_HEAD` 抬头作为完成信号。`head_up` 同样只有一个
候选且排在最后，并进 `exit` 就只有 1/3 的概率会抬头，不能当完成信号。

`respond_owner_call` 的 `ACT_INTERACT_RESPOND_CALL` 使用独立
`wake_orientation` 路由：只接受 `microphone_array` 原始声源角度，由 Action 经
零点/方向校准转换为 `base_link` 相对 yaw 后调用 Nav2 `/spin`。它不进入固定
Twist 运动组。

`approach_voice_caller` 则严格映射到
`ACT_INTERACT_APPROACH_VOICE_CALLER -> person_nav_approach`。Action 只接受
匹配已识别主人或家人的 `speaker_role + speaker_id`、`wake_id`、
`interaction_id`、严格目标锁和 `vision_epoch + target_id` 的 human Goal。
VisionTask 对该目标执行一次 `locate_person_once`，SLAM 返回有效地图目标后
Action 最多发送一次 Nav2 `/navigate_to_pose`；内层终态未知时锁定恢复状态。

普通 Social / Exploration 目标使用独立的
`ACT_APPROACH_VISUAL_TARGET -> visual_target_approach`，不复用也不放宽语音呼叫者
契约。它先把 Tree 的语义目标唯一绑定到当前视觉 epoch 中的稳定 track，再使用
配置的距离模式接近；成功停车后 StageExecutor 才进入原互动、检查或人物情绪
表达 Stage。13 个 Social / Exploration 行为和六个普通 `express*WithHuman` 的
类型和停止条件白名单由 `visual_target_approach.yaml` 管理，`exploreRoom` 不在
白名单内。

`unhappy`、`miss_owner`、`farewell_leave` 复用同一不可变视觉目标锁，但通过
每行为策略覆盖线速度、加速度、框高目标和到达保持。`farewell_leave` 在保持期
持续消费视觉帧，锁定主人走远时恢复接近；其后定制表达使用所选平台动作，
不能冒充独立头部、尾部或四足控制器验收。

所有前台闭环在行为执行锁内独占 `/cmd_vel`；后台 attention 的“计算 + 发布”也在
同一锁内完成，避免检查后再发布的竞态。目标闭环另以执行 generation 和视觉
revision 串行门保护非零 Twist，保证取消/急停返回后不会再出现迟到 MOVE。

行为音频由 `sound_config.yaml` 的精确 `behavior_sounds` 映射选择。固定点或随机点
导航成功后，音频与首个 Stage 同时开始，且同一时刻只保留一个播放进程；行为
完成、取消、抢占、急停和节点退出都会停止当前音频。相对音频路径以安装后的
`config/` 为基准，因此素材必须随包安装，不能依赖开发机桌面目录。

`sound_config.yaml` 另有一张 `stage_sounds` 表（行为 → Stage → 文件），把音频收窄到
**单个** Stage：进入该 Stage 时起播，`execute_stage` 返回时立刻停止，**不**回退到
通用犬叫，未列出的 Stage 一律安静。一个行为可以挂多个 Stage，每个 Stage 各自起播
（同一个文件在两个 Stage 上会各从头播一次）。它复用同一条播放进程，因此行为级音频
会被它顶掉、且不会在 Stage 结束后恢复；取消、抢占、急停、节点退出这些出口也免费
覆盖它。目前只有睡觉的 `sleep_pose` + `sleeping` 用到它，让呼噜只在狗真正趴下和
保持姿势时响。这张表只在 ROS 节点的 Stage 循环里生效——`standalone_demo.py` 走自己
的循环，离线跑不会出声。

新增音频素材后必须重新 `colcon build`：`config/sounds/` 在安装树里是**真实目录**，
只有重新编译才会把新文件链接进去，否则播放会以文件不存在失败（这时加载器会打一条
`audio file not found` 的 WARNING，不会拒绝启动）。

`emotion_display` 保持订阅原有 Goal、Feedback、Result 调试事件，主视觉为 Qt 实时绘制的
金毛风格卡通电子宠物脸；长下垂耳、圆润头型、金黄渐变毛色和奶油色口鼻由 QPainter
实时绘制。展示层先把兼容分类转换为独立的 `FaceState(emotion, activity,
system_state, intensity)`，再由 `PresentationState` 统一生成用户文案、face cue、动画 cue
和进度显隐，避免把行为映射散落到 `paintEvent()`。

默认 Normal Mode 只显示大尺寸面部、自然语言状态和细光带；完整工程字段由 `D/F1`
控制的右侧半透明 Debug Overlay 呈现。两种模式共享同一实时状态。情绪、Activity、Action
cue 和 intensity 共同控制眼睛开合、瞳孔、耳朵、嘴型、配饰颜色、动画速度/幅度、五官
形变和特效数量；30 FPS 定时器通过插值驱动状态间的柔和过渡。喜悦/社交显示上浮爱心，
焦虑/恐惧显示分级汗滴。三个安全语义保持独立：人员跌倒使用橙红警告和“检测到
有人跌倒”，停止手势使用红色停车提示，真正急停使用深红强警报；它们都抑制
日常装饰并显示脉冲边缘。底盘快速完成零速度 Action 后，跌倒警告仍至少保持
4 秒，但 Debug Overlay 会立即显示真实 Result，新 Goal 可立即覆盖该展示保持。

整体继续采用暖色实体狗狗造型，不使用科技网格、扫描线或雷达光环。该动画层不改变 ROS2
接口，原 PNG 情绪资源继续随包安装，供兼容或其他展示端使用。显示端按 `goal_id` 关联
Goal、Feedback 和 Result，忽略被抢占任务的迟到终态；若启动较晚漏收 Goal，则从首条
Feedback 恢复当前行为。

## 8. ROS2 可观测性

正式接口：

```text
/execute_behavior
marsdog_interfaces/action/ExecuteBehavior
```

调试接口：

```text
/debug/execute_behavior/goal
/debug/execute_behavior/feedback
/debug/execute_behavior/result
```

Feedback 每个 Stage 完成后发布一次。`current_action` 是该 Stage 选择的精确
动作 ID。仿真页面细节见
[SIMULATION_PAGE_INTEGRATION.md](SIMULATION_PAGE_INTEGRATION.md)。

平台细节见 [GO2_ROS2_INTEGRATION.md](GO2_ROS2_INTEGRATION.md) 与
[LITE3_ROS2_INTEGRATION.md](LITE3_ROS2_INTEGRATION.md)。

## 9. 启动校验

`ConfigLoader.load_all()` 会检查：

- 行为存在非空 Stage；
- Stage order 不重复；
- `selection_policy` 合法；
- required Stage 候选不为空；
- 每个候选动作存在于动作目录；
- 动作目录没有新表之外的动作；
- `unit_type`、`interrupt_policy`、timeout、weight 合法。
- Go2、Lite3 动作计划与有效 controller route 一致；
- 导航 Stage 动作由所选平台实际执行，未配置动作失败关闭。

## 10. 扩展流程

增加行为：

1. 在 `behavior_tree_actions.yaml` 增加行为和 Stage；
2. 在 `action_catalog.yaml` 注册新增动作，并保证不保留未引用动作；
3. 更新行为契约测试的期望名称和映射摘要；
4. 同步仿真页面动作资源；
5. 运行测试。

不要通过添加 alias、旧模板或隐式动作替换扩展接口。
