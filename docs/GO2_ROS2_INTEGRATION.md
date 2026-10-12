# Unitree Go2 ROS2 动作集成

## 协议与完成依据

Go2 使用 `unitree_api/msg/Request`，发布到 `/api/sport/request`。
Lite3 继续使用自己的 `/simple_cmd`、`/cmd_vel` 和动作表；两者只共享 Tree 行为名与通用动作目录。

实现对照的官方来源：

- [Unitree Go2 API ID 定义](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/sport/sport_api.py)
- [Unitree Go2 SportClient 参数编码](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/sport/sport_client.py)
- [Unitree ROS2 SportClient](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/common/ros2_sport_client.cpp)
- [Nav2 Humble Spin 实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/plugins/spin.cpp)

| 接口 | API ID | 用途及参数 |
|---|---:|---|
| BalanceStand | 1002 | 显式切换平衡站立模式，供身体姿态与运动准备 |
| StopMove | 1003 | 停止、保持与取消出口 |
| StandUp | 1004 | 普通起身；不使用跌倒恢复代替普通起身 |
| StandDown | 1005 | 趴下；不等同于侧卧、仰卧或翻滚 |
| RecoveryStand | 1006 | 跌倒恢复接口，当前普通动作表不调用 |
| Euler | 1007 | `{"x":roll,"y":pitch,"z":yaw}`，单位 rad；机身姿态，不是独立头部动作 |
| Move | 1008 | `{"x":vx,"y":vy,"z":vyaw}`，单位 m/s、rad/s；速度控制，不提供圈数目标 |
| Sit / RiseSit | 1009 / 1010 | 坐下 / 从坐姿起身；当前动作表使用 Sit、StandUp |
| SpeedLevel | 1015 | `{"data":level}`，编码已修正；当前动作表不修改速度档位 |
| Hello | 1016 | 招手抬前爪，也作为获准的抓挠、跳扑表达替代；不接触目标或跳跃 |
| Stretch | 1017 | 原生伸懒腰，替换旧的 Hello 拉伸代理 |
| Dance1 / Dance2 | 1022 / 1023 | 原生舞蹈；当前 `ACT_BONE_DANCE` 使用 Dance1 |

SportMode 发布成功只表示请求已经发布且存在匹配订阅者。当前没有接入
`/api/sport/response` 的固件 ACK 或动作完成反馈，因此固定等待结束不代表真机动作已被确认完成。
运行结果明确记录 `completion=command_sequence_wait_only`、
`physical_completion_confirmed=false` 和实际配置的命令姿态。
固件是否支持 Stretch、Dance1 等接口仍需当前设备验收，不能从 SDK 存在该 API 推断固件一定支持。

## 完整动作核对结果

75 个行为引用的全部 191 个动作都已核对，逐项记录在
[GO2_ACTION_AUDIT.csv](GO2_ACTION_AUDIT.csv)。可直接用表格软件筛选动作、行为、控制路由、参数和限制。

| 分类 | 数量 | 含义 |
|---|---:|---|
| `exact` | 21 | 语义直接对应原生动作，例如坐下、趴下、起身、停止 |
| `composed` | 8 | 原生动作或导航接口组合，例如起身后伸展、睡前三圈 |
| `proxy` | 150 | 用户授权的安全替代动作；记录实际身体表达，不声称完成原口部、搬运、接触或生物任务 |
| 独立路由或音频策略 | 12 | 导航、视觉、UWB、唤醒转向、quiet；是否可用取决于实际服务和开关 |

`config/go2_sport.yaml` 的每个动作计划均保留 `fidelity`、`description`、
`commanded_posture`、`sequence`，必要时有 Go2 专用 `timeout_sec`。
配置校验拒绝缺失分类、重复映射、未知 API、非法参数和不足以执行完整序列的时限。
Go2 的专用时限不会修改 Lite3 的目录或动作配置；显式 Stage 时限和整个 Behavior Goal 时限继续有效。

原先明确不支持的 85 个身体动作，按用户最新授权配置为安全替代：

- 口部、叼取、放下、搬运：小幅机身低头再回正；不实际搬运物体。
- 指定爪抓挠、跳扑：原生 Hello；不跳跃或接触目标。
- 翻滚、露肚：原生 Stretch；不翻身、翻腾。
- 舔舐、尾巴、面部：小幅机身横滚再回正。
- 未接入的目标、路径、接触任务：小幅左右观察；不宣称完成原目标任务。
- 返充：StopMove 后恢复站立；不对接、不确认充电。排泄：Sit → StandUp 替代。

这些动作仍使用原 ACT ID 和 Tree 行为名，`fidelity=proxy`、描述及完成依据随 Action Result 返回。
代理成功仅代表替代指令序列完成。Go2 返充结果的 `charging_completed=false`，
兼容字段 `energyValue` 仍来自既有配置默认值，并标注 `energyValue_source=configured_default`，不是 BMS 实测。配置与执行器仍拒绝未知动作，不会任意选动作回退。
随机候选可选这些代理；固定和顺序候选仍必须实际执行，不能靠过滤跳过。
不启用后空翻、前空翻、FrontPounce 或 Scrape。

`ACT_UWB_RANDOM_ROAM` 保留 `uwb_roam` 路由，但当前 Go2 运行时没有注册该控制器，因此 `play_alone`
也不会获得模拟成功。UWB 跟随开关与此不同，沿用原设置。

重生成核对表：

```bash
cd /home/cat/xbb/20260707_MarsDogAction
python3 scripts/export_go2_action_audit.py
```

## 睡眠、起身与姿态

睡眠阶段依次执行：导航到睡眠位置 → `/spin` 三圈 → 睡前准备 → 趴下 → 静止休息 → 起身。

`ACT_CIRCLE_AROUND` 使用单个 Nav2 Spin Goal，`target_yaw=6π`，接口时间额度默认 60 秒，
Go2 动作时限 65 秒。保留完整角度，不归一化为零，也不把速度乘时间当作完成圈数。
Nav2 Spin 自己累积跨越 ±π 的航向变化并判断完成；Action 只使用关联 Goal 的真实终态 Result。
接口未启动、拒绝、失败、取消或终态未知都不能进入后续趴下阶段，且没有定时 Move 回退路径。
`spin_around` 与其他原地转圈表达使用同一接口转一圈，时间额度 20 秒，动作时限 25 秒。
本机实际导航配置 `/home/cat/robot_ws/slam_ws/src/robot_slam_bringup/config/nav.yaml` 的
`behavior_server.ros__parameters.max_rotational_vel` 为 **0.8 rad/s**；接近目标时仍由 Nav2 减速。
此参数是 Spin 的速度上限，不是 Go2 Move 出口的 1.2 rad/s 限幅。
Humble Spin 在 configure 时读取并缓存参数，修改后须重新配置导航生命周期，
不能只修改参数显示值。参见 [Nav2 Humble Spin 实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/plugins/spin.cpp)。

取消 ACK 不表示动作已结束：要等待真实终态。接收或终态超时会保留 Goal 所有权并锁定恢复，
迟到的接收结果也会取消并等待其真实 Result；锁定期间不会重新起身或启动另一段身体运动。
真实 Result 的成功、取消、失败不会被本地姿态、距离、速度或停车发布结果改判。

睡眠各姿态统一为标注的 StandDown 趴姿代理，不声称侧卧、仰卧或蜷缩。
表达类趴下和睡姿单元均在 2.5 秒下趴过渡后额外保持 6 秒；单元内不会自动起身。
静止睡眠阶段即使选中翻身/踢腿代理，也只执行 StopMove 保持当前姿态；发声由行为音频负责。
普通爬起/滚起/弹起变体使用标注的 StandUp 代理；伸展起身为 StandUp → Stretch，
坐起为 StandUp → Sit，姿态记录为 sitting，而非通用目录里的 standing。

下一次需要运动时，显式发送 StopMove → StandUp，等待 `stand_up_wait_sec`（默认 2.5 秒），
再选择 BalanceStand。已记录起身准备完成时复用该记录，只补缺少的 BalanceStand。
连续 Move 和普通 StopMove 不清除平衡标记；趴下、坐下、阻尼、取消或准备失败使两个标记失效。
单独 StandUp 的等待完成只记录起身，平衡标记只能在另行发送 BalanceStand 后建立。
速度出口不阻塞视觉回调，等待期间丢弃速度，只接受准备完成后新到的指令。
显式“站好”也会先起身，不再只发送 StopMove。

Euler 扭腰使用机身 yaw，避免旧的 Move 左右旋转改变整机航向。
Euler 序列失败或取消时尽力回正，动作结束冗余发送 StopMove。
站立类动作在停止后补 BalanceStand；成功的完整行为链也执行统一平衡收尾，
必要时先 StandUp 并等待。睡眠中间阶段保持趴姿，整条链完成后才恢复站立。
单独坐下、趴下、装死、保持、等待、quiet、停止手势及急停保留原姿态或停止语义。
取消或失败的行为链不自动起身；收尾结果记录为 go2_readiness，固定等待不确认真实姿态。
序列预算不足会失败，不会把截断的 Stretch 或组合动作报告为完成。
Go2 固定指令等待不再乘情绪 speed_scale/duration_scale，避免连贯动作被缩短后报预算不足；
显式单元时限和整个行为 Goal 截止时间仍然有效。
这些都是命令准备与账面姿态，未加入物理姿态成功判定。

Go2 的三种平静行为使用 `go2_sport.yaml.behavior_stage_overrides` 的随机表达组合：
每次从 8 个候选中无放回抽取 2～3 个，抽取顺序就是执行顺序。同一组合不重复动作；
跨次执行仍可能巧合抽中相同组合。独处候选为低头抬身、短低头停留、左右观察停留、
扭腰、轻摆身体、Stretch、坐下、趴姿休息；有人时将低头抬身换成轻抬身体，趴卧换成
`ACT_SLEEP_BY_FEET` 的原地趴卧代理，不声称接触主人。伸展、坐下和趴卧均为可选。
例如：左右观察 → 轻摆身体；坐下 → 起身扭腰 → 趴卧休息；低头抬身 → Stretch。
WithHuman 保留原接近目标阶段，InPlaceWithHuman 不增加接近目标阶段。
表达只使用原地 Euler、Stretch、Sit、StandDown/StandUp 和 StopMove；无 Move、
Spin、舞蹈、跳跃、后空翻或侧翻。Euler 沿用小幅姿态：roll ≤ 0.10、pitch ≤ 0.15、
yaw ≤ 0.20 rad。Lite3 沿用原模板。
趴卧仍完整保持 6 秒，后续站姿动作先执行起身等待；表达链全部成功后统一起身和平衡收尾。
取消和失败不继续剩余动作，也不触发自动起身。每个单元与整条链共享原 Goal 截止时间，
组合不会延长上游总预算。日志打印抽中的动作顺序；软件验证不代表负载安装后的真机验收。

## 导航与速度边界

原有 waypoint_nav、单次视觉定位再 Nav2、唤醒方向和 UWB 跟随链路保留。
导航成功、失败、取消只来自导航接口的终态，不增加机身高度、速度或距离检测。
禁止用这些检测改判接口成功或替换接口失败；停车和平衡结束钩子仅尽力执行，
收尾失败不覆盖真实导航/Spin Result，但会使准备标记失效。

Move 的上限仍为 x=0.30 m/s、y=0.20 m/s、yaw=1.20 rad/s。
`min_linear_x=0.25` 是本设备既有部署配置，不是 Unitree 协议统一阈值；非零 x 会按此配置抬高，
零速度保持零，且配置校验保证最小值不超过上限。后退仍为时间式位移，不保证精确步数或距离。
Go2 Action 默认通过内置 `Go2VelocityBridge` 订阅 `/cmd_vel`，和所有动作共用一个后端及准备标记。
Action 导航在提交 Goal 前执行 `prepare_navigation()`；RViz、直接航点服务或直接 `/cmd_vel`
绕过 Action Goal 时，在第一个非零速度启动一次非阻塞起身准备。等待期间的速度被丢弃，
准备结束后只使用新收到的速度，不逐帧起身。订阅队列深度为 1，0.5 秒没有新速度时仅发送停止。
Action 从接受执行到整条行为结束期间，外部非零速度只在该行为的导航或 Spin 窗口通过；
普通身体序列、单元间隙和静止休息期间均丢弃，避免另一个速度源提前触发 StandUp。
停止速度仍可通过；没有 Action 行为执行时，RViz/航点服务/直接速度仍使用同一个准备入口。

内置桥接开启时必须停掉独立 `go2_twist_bridge/twist_bridge`，否则其无准备的请求会绕过门禁。
配置见 `cmd_vel_bridge.enabled/topic/timeout_sec`，节点参数为 `go2_cmd_vel_bridge_enabled` 和
`go2_cmd_vel_topic`。取消或发送失败后，持续速度不能解除停止门禁；下一次 Action Goal 的显式准备才能重新启用。

## 构建与启动

本机路径：

```bash
source /home/cat/unitree_ros2/setup.sh go2
source /home/cat/ros2_ws/install/setup.bash
cd /home/cat/ros2_ws
colcon build --packages-select marsdog_action_executor --symlink-install
source install/setup.bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=go2 go2_enabled:=true lite3_enabled:=false \
  navigation_enabled:=true uwb_follow_enabled:=false
```

这里的 `uwb_follow_enabled:=false` 延续当前现场暂时关闭跟随的启动选择；恢复跟随时显式改为 true。
原有启动参数保留。睡前三圈依赖 Nav2 behavior server 的 `/spin` 和 Action 内置 `/cmd_vel` 桥接；独立桥接需停止。
只修改源码不会更新已启动的 Python 进程，构建后需重启 Action 节点才加载新代码与动作表。

验证先区分软件与真机证据：自动化测试覆盖路由分类、原生 API 编码、三圈目标、真实终态、取消所有权、
起身准备、停止失败和候选过滤；构建和测试不能证明当前固件支持或真机动作已经完成。
验收需在电机冷却且现场可观察时重复“导航 → 趴下 → 起身 → 再导航”，以及完整行为链后导航。
导航判定仍只看接口 Result；实际移动与里程计变化单独作为真机验收证据。当前用户要求因电机过热暂缓真机验收。

2026-10-09 本轮软件验证：完整测试 659 passed、4 skipped；独立 ROS Domain 173 的 6 项流程检查通过，
全部底盘发布已拦截，Spin 使用受控返回值，不属于真机验收。共享工作区构建成功，安装代码与源码一致。
Action 已保留原命令和环境重启，98 项在线参数逐项一致；Spin 上限为 0.8 rad/s，
behavior_server、bt_navigator、controller_server 均为 active。按用户选择，行为树保持停止，旧速度桥未运行。
原文件、修改后文件、测试日志和运行快照保存在
`/home/cat/go2_communication_backup/20261009_220020_go2_sleep_calm/`。
