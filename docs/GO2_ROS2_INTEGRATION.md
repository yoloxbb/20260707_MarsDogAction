# Unitree Go2 ROS2 动作集成

## 1. 实现边界

动作执行器只接受 Go2 与 Lite3 两种底盘：

```text
chassis_type:=go2
  controller_routes.yaml + go2_sport.yaml
  -> unitree_api/msg/Request -> /api/sport/request

chassis_type:=lite3
  controller_routes.yaml + lite3_actions.yaml
  -> /simple_cmd + /cmd_vel
```

节点一次只创建所选底盘后端。未配置的 Go2 动作失败关闭。
`unitree_api` 采用运行时可选导入，未 source Unitree ROS2 工作空间时，
`chassis_type:=go2 go2_enabled:=true` 会明确报错。

以下已有能力保持原控制器和配置不变：

- `walk_to_random_point`、`go_out_to_play`：向 waypoint_nav 发送保留随机目标 `K`；
- `go_home`：按 `navigation_waypoints.yaml` 发送 YAML 精确地点名；
- 其他定点行为：同样发送 YAML 精确地点名；
- `follow_owner`：原 `uwb_follow` 进程链，由长期 Behavior Goal 持有 `/cmd_vel` 所有权。

Go2 只替换 Nav2/视觉闭环所使用的底盘速度出口：原先的 Twist 被转换为高层
SportMode `Move`/`StopMove` 请求。导航地点名、随机目标 `K`、Nav2 Goal、UWB 算法和
启停契约没有改写。

## 2. 官方接口

实现依据 Unitree 官方 ROS2 示例：高层运动请求使用
`unitree_api/msg/Request` 并发布到 `/api/sport/request`。本项目使用的 API ID
包括：

| SportMode | API ID | 本项目用途 |
|---|---:|---|
| `StopMove` | 1003 | 停止、保持、取消和急停出口 |
| `StandUp` | 1004 | 站起 |
| `StandDown` | 1005 | 趴下、装死身体姿态、部分低落表达 |
| `RecoveryStand` | 1006 | 睡眠行为的起身恢复代理 |
| `Euler` | 1007 | 小幅身体俯仰表达 |
| `Move` | 1008 | 后退、转圈、Nav2/视觉速度出口 |
| `Sit` | 1009 | 坐下 |
| `Hello` | 1016 | 挥手；握手/击掌、比心、舞蹈、拉伸及前爪/扑跳/抓取类动作的统一安全代理 |

> ⚠️ `Scrape`(1029，拜年作揖) 和 `FrontPounce`(1032，向前扑) 会让 Go2 后腿站立或向前扑，
> 真机有翻倒、扑人风险，已从所有动作映射中移除，请勿重新引入。

参考：

- [Unitree ROS2 中文说明](https://github.com/unitreerobotics/unitree_ros2/blob/master/README%20_zh.md)
- [官方 SportClient 请求定义](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/include/common/ros2_sport_client.h)
- [官方 Go2 Sport 示例](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/go2/go2_sport_client.cpp)

## 3. 当前指令映射

| 语义 | Behavior | Go2 执行 | 状态/限制 |
|---|---|---|---|
| 走/去 | `walk_to_random_point` | waypoint_nav `place="K"` | 实时地图随机目标 |
| 出去玩 | `go_out_to_play` | waypoint_nav `place="K"` | 实时地图随机目标 |
| 回家 | `go_home` | waypoint_nav YAML 精确地点名 | 固定地点导航 |
| 跟着我 | `follow_owner` | 外接 FTDI UWB + `go2_uwb_local_follow` | 外部进程管线与 `/cmd_vel` |
| 过来/回来 | `come_to_owner` / `return_to_owner` | Action 绑定当前新鲜人体轨迹，单次定位并按需调用 Nav2 | `unknown` 身份允许；目标失鲜、定位或导航失败即停止并返回失败 |
| 靠近点 | `approach_owner` | 同一单次定位与 Nav2 链路，1.5 m 停靠距离 | `unknown` 身份允许；没有新鲜人体轨迹则失败停车 |
| 退后 | `back_up` | `Move(x=-0.20)` 1.2 秒 | 时间式“两步”代理，真机需标定 |
| 坐下 | `sit_down` | `Sit` | 已实现 |
| 趴下 | `lie_down` | `StandDown` | 已实现 |
| 起来 | `stand_up` | `StandUp` | 已实现 |
| 站好 | `stand_still` | `StandUp` 后持续 `StopMove` | 已实现 |
| 别动/等着 | `hold_position` / `wait_in_place` | `StopMove` | 保持当前姿态 |
| 握手/击掌 | `give_paw` / `high_five` | `Hello` | 官方高层前肢动作代理，不能指定任意腿 |
| 转圈 | `spin_around` | `Move(z=1.2)` 约一圈后停止 | 真机需按地面摩擦标定 |
| 装死 | `play_dead` | `StandDown` | 仅身体姿态；无闭眼接口 |
| 安静 | `quiet` | 停止 Action 行为音频，不发身体动作 | 已实现 |
| 翻滚 | `roll_over` | `StandDown` 身体代理 | 不声称完成侧向翻滚 |
| 放下/张嘴 | `drop_object` | 低头 `Euler` 身体代理 | 无口部执行器，不声称物体已释放 |

73 个行为引用的 182 个动作均有明确 Go2 有效路由：171 个展开后的 SportMode
动作，4 个视觉目标闭环，3 个 Nav2 行为移动，以及 UWB 跟随、唤醒转向、语音
目标接近和无身体动作的 `quiet`。没有等价高层能力的口部、尾部、面部、物体
夹持和生物动作统一标记 `fidelity: proxy`；代理成功只代表身体序列完成。

## 4. 夸赞、责备和主人行为

Tree 保持原事件语义：

```text
PRAISE -> Joy / Excitement behavior
SCOLD  -> Anxiety / Curiosity / Fear behavior
```

Action 在候选选择前过滤当前 Go2 后端不可执行的动作；当前所有表达候选均有
Go2 映射，因此保留原来的均匀随机选择。有人场景仍先执行
通用视觉接近；活跃语音会话的 `*InPlaceWithHuman` 保持原地，无人场景继续按
既定概率选择原地表达或随机点位活动。

三个定制行为保持两阶段：

```text
unhappy        -> 视觉锁定并接近 owner -> StandDown
miss_owner     -> 视觉锁定并接近 owner -> Hello
farewell_leave -> 视觉锁定并接近 owner -> Hello
```

Tree 必须在 Goal 中提供稳定的 `vision_epoch + target_id`，且
`target_type=human`、`identity=owner`。Action 不猜主人、不在多人之间换 Track；
目标缺失、身份错误、低置信度、旧帧、丢失、取消和超时都会先发送停止请求，并
中止后续表达。

## 5. 配置

Go2 专属映射位于 `config/go2_sport.yaml`：

```yaml
enabled: false
request_topic: /api/sport/request
publish_rate_hz: 10.0
limits:
  max_linear_x: 0.30
  max_linear_y: 0.20
  max_angular_z: 1.20
  min_linear_x: 0.25
```

`min_linear_x` 是 Go2 SportMode `Move` 的起转阈值：任何非零线速度都会被上抬到该值，
否则固件会静默丢弃低于阈值的指令、机器人原地不动。`action_sequences` 保存需要逐
动作精确配置的序列；`action_groups` 保存可复用动作
谱，并为每组声明 `fidelity: exact|proxy`、说明、SportMode 序列和完整动作 ID
列表。`ConfigLoader` 会把两者展开成 171 个精确 `ACT_*` 映射，并拒绝重复动作、
未知动作、超时超限或未接入控制器的 Go2 配置。

后退距离和转圈角度目前由速度乘持续时间近似。首次真机联调只应在清空场地、低速
和有人急停监护下调整 `ACT_BASIC_BACK_UP`、`ACT_TRICK_SPIN` 的速度/时长；不要
通过放宽视觉失鲜、安全距离或速度上限来补偿标定误差。

## 6. 启动

先按官方说明配置 Unitree 网卡并构建/安装 `unitree_ros2`。下面的路径应替换为
设备上的实际工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/setup.sh
cd ~/ros2_ws
colcon build \
  --packages-select marsdog_interfaces marsdog_action_executor \
  --symlink-install
source install/setup.bash
```

确认 Unitree 消息和 SportMode 服务可见：

```bash
ros2 interface show unitree_api/msg/Request
ros2 topic info /api/sport/request -v
```

启动 Go2 Action：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=go2 \
  go2_enabled:=true \
  go2_request_topic:=/api/sport/request \
  navigation_enabled:=true
```

Tree 必须与 Action 一同更新并启动；主人靠近命令始终先查询已确认的视觉目标：

```bash
ros2 launch marsdog_behavior behavior_tree.launch.py
```

若只联调姿态动作，可先设 `navigation_enabled:=false`。当前 Go2 UWB 跟随
经外接 FTDI UWB 与 `go2_uwb_local_follow` 本地规划；Action 管理进程启停，
规划链路拥有 `/cmd_vel`，需检查串口、订阅者和底盘控制所有权。

## 7. 真机验收顺序

1. 架空或清空场地，确认 App/遥控器急停可用；只发送 `stand_up`、`sit_down`、
   `lie_down`。
2. 检查 `/api/sport/request` 有匹配订阅者；无订阅者时 Action Goal 必须失败，
   不能报告模拟成功。
3. 低速验收 `back_up`、`spin_around`，记录实测位移/角度后再标定时长。
4. 用 owner、非 owner、多人和遮挡场景验收三个主人导航动作，确认错人和失鲜时
   不下发 Nav2 Goal；导航中取消要等真实 Result 和底盘停稳。
5. 单独验收 waypoint_nav 精确地点名与 `K` 随机目标，确认当前
   `waypoints.yaml`、实时 `/map`、定位和局部规划适配四足底盘。
6. 最后验收 UWB 会话启停与抢占；确认外部进程终止、底盘停车，
   且没有第二个 `/cmd_vel` 所有者。

当前自动化测试证明的是配置、路由、消息构造、候选过滤、停止出口和视觉目标
约束；不等于 Go2 固件、DDS 网络、步态稳定性、Nav2 或 UWB 真机验收。
