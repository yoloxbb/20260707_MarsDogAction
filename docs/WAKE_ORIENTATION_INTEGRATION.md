# 唤醒声源朝向与 Nav2 Spin 对接说明

> 同步版本：2026-09-21。适用于语音、行为树、动作执行器、底盘和仿真页面联调。

## 1. 最终链路

```text
/perception/audio_event  std_msgs/msg/String(JSON)
  EVT_VOICE_CALL_NAME
  header.frame_id=microphone_array
  wake_angle=<声源角度，度>
              │
              ▼
marsdog_behavior
  respond_owner_call
  params_json:
    use_wake_angle=true
    wake_angle_deg
    wake_confidence
    wake_frame_id
              │
              ▼
/execute_behavior
              │
              ▼
ACT_INTERACT_RESPOND_CALL
  controller route: wake_orientation
              │
              ▼
/spin  nav2_msgs/action/Spin
  target_yaw=<校准后的最短相对转角，弧度>
              │
              ▼
Nav2 / TF / odometry / collision checking
              │
              ▼
底盘原地旋转，车头朝向声源
```

该动作只旋转，不向声源盲目前进。行为名和精确动作名保持：

```text
respond_owner_call -> ACT_INTERACT_RESPOND_CALL
```

## 2. 上下游字段契约

语音事件必须包含：

```json
{
  "header": {
    "frame_id": "microphone_array"
  },
  "event_type": "EVT_VOICE_CALL_NAME",
  "wake_word": "你好小狗",
  "wake_angle": 35.0,
  "wake_confidence": 1205.0
}
```

行为树发送给动作执行器的 `params_json`：

```json
{
  "use_wake_angle": true,
  "wake_angle_deg": 35.0,
  "wake_confidence": 1205.0,
  "wake_frame_id": "microphone_array"
}
```

字段定义：

| 字段 | 类型 | 单位/约束 | 说明 |
|---|---|---|---|
| `use_wake_angle` | boolean | 必须为 `true` | 启用动态朝向 |
| `wake_angle_deg` | number | 度、有限数值 | 声卡原始声源角度 |
| `wake_confidence` | number | 上游原值 | 仅用于观测，不改变角度 |
| `wake_frame_id` | string | 必须为 `microphone_array` | 麦克风阵列原始角度帧 |

缺失角度、非有限角度、缺失坐标系或坐标系不匹配时不驱动车辆，动作返回失败。

## 3. 角度校准

动作执行器按以下公式转换：

```text
relative_ros_deg =
  normalize_to_[-180,180)(
    wake_angle_direction_sign
    * (wake_angle_deg - wake_angle_zero_offset_deg)
  )
```

ROS 约定正 `angular.z`/正 yaw 为逆时针左转。声卡硬件文档尚未在项目中明确
原始角度的零点和正方向，因此首次真机联调必须标定：

1. 声源放在车头正前方，记录 `wake_angle`，填入
   `wake_angle_zero_offset_deg`；
2. 声源放在左侧。当前安装标定为
   `wake_angle_direction_sign:=-1.0`；
3. 如果更换麦克风安装方向后整体左右颠倒，再改为 `1.0`；
4. 分别测试车头、左侧、右侧和后方；
5. 在前方小角度抖动时，用 `wake_angle_deadband_deg` 抑制反复微转。

默认参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `wake_orientation_enabled` | `true` | 已启用 Go2/Lite3 底盘时创建唤醒朝向适配器 |
| `wake_spin_action_name` | `/spin` | Nav2 Spin Action |
| `wake_spin_server_timeout_sec` | `5.0` | 等待 Action Server |
| `wake_spin_result_timeout_sec` | `15.0` | 等待旋转结果 |
| `wake_spin_time_allowance_sec` | `12.0` | 发送给 Nav2 的允许时长 |
| `wake_angle_zero_offset_deg` | `90.0` | 当前双麦模组正前方原始读数 |
| `wake_angle_direction_sign` | `-1.0` | 当前安装的原始角度到 ROS yaw 方向 |
| `wake_angle_deadband_deg` | `5.0` | 前方免转死区 |
| `wake_angle_frame_id` | `microphone_array` | 要求的原始输入帧 |
| `wake_linear_array_back_search_enabled` | `false` | 首个朝向未见人时检查后向镜像角（默认关闭） |
| `wake_visual_confirm_timeout_sec` | `1.5` | 每次 Spin 后等待新鲜人物的时间 |
| `wake_visual_min_confidence` | `0.60` | 用于朝向确认的最低人物置信度 |
| `wake_visual_max_age_ms` | `800` | 允许的人物观测最大年龄 |

当前为线性四麦阵列，声学角存在前后镜像歧义。`wake_linear_array_back_search_enabled`
**默认关闭**：开启后 Action 会先转向首选角并等待一条新鲜的 `active_target`
visual event；若未看到 `tracking` 人物，再计算并转向后向镜像候选角
（多转 180°，不确认机头最终朝向）。该步骤只确认“看到人”，不选人、不伪造
`target_id`，后续 `approach_voice_caller` 仍使用严格的
`vision_epoch + target_id` 锁定。

## 4. 编译和启动

修改后必须重新构建行为树和动作执行器，不能继续使用旧的 `install/`：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws

colcon build \
  --packages-select \
  marsdog_interfaces \
  marsdog_behavior \
  marsdog_action_executor \
  --symlink-install

source install/setup.bash
```

所有节点必须与底盘处于同一 ROS Domain：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

启动：

```bash
ros2 launch marsdog_action_executor action_executor.launch.py \
  chassis_type:=go2 \
  go2_enabled:=true \
  wake_orientation_enabled:=true \
  wake_spin_action_name:=/spin \
  wake_angle_zero_offset_deg:=90.0 \
  wake_angle_direction_sign:=-1.0 \
  wake_angle_deadband_deg:=5.0
```

唤醒朝向不要求 `navigation_enabled:=true`，但板端 Nav2 Behavior Server 必须
已经提供 `/spin`。

## 5. 联调

确认接口：

```bash
ros2 action list -t | grep -E 'execute_behavior|spin'
ros2 action info /spin
```

先脱离语音链路直接测试 Nav2：

```bash
ros2 action send_goal --feedback \
  /spin nav2_msgs/action/Spin \
  "{target_yaw: 1.5708, time_allowance: {sec: 12, nanosec: 0}}"
```

正值应左转约 90°。若这一步不动，应先检查 Nav2、定位/TF、碰撞检测和底盘，
不是检查行为树。

再直接测试动作执行器：

```bash
ros2 action send_goal --feedback \
  /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'wake-test-001', behavior_id: 'wake-test-001', \
    behavior_name: 'respond_owner_call', priority_level: 1, \
    params_json: '{\"use_wake_angle\":true,\"wake_angle_deg\":35.0,\
\"wake_confidence\":1205.0,\"wake_frame_id\":\"microphone_array\"}', \
    timeout_sec: 20.0}"
```

预期日志包含：

```text
Orienting to wake source: raw=35.00deg, calibrated=35.00deg
Nav2 Spin goal: target_yaw=0.6109rad (35.00deg)
Nav2 Spin succeeded
```

最后触发真实唤醒词，检查：

```bash
ros2 topic echo /perception/audio_event
ros2 topic echo /debug/execute_behavior/goal
ros2 topic echo /debug/execute_behavior/result
```

## 6. 仿真页面

页面仍然只调用 `/execute_behavior` 或显示行为树发出的 Goal，不直接调用
`/spin` 和 `/cmd_vel`。

对于 `respond_owner_call`，Goal Debug JSON 的 `params` 会包含：

```json
{
  "use_wake_angle": true,
  "wake_angle_deg": 35.0,
  "wake_confidence": 1205.0,
  "wake_frame_id": "microphone_array"
}
```

页面可显示“检测声源 35°，正在转向”。结构化字段来自 Goal `params`，不要从
日志文本或 Feedback `message` 反解析。最终成功仍以 Result 中
`status=SUCCESS` 且 `result=completed` 为准。

## 7. 取消和安全

- 普通取消、`emergency_stop` 和节点退出都会取消当前 `/spin` Goal；
- `/spin` 使用 TF/里程计闭环反馈，并由 Nav2 做碰撞检查；
- 首次测试仍应架空车轮或清空周围区域；
- 不要在 Spin 执行期间用另一个节点直接发布冲突的 `/cmd_vel`；
- 如果系统存在遥控、导航和动作等多个速度源，应使用 velocity mux；
- 声源角度只有方向没有距离，所以本适配器不执行前进。

### 7.1 底盘接管（2026-09-20 硬件实测补充）

唤醒转向走的是 Nav2 `/spin`，**最终仍由常规 `/cmd_vel` 驱动底盘**，因此
和导航、UWB 跟随共用同一条底盘所有权链。`ros_node.py` 给
`WakeOrientationAdapter` 注入与导航适配器相同的 `prepare_motion` /
`finish_motion` 钩子：

- 需要转向时先 `prepare_navigation()` —— 进入 Vision Mode，否则底盘处于
  摇杆模式会把速度采样**静默丢弃**（实测：`/cmd_vel` 连续 8 秒稳定输出
  `-0.6 rad/s`，而 `/leg_odom2` 的 yaw 一动不动、`/robot_status` 的
  `motion_state` 始终为 0，日志里没有任何报错）；
- 转向结束（含失败、超时）在 `finally` 里 `finish_navigation()` 释放，
  即零速度 + 退出 Vision Mode + 等待底盘 settle；
- 死区内免转的路径**完全不碰底盘**，不开销也不引入冲突；
- 冲突判据是 UWB **是否正在驱动底盘**，不是 UWB 节点是否存在：
  `_ros2_control_conflicts` 接受一个可选的 `uwb_driving` 回调
  （`ros_node.py` 注入 `_follow_is_driving`），只在跟随会话活跃时把 UWB
  链条计入。回调缺失或抛异常一律按冲突处理（fail closed）；
- 跟随真正在跑时，`prepare_navigation()` 会以
  `lite3_control_conflict:active_nodes=...` 立即失败并结束本单元——
  这是正确行为（两条链都往 `/cmd_vel` 写就是硬冲突），比旧版静默等到
  8 秒 goal 超时更快也更明确。
