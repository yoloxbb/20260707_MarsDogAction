# 视觉目标点“过来”：定位与运动执行分离

语音 → Tree 仲裁并发送 `/execute_behavior` Goal → Action 绑定一个新鲜人体 ID
→ VisionTask `locate_person_once` → SLAM `LocateFromBbox` 生成固定地图目标
→ Action 发送一次 Nav2 `/navigate_to_pose` → 等待真实 Result → 底盘收尾。

`come_to_owner`、`approach_owner`、`return_to_owner` 和已授权的
`approach_voice_caller` 复用已有 `person_nav_approach` 路由。
Tree 继续负责行为选择、抢占和取消；Nav2 客户端统一留在 Action。
点位生成成功或 `ComputePathToPose` 规划成功都不代表已到达。

Vision 的 `locate_person_once` 已补齐：按完整 `vision_epoch:human:track_id`
冻结原始推理帧中的人体候选，校验 body 几何、`motion_eligible=true`、置信度至少 0.5、
目标年龄不超过 0.35 秒、原图尺寸和时间戳；将归一化人体框还原为原图像素 ROI，
调用 `/person_3d_localization/locate_from_bbox`。
该服务只返回目标数据，不能驱动机器人；失败或晚到响应不会自动启动导航。
本接口使用单路对齐的彩色/深度相机，拒绝切分双目图像和仅人脸目标。

Vision 通过 `config/vision.yaml` 的 `person_localization` 配置服务名和有限等待时间
（默认 25 秒，匹配下游最多 20 秒候选路径搜索）。
Action 检查任务 ID、任务类型、目标 ID、业务码、`map` 人体点和有效位姿。
`navigation_required=false` 表示已处于安全距离，完成定位后直接结束；
`true` 才发送固定 Nav2 Goal，保持不少于 1.5 米安全距离。

## 控制权与取消

保留 Lite3 `prepare_navigation()` / `finish_navigation()` 钩子。
导航前确认状态、站立并切程序控制；只有真实 Nav2 Result 到达后才执行收尾。
取消确认仅代表收到请求；Goal 接受超时、Result 未知或传输异常时，
保留关联 Goal 和恢复锁，不提前发送 `0x21000C02`，拒绝普通替代行为。
人工确认停车并恢复下游后重启 Action。

这修复了未知终态仍无条件执行 finish 的问题。尚未实机重现“能规划但不走”，
不能将该现象确定归因于模式切换；仍需查看 preflight、Nav2 Result 和 `/cmd_vel` 链路。

## 部署

构建和运行终端都需要 source 含 `LocateFromBbox` 的 SLAM 安装空间：

```bash
source /opt/ros/humble/setup.bash
source /home/cat/robot_ws/slam_ws/install/setup.bash
source /home/cat/robot_ws/go2_follow_develop/install/setup.bash
source /home/cat/ros2_ws/install/setup.bash
cd /home/cat/ros2_ws
colcon build --symlink-install --packages-select marsdog_vision_interaction marsdog_action_executor
source install/setup.bash
```

重启 Vision 和 Action；运行中的 Python 不会热加载。按原部署启动 RealSense、
SLAM 定位、`person_3d_localization` 与 Nav2。相机、Vision 和深度定位服务必须对应
同一彩色原图，所有节点使用同一 ROS 域及时间源；需要有效地图 TF、代价地图、
`/compute_path_to_pose` 规划服务和 active 的 `/navigate_to_pose`。
Lite3 底盘使用 `use_bridge:=false use_action:=false`，由 Action 管理控制权。

如果还要启动 UWB 仓库的无速度目标生产者，使用：

```bash
ros2 launch go2_uwb_behavior navigation_targets.launch.py
```

其节点名是 `/uwb_navigation_target_node`；旧运动模式仍为
`/uwb_behavior_controller_node`。两种模式共享接口和感知话题，切换需停止旧链后重启，
不能同时启动。视觉定位本身不依赖 UWB 随机目标服务。
`play_alone` 默认继续使用原 `/go2/random_roam`；UWB 点位 → Nav2 是显式可选项，
见 [PLAY_ALONE_LITE3.md](PLAY_ALONE_LITE3.md)。

## 验证

单元测试覆盖人体身份、原图 ROI、失鲜/人脸拒绝、响应关联、取消与终态锁。
`tests/test_visual_nav_chain_ros2.py` 是显式启用的独立 ROS 域测试：使用虚拟深度定位
和虚拟 Nav2 服务，验证真实 ROS 消息经过 Vision → Action → Nav2，且只发送一个 Goal。
软件测试不代表深度、Nav2 实际到点、底盘制动或实机模式切换已验收。

## 通用人、动物和物体接近

`ACT_APPROACH_VISUAL_TARGET -> visual_nav_approach` 使用新增 VisionTask `locate_target_once`，
复用本页的一次固定 Nav2 Goal、真实终态锁与底盘收尾。
Vision 接收 target_type、完整 target_id、stand_off_distance、min_confidence 和可选 required_identity；
成功响应包含 ok、target_id、target_type、vision_epoch、status、navigation_required、target_point、navigation_goal。
物体和动物冻结自己的原始推理帧，不使用人体显示帧或预测旧框。详见 [通用接近](VISUAL_TARGET_APPROACH.md)。
