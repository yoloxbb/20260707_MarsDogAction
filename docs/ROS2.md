# MarsDog Action Executor — ROS2 集成说明

> 本文档描述 `marsdog_action_executor` 包如何与 ROS2 集成，包括 Action Server、Topic、Launch、编译构建以及通信协议。

## 目录

1. [ROS2 环境要求](#1-ros2-环境要求)
2. [三包架构与通信拓扑](#2-三包架构与通信拓扑)
3. [ROS2 Action Server：/execute_behavior](#3-ros2-action-serverexecute_behavior)
4. [Launch 系统](#4-launch-系统)
5. [Debug Topic 接口](#5-debug-topic-接口)
6. [ROS2 编译与构建](#6-ros2-编译与构建)
7. [通信协议 (params_json 契约)](#7-通信协议-params_json-契约)
8. [Action 消息结构](#8-action-消息结构)
9. [与行为树的通信流程](#9-与行为树的通信流程)
10. [无 ROS2 环境开发与测试](#10-无-ros2-环境开发与测试)
11. [手动调试与验证](#11-手动调试与验证)
12. [ROS2 兼容性设计](#12-ros2-兼容性设计)
13. [控制器适配层与 ROS2 Service](#13-控制器适配层与-ros2-service)
14. [扩展指南](#14-扩展指南)

---

## 1. ROS2 环境要求

| 项目 | 要求 |
|------|------|
| ROS2 发行版 | **Humble** (或其他兼容版本) |
| 构建工具 | `colcon` |
| Python | ≥ 3.10 |
| 核心依赖 | `rclpy`, `std_msgs`, `action_msgs` |
| 接口依赖 | `marsdog_interfaces` (提供 `ExecuteBehavior.action`) |

---

## 2. 三包架构与通信拓扑

```
┌──────────────────────────────────────────────────────────────────┐
│                     marsdog_interfaces                           │
│                 (公共接口: ExecuteBehavior.action)                 │
└──────────────────────────┬───────────────────────────────────────┘
                           │ import
              ┌────────────┴────────────┐
              ▼                         ▼
┌─────────────────────────┐   ┌─────────────────────────────────────┐
│   marsdog_behavior      │   │   marsdog_action_executor (本包)     │
│   (Action Client)       │   │   (Action Server)                   │
│                         │   │                                     │
│   行为树 / 决策引擎       │   │   Action Server: /execute_behavior  │
│                         │   │   Debug Topics: /debug/execute_*     │
└────────────┬────────────┘   └──────────────────┬──────────────────┘
             │                                   │
             │    ExecuteBehavior Goal/Result     │
             └─────────── Action ─────────────────┘
```

| 包 | 角色 | ROS2 节点 |
|---|---|---|
| `marsdog_interfaces` | 接口定义 | 无（纯 action/msg 定义） |
| `marsdog_behavior` | 行为树 / Action Client | 决策节点 |
| **`marsdog_action_executor`** | **Action Server** | `action_executor_node` |

**通信关系**：
- **上行**：`marsdog_behavior` 通过 `/execute_behavior` Action 向本包下发行为请求
- **下行**：本包通过 Controller Adapter 层驱动硬件（Motion/Gimbal/Audio/Navigation/Perception/Expression Service）
- **调试**：本包通过 `/debug/execute_behavior/*` Topic 发布调试信息

---

## 3. ROS2 Action Server：/execute_behavior

### 3.1 概述

`ActionExecutorNode` ([ros_node.py:65](../marsdog_action_executor/ros_node.py#L65)) 是本包的唯一 ROS2 Node。它将自身注册为 `/execute_behavior` Action Server。

### 3.2 节点信息

| 属性 | 值 |
|------|-----|
| Node 名称 | `action_executor_node` |
| Action 名称 | `/execute_behavior` |
| Action 类型 | `marsdog_interfaces.action.ExecuteBehavior` |
| 反馈频率 | **10 Hz** (`TICK_RATE = 0.1s`) |
| Executable | `action_executor_node` |

### 3.3 Action Server 回调链

```
Goal 到达
  │
  ▼
_on_goal(goal_request) → GoalResponse
  │  ├─ 检查统一注册表 (catalog + canonical + alias)
  │  ├─ 未知 behavior → REJECT
  │  └─ 已知 behavior → ACCEPT
  ▼
_on_accepted(goal_handle)
  │  ├─ goal_handle.execute() → 启动异步执行
  │  └─ 发布 Goal 到 /debug/execute_behavior/goal
  ▼
_on_execute(goal_handle)  [async]
  │  ├─ GoalParser: 解析 params_json → ExecutionContext
  │  ├─ BehaviorResolver: alias 解析 → 参数注入 → 交互回退
  │  ├─ Planner: 生成 ActionPlan
  │  ├─ Executor: 启动执行序列
  │  │
  │  └─ 循环 (10Hz):
  │       ├─ 检查 is_cancel_requested → cancel → CANCELED
  │       ├─ executor.tick() → events
  │       ├─ ExecutionFeedback → goal_handle.publish_feedback()
  │       │                      + /debug/execute_behavior/feedback
  │       └─ ExecutionResult → goal_handle.succeed()
  │                            + /debug/execute_behavior/result
  ▼
Result 返回 (SUCCESS / CANCELED / TIMEOUT / FAILED)
```

### 3.4 Goal 验证逻辑

```python
# 三级接受机制 (ros_node.py:109-134)
# Step 1: 检查统一注册表 (catalog entries + canonical + aliases)
if name in _ALL_ACCEPTABLE_BEHAVIORS:
    return GoalResponse.ACCEPT

# Step 2: 尝试 alias 解析
canonical, _ = _resolver._resolve_name(name)
if canonical is not None and canonical in _ALL_ACCEPTABLE_BEHAVIORS:
    return GoalResponse.ACCEPT

# Step 3: REJECT
return GoalResponse.REJECT
```

### 3.5 取消处理

| 回调 | 行为 |
|------|------|
| `_on_cancel` | 始终返回 `CancelResponse.ACCEPT` |
| `_on_execute` 内部 | 调用 `executor.cancel_goal(gid)` → 返回 `CANCELED` |

---

## 4. Launch 系统

### 4.1 Launch 文件

[launch/action_executor.launch.py](../launch/action_executor.launch.py)：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="marsdog_action_executor",
            executable="action_executor_node",
            name="action_executor_node",
            output="screen",
            parameters=[],
        ),
    ])
```

### 4.2 启动命令

```bash
# 方式 1: ros2 run
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run marsdog_action_executor action_executor_node

# 方式 2: ros2 launch
ros2 launch marsdog_action_executor action_executor.launch.py
```

### 4.3 安装路径

CMakeLists.txt ([CMakeLists.txt:31-48](../CMakeLists.txt#L31-L48)) 定义安装规则：

| 源 | 安装目标 |
|---|---------|
| `scripts/action_executor_node` | `lib/marsdog_action_executor/` |
| `launch/*.launch.py` | `share/marsdog_action_executor/launch/` |
| `config/*.yaml` | `share/marsdog_action_executor/config/` |
| `marsdog_action_executor/*.py` | `PYTHON_INSTALL_DIR/marsdog_action_executor/` |

---

## 5. Debug Topic 接口

### 5.1 Topic 列表

| Topic | 类型 | 方向 | QoS | 说明 |
|-------|------|------|-----|------|
| `/debug/execute_behavior/goal` | `std_msgs/String` (JSON) | → | queue=10 | Goal 到达时发布 |
| `/debug/execute_behavior/feedback` | `std_msgs/String` (JSON) | → | queue=10 | 10Hz 执行进度 |
| `/debug/execute_behavior/result` | `std_msgs/String` (JSON) | → | queue=10 | 执行结束时发布 |

> **注意**：这些 Topic 仅供调试使用。业务逻辑应依赖 `/execute_behavior` Action 的 Feedback/Result，不应依赖 Debug Topic。

### 5.2 已废弃的旧 Topic

设置环境变量 `MARSDOG_LEGACY_DEBUG_TOPICS=1` 可启用旧版 Topic（无 `/debug` 前缀）：

- `/execute_behavior/goal`
- `/execute_behavior/feedback`
- `/execute_behavior/result`

> 这些旧 Topic 将在未来版本中移除。

### 5.3 Debug 消息示例

**Goal (JSON)**:
```json
{
  "goal_id": "t1",
  "behavior_name": "eatNormally",
  "priority_level": 3,
  "params": {"source": "need", "level": "TRIGGERED"},
  "timeout_sec": 30.0,
  "timestamp": 1720953400.123
}
```

**Feedback (JSON, 10Hz)**:
```json
{
  "goal_id": "t1",
  "behavior_name": "eatNormally",
  "status": "RUNNING",
  "progress": 0.0,
  "current_stage": "prepare",
  "current_action": "ACT_SNIFF_BOWL_EDGE",
  "safe_to_interrupt": true,
  "message": "Step 1/4: ACT_SNIFF_BOWL_EDGE"
}
```

**Result (JSON)**:
```json
{
  "goal_id": "t1",
  "behavior_name": "eatNormally",
  "status": "SUCCESS",
  "result": "completed",
  "reason": "All action steps finished",
  "duration_sec": 8.5,
  "reward": 1.0
}
```

---

## 6. ROS2 编译与构建

### 6.1 package.xml

[package.xml](../package.xml) 采用 `format="3"` 格式：

```xml
<package format="3">
  <name>marsdog_action_executor</name>
  <version>0.1.0</version>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>action_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**关键说明**：
- `rosidl_default_generators` 用于生成本地 `ExecuteBehavior.action`（过渡期 fallback）
- 正式接口 `marsdog_interfaces` 未列为 `depend`（待接口包就绪后添加）

### 6.2 CMakeLists.txt

[CMakeLists.txt](../CMakeLists.txt) 关键构建步骤：

```cmake
# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# 2. 生成 Action 接口 (过渡期 fallback)
rosidl_generate_interfaces(${PROJECT_NAME} "action/ExecuteBehavior.action")

# 3. 安装 Python 模块
install(DIRECTORY .../marsdog_action_executor/
        DESTINATION ${PYTHON_INSTALL_DIR}/marsdog_action_executor)

# 4. 安装 launcher script
install(PROGRAMS scripts/action_executor_node
        DESTINATION lib/${PROJECT_NAME})

# 5. 安装 launch 文件和配置文件
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
```

### 6.3 完整编译步骤

```bash
# 激活 ROS2 环境
source /opt/ros/humble/setup.bash

# 进入工作空间
cd ~/ros2_ws

# 编译接口包 (需先编译)
colcon build --packages-select marsdog_interfaces

# 编译执行器包
colcon build --packages-select marsdog_action_executor --symlink-install

# 激活工作空间
source install/setup.bash

# 验证
ros2 run marsdog_action_executor action_executor_node
```

### 6.4 pyproject.toml

[pyproject.toml](../pyproject.toml) 定义了 Python 侧依赖和脚本入口：

```toml
[project.scripts]
marsdog-action-demo = "marsdog_action_executor.standalone_demo:main"

[project]
dependencies = ["pyyaml>=6.0", "pytest>=9.0"]
```

---

## 7. 通信协议 (params_json 契约)

### 7.1 参数字段规范

`params_json` 是行为树下发的 JSON 字符串，由 [execution_context.py](../marsdog_action_executor/execution_context.py) 统一标准化。

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `source` | string | 否 | `need` / `emotion` / `audio` / `external` |
| `trigger_event` | string | 否 | 如 `NEED_HUNGER_TRIGGERED` |
| `intent` | string | 否 | 如 `eat_normal` |
| `priority_level` | int | 否 | 0-6，默认 5 |
| `level` | string | 否 | `LOW` / `MID` / `HIGH` / `TRIGGERED` / `OVERFLOW` / `RECOVERED` |
| `intensity` | number | 否 | 0-100 数值 |
| `interaction_mode` | string | 否 | `solo` / `interactive` |
| `interactive` | bool | 否 | 与 `interaction_mode` 互推导 |
| `target` | dict | 否 | 目标信息 `{target_type, identity, visible, ...}` |
| `sleep_depth` | string | 否 | `shallow` / `deep` |
| `elimination_type` | string | 否 | `pee` / `poop` |
| `charger_known` | bool | 否 | 充电桩已知 |
| `charger_available` | bool | 否 | 充电桩可用 |
| `object_category` | string | 否 | 物品类别 |
| `random_seed` | int | 否 | 固定随机种子 (调试/测试用) |

### 7.2 设计原则

- **缺失可选字段** → 使用默认值，不崩溃
- **未知字段** → 保留在 `metadata` 字典中，透传不丢弃
- **非法值** → 记录 warning，替换为安全默认值
- `is_valid=False` 时读取 `error_reason` 了解原因

### 7.3 需求触发示例

```json
{
  "source": "need",
  "trigger_event": "NEED_HUNGER_TRIGGERED",
  "intent": "eat_normal",
  "priority_level": 3,
  "level": "TRIGGERED",
  "intensity": 82,
  "interaction_mode": "solo"
}
```

### 7.4 情绪触发示例

```json
{
  "source": "emotion",
  "trigger_event": "EMO_JOY_MID",
  "intent": "express_joy",
  "level": "MID",
  "interaction_mode": "interactive",
  "target": {
    "target_type": "human",
    "identity": "owner",
    "visible": true
  }
}
```

### 7.5 别名自动解析

旧行为名通过 alias 自动映射到 canonical name：

```
wagTailFast  →  expressJoy    (level=MID,  interaction_mode=interactive)
hideAway     →  expressFear   (level=HIGH, interaction_mode=solo)
headTilt     →  expressCuriosity (level=LOW, interaction_mode=interactive)
```

alias 注入的参数会自动合并到 `params_json`，并写入 `ExecutionContext` 的 typed fields。

### 7.6 交互模式回退

当 `interaction_mode=interactive` 但 `target` 为空时，自动回退为 `solo`：

```
interaction fallback: no valid target, requested=interactive → resolved=solo
```

此信息会记录在 `ctx.metadata["fallback_reason"]` 中。

---

## 8. Action 消息结构

### 8.1 ExecuteBehavior.action

Action 定义位于 `marsdog_interfaces` 包（过渡期 fallback 在本地 `action/ExecuteBehavior.action`）。

**Goal**:
```
string goal_id          # 唯一标识 (UUID)
string behavior_id      # 行为ID (行为树侧)
string behavior_name    # 行为名, 如 "eatNormally"
int32  priority_level   # 优先级 0-6
string params_json      # JSON 参数
float64 timeout_sec     # 超时(秒)
```

**Result**:
```
string goal_id
string behavior_id
string behavior_name
string status           # SUCCESS / FAILED / CANCELED / TIMEOUT
string result           # completed / failed / canceled / timeout
string reason           # 人类可读的原因
float64 reward          # -1.0 ~ 1.0
string emotion_delta_json  # 情绪变化 (JSON)
string need_delta_json     # 需求变化 (JSON)
```

**Feedback**:
```
string goal_id
string behavior_id
string behavior_name
string status           # 始终 RUNNING
float64 progress        # 0.0 ~ 1.0
bool    safe_to_interrupt  # 当前是否可安全中断
string current_action   # 当前执行的 action_id
string message          # 人类可读的状态消息
```

### 8.2 兼容性导入策略

[ros2_compat.py](../marsdog_action_executor/ros2_compat.py) 实现两级导入策略：

```
优先级 1: marsdog_interfaces.action.ExecuteBehavior   (正式接口)
优先级 2: marsdog_action_executor.action.ExecuteBehavior (过渡期 fallback, DEPRECATED)
```

```python
from .ros2_compat import HAS_ROS2, get_execute_behavior_action

if HAS_ROS2:
    ExecuteBehavior = get_execute_behavior_action()
```

---

## 9. 与行为树的通信流程

### 9.1 完整时序

```
Behavior Tree (marsdog_behavior)           ActionExecutor (本包)
─────────────────────────────────          ─────────────────────
                               │
  send_goal(ExecuteBehavior.Goal)│
    goal_id="t1"                │
    behavior_name="eatNormally" │
    params_json="{...}"         │
    timeout_sec=30.0            │
                               │─────►
                               │           _on_goal() → ACCEPT
                               │           _on_accepted() → 启动执行
                               │
                               │◄────────  Goal accepted (implicit)
                               │
                               │◄────────  Feedback (10Hz):
                               │             progress=0.0, current_action="..."
                               │             progress=0.25, ...
                               │             progress=0.5, ...
                               │             ...
                               │
                               │           [执行中: GoalParser → Resolver → Stages → Units]
                               │
                               │◄────────  Feedback:
                               │             progress=1.0
                               │
                               │◄────────  Result:
                               │             status="SUCCESS"
                               │             reward=1.0
                               │
                               │
  get_result()                 │
    → SUCCESS, reward=1.0      │
```

### 9.2 取消流程

```
Behavior Tree                   ActionExecutor
─────────────                   ──────────────
  cancel_goal(goal_id)  ──────►
                               _on_cancel() → ACCEPT
                               goal_handle.is_cancel_requested → True
                               executor.cancel_goal(gid)
                               goal_handle.canceled()
                               ──────►  Result: status="CANCELED"
```

---

## 10. 无 ROS2 环境开发与测试

### 10.1 设计思路

本包在架构上将 **ROS2 通信层** 与 **业务逻辑** 完全分离：

```
ros_node.py           ← ROS2 依赖 (rclpy, action server)
    │
    ▼ 调用但不依赖 ROS2 类型
goal_parser.py        ← 无 ROS2 依赖
behavior_resolver.py  ← 无 ROS2 依赖
planner.py            ← 无 ROS2 依赖
executor.py           ← 无 ROS2 依赖
stage_executor.py     ← 无 ROS2 依赖
controller_adapters.py ← 无 ROS2 依赖
```

核心逻辑（Plan / Execute / Resolve / Filter）可以通过 `pytest` 直接在 **CI 或本地 venv** 中测试，无需启动 ROS2。

### 10.2 条件导入

```python
# ros2_compat.py
try:
    import rclpy
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
```

所有依赖 ROS2 的代码受 `if HAS_ROS2` 保护。

### 10.3 无 ROS2 Demo

```bash
# 使用 uv (纯 Python venv，不需要 ROS2)
uv run marsdog-action-demo                    # 默认: excretion_request
uv run marsdog-action-demo eatNormally        # 指定行为
uv run marsdog-action-demo wagTailFast 42     # 指定行为 + 固定种子

# 输出示例:
# Goal:      demo_001
# Behavior:  eatNormally
# Stages:    4
# Steps:     4
# Duration:  8.2s
#
#   Stage 1: [random_one] prepare  → ACT_SNIFF_BOWL_EDGE (2.1s, interrupt=yes)
#   Stage 2: [random_one] eating   → ACT_LICK_FOOD (3.0s, interrupt=yes)
#   Stage 3: [random_one] interaction → ACT_PAUSE_AND_LOOK_AT_OWNER (1.5s, interrupt=yes)
#   Stage 4: [random_one] exit     → ACT_LICK_LIPS_OR_NOSE (1.6s, interrupt=yes)
#
#   [demo_001] RESULT: SUCCESS
#     reason:   All action steps finished
#     reward:   1.0
#     duration: 8.2s
```

### 10.4 运行测试

```bash
uv run pytest -q                                    # 全部 619 tests
uv run pytest -q tests/test_catalog.py              # 目录完整性
uv run pytest -q tests/test_catalog_new.py          # 新行为专项
uv run pytest -q tests/test_executor.py             # 执行引擎
uv run pytest -q tests/test_planner.py              # weighted / alias / seed
uv run pytest -q tests/test_new_executor.py         # 新 API + adapter
```

所有测试在无 ROS2 环境下正常运行。

---

## 11. 手动调试与验证

### 11.1 使用 ros2 action send_goal

```bash
# 生理行为: 正常进食
ros2 action send_goal /execute_behavior marshdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 't1', behavior_name: 'eatNormally', priority_level: 3, \
    params_json: '{\"source\":\"need\",\"level\":\"TRIGGERED\"}', timeout_sec: 30.0}"

# 情绪行为: 表达快乐
ros2 action send_goal /execute_behavior marshdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 't2', behavior_name: 'expressJoy', priority_level: 5, \
    params_json: '{\"source\":\"emotion\",\"level\":\"MID\",\"interaction_mode\":\"interactive\", \
    \"target\":{\"target_type\":\"human\",\"visible\":true}}', timeout_sec: 10.0}"

# 别名: wagTailFast → expressJoy (自动注入 level=MID, interaction_mode=interactive)
ros2 action send_goal /execute_behavior marshdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 't3', behavior_name: 'wagTailFast', priority_level: 5, \
    params_json: '{}', timeout_sec: 10.0}"

# 充电行为
ros2 action send_goal /execute_behavior marshdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 't4', behavior_name: 'recharge', priority_level: 2, \
    params_json: '{\"charger_known\":true,\"charger_available\":true}', timeout_sec: 120.0}"
```

### 11.2 监听 Debug Topic

```bash
# 监听 Goal
ros2 topic echo /debug/execute_behavior/goal

# 监听 Feedback (10Hz)
ros2 topic echo /debug/execute_behavior/feedback

# 监听 Result
ros2 topic echo /debug/execute_behavior/result
```

### 11.3 查看 Action Server 状态

```bash
# 列出所有 Action Server
ros2 action list

# 查看 Action 接口定义
ros2 action info /execute_behavior
```

---

## 12. ROS2 兼容性设计

### 12.1 HAS_ROS2 条件保护

所有 ROS2 相关代码受两级保护：

1. **模块级**：`ros2_compat.py` 提供 `HAS_ROS2` 标志
2. **类定义级**：`ros_node.py` 中的 `ActionExecutorNode` 类在 `if HAS_ROS2` 块内定义

### 12.2 接口导入优先级

```
marsdog_interfaces.action.ExecuteBehavior   ← 优先（公共接口包）
marsdog_action_executor.action.ExecuteBehavior ← 过渡期 fallback (DEPRECATED)
```

### 12.3 Duck-typing Goal 处理

`GoalParser.parse_from_ros_goal()` 使用 duck-typing 访问 goal request 字段，不直接依赖 Action 类型，提高了灵活性。

### 12.4 依赖关系图

```
                     ┌──────────────┐
                     │   rclpy      │  ← 运行时必需
                     └──────┬───────┘
                            │
          ┌─────────────────┼─────────────────┐
          │                 │                 │
          ▼                 ▼                 ▼
   ros_node.py       debug_publishers.py   launch file
   (ActionServer)    (String publishers)   (ros2 launch)
          │                 │
          │                 │
          ▼                 ▼
   ros2_compat.py   ←── std_msgs/String
   (条件导入层)
```

---

## 13. 控制器适配层与 ROS2 Service

本包通过 6 个 Adapter 驱动硬件，每个 Adapter 对应下游 ROS2 Service/Topic：

| Adapter | 下游 ROS2 接口 | 接口类型 | 路由的动作 |
|---------|---------------|---------|-----------|
| Motion | `/motion/execute_motion` | Service / Action | 默认（多数动作） |
| Gimbal | `/gimbal/set_target` | Service / Action | `ACT_TILT_HEAD` 等 |
| Audio | — | Service | `ACT_BARK_*`, `ACT_WHINE_*`, `ACT_GROWL_*` |
| Navigation | `/navigation/navigate_to` | Action | `ACT_RETURN_TO_CHARGER`, `ACT_RUN_ZOOMIES` |
| Perception | — | Service / Topic | `TASK_APPROACH_*`, `ACT_FETCH_TOY` |
| Expression | `/expression/play` | Service | `ACT_DILATE_PUPILS` |

> **当前状态**：所有 Adapter 均为 Mock 实现（[controller_adapters.py](../marsdog_action_executor/controller_adapters.py)）。真实硬件 Adapter 为待完成项。

路由配置见 [config/controller_routes.yaml](../config/controller_routes.yaml)。

### Adapter 注册机制

```python
from marsdog_action_executor.controller_adapters import register_adapter, create_adapter

# 注册真实硬件 Adapter
register_adapter("motion", RealMotionAdapter)
register_adapter("gimbal", RealGimbalAdapter)

# 创建实例
adapter = create_adapter("motion", device="/dev/ttyUSB0")
```

---

## 14. 扩展指南

### 14.1 添加新 Behavior（需 ROS2 集成）

1. 在 [config/behavior_templates.yaml](../config/behavior_templates.yaml) 添加 behavior → stages 定义
2. 在 [behavior_resolver.py:145-160](../marsdog_action_executor/behavior_resolver.py#L145-L160) 的 `_CANONICAL_BEHAVIORS` 集合中注册
3. 在 [result_evaluator.py](../marsdog_action_executor/result_evaluator.py) 添加行为级成功条件（如需）
4. 重新编译：`colcon build --packages-select marsdog_action_executor`

### 14.2 添加新 Controller Adapter

1. 实现 `BaseControllerAdapter` 子类（继承 [controller_adapters.py:30](../marsdog_action_executor/controller_adapters.py#L30)）
2. 调用 `register_adapter("adapter_name", YourAdapter)` 注册
3. 在 [config/controller_routes.yaml](../config/controller_routes.yaml) 添加 action → adapter 路由

### 14.3 添加旧行为别名

在 [config/behavior_aliases.yaml](../config/behavior_aliases.yaml) 添加：

```yaml
old_name:
  resolved_behavior_name: canonical_name
  injected_params:
    level: MID
    interaction_mode: interactive
  alias_reason: "migration from legacy naming"
```

无需重新编译（YAML 文件在安装目录中动态加载）。

### 14.4 添加新条件

1. 在 [eligibility_checker.py](../marsdog_action_executor/eligibility_checker.py) 实现 `_cond_*` 函数
2. 注册：`register_condition("condition_name", _cond_*)`
3. 在 behavior templates 的 `conditions` 列表中使用

---

## 附录 A：关键文件索引（ROS2 相关）

| 文件 | 行数 | ROS2 职责 |
|------|------|----------|
| `ros_node.py` | 316 | Action Server Node，Goal/Cancel/Accept/Execute 回调 |
| `ros2_compat.py` | 93 | `HAS_ROS2` 标志 + ExecuteBehavior 优先导入 |
| `debug_publishers.py` | 130 | Debug Topic 发布器 |
| `launch/action_executor.launch.py` | 24 | Launch 文件 |
| `package.xml` | 31 | ROS2 包清单 |
| `CMakeLists.txt` | 50 | colcon 构建 + 接口生成 + 安装规则 |
| `setup.py` | 39 | Python entry point + data_files |

## 附录 B：常用命令速查

```bash
# === 编译 ===
cd ~/ros2_ws
colcon build --packages-select marsdog_interfaces marsdog_action_executor --symlink-install
source install/setup.bash

# === 启动 ===
ros2 run marsdog_action_executor action_executor_node
ros2 launch marsdog_action_executor action_executor.launch.py

# === 调试 ===
ros2 action list                                  # 列出 Action Server
ros2 action info /execute_behavior                # Action 详情
ros2 topic echo /debug/execute_behavior/feedback  # 监听调试反馈

# === 发送请求 ===
ros2 action send_goal /execute_behavior \
  marsdog_interfaces/action/ExecuteBehavior \
  "{goal_id: 'test1', behavior_name: 'expressJoy', priority_level: 5, \
    params_json: '{\"level\":\"MID\",\"interaction_mode\":\"interactive\"}', \
    timeout_sec: 10.0}"

# === 测试 (无 ROS2) ===
uv run pytest -q                                  # 619 tests
uv run marsdog-action-demo eatNormally            # 独立 Demo
```
