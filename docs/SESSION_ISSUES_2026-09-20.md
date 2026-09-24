# 历史记录（2026-09-20）本文件记录当时的会话式跟随实现；当前长期 Goal、续租与真实 Result 契约见
[UWB_FOLLOW_ROS2_INTERFACE.md](UWB_FOLLOW_ROS2_INTERFACE.md)。下文的
`follow_priority`、`follow_interrupt_allowlist` 和 `auto_resume` 已不再用于运行时仲裁。

# 2026-09-20 会话问题记录

这次会话实际踩到的问题，按「现象 → 根因 → 修法 → 状态」逐条记录，每条带
`file:line`。两条主线：

- **A. UWB 追随的打断判据** —— 追随跟不住，以及修它时自己造出来的洞。
- **B. Lite3 趴下态死循环** —— 真机日志里那条 `lite3_posture_mismatch`。

C 节是不属于产品 bug、但每次都会真咬一口的工程陷阱，D 节是仍待办。

## 0. 速览

| # | 问题 | 性质 | 状态 |
|---|---|---|---|
| A1 | 追随被任何行为打断，含更低优先级的 | 设计缺口 | 已修 |
| A2 | 「优先级更高才打断」字面实现会挡死安全行为 | 修 A1 时发现的风险 | 已修（白名单） |
| A3 | 抬高 Stage 预算后，安全行为在一个 Stage 期间被拒 | **A1 的修复自己造出来的** | 已修 |
| A4 | 5 秒预算被误读成「跟随时长」 | 认知错误（文档 + 我的报告） | 已修（15.0） |
| A5 | `_current_priority` 不能当追随的水位 | 设计约束 | 已按此设计 |
| A6 | 被打断后是否自动恢复 | 用户决策 | `auto_resume: false` |
| A7 | `avoid_danger`（上游 L0）进不了本仓 | 跨仓库缺口 | 仅记录 |
| B1 | 趴着的狗走导航路线永久卡死 | **2026-09-18 的修复漏了一个入口** | 已修 |
| B2 | 恢复不能放进 `_motion_ready` | 设计边界 | 已用测试钉住 |
| B3 | 「入口」不止一个，补漏要数全 | 方法论 | 见 §B3 清单 |

## A. UWB 追随的打断判据

### A1 追随被任意行为打断

**现象.** 跟随时只要有任何别的行为进来，追随就被挂起；`idle_look_around`（L6）、
`express_happy`（L5）这种明显更低优先级的也一样。用户要求的是「一直跟着，直到有
**等级比他高**的行为进来打断」。

**根因.** `follow_owner` 在上游是**会话状态，不是行为树里 running 的节点**：
`MarsDogTree/marsdog_behavior/ros_node.py:1077-1085` 选中它时只置 `_attention_mode`
并发 `/behavior/attention_tracking`，不派发运动行为。于是行为树自己的抢占仲裁
（`bionic_dog_bt/actions.py:61-63`，规则 5）**根本看不见追随在跑**，任何行为都会被
照常派发下来。本仓的 `_on_execute` 里判据是 `request.behavior_name != "follow_owner"`
——**任何**别的行为都满足。

这一半仲裁只能在本仓补：追随链路的会话消息落在 `ros_node._on_attention_control`，
只有这里同时知道「会话还在」和「来了什么 Goal」。

**修法.** `_on_goal`（`ros_node.py:1795`）加两路互斥的门：

- **锁外分支**（`ros_node.py:1855`）：只要适配器 `session_desired` 为真，追随就占用
  配置的 `follow_priority` 槽位，越不过它的 Goal 一律 `GoalResponse.REJECT`。
- **锁内分支**（`ros_node.py:1819`）：追随自己带 Goal 跑 Stage 时会持执行锁，那条
  分支也接同一张白名单。
- 纯函数 `_follow_yields_to`（`ros_node.py:132`）承载判据，便于脱 rclpy 测试。
- 配置 `config/uwb_follow.yaml:22`：`follow_priority: 1` —— 数字更小 = 更紧急，与上游
  `behaviors.yaml` 和 `_on_goal` 既有规则同一约定。

### A2 「优先级更高才打断」的字面实现会挡死安全行为

**现象（风险）.** `follow_owner` 是 **L1**，而 `respond_person_fall`（人员跌倒）、
`respond_stop_gesture`（停止手势）**也是 L1**；本仓 90 个 L1 行为全都是。照字面实现，
只有 L0 的 `emergency_stop` 能打断追随——**狗继续往前走，而地上躺着人**。

**根因.** 上游 `priority_level` 不是「打断权限」分级，只是行为树里的一个调度水位。

**修法.** `follow_interrupt_allowlist`（`config/uwb_follow.yaml:28`）= 与追随同级、
但必须能打断它的行为，当前两条安全行为。配套启动校验
`config_loader._validate_uwb_session_arbitration`（`config_loader.py:2094`）：表里的名字
必须真实存在于 `behavior_tree_actions.yaml`，**拼错一个字母就在启动时失败**而不是
在那个人倒下的时候。

### A3 抬高 Stage 预算自己造出来的洞

**现象（风险）.** A4 把 `ACT_INTERACT_FOLLOW_OWNER.timeout_sec` 从 5 s 抬到 15 s 之后，
追随自己带 Goal 跑 Stage 的那 15 秒里 `_behavior_execution_lock` 是被持有的。锁内
分支原本只看优先级，**白名单在那段窗口完全不生效**——安全行为会被 REJECT 整整 15 秒。
换句话说：修 A1 的过程让安全行为又多了一个被挡住的窗口。

**根因.** 门有两张（锁内持锁 / 锁外不持锁），只补了一张。而且两张门的判据**不能
共用同一个谓词**：

- 锁外那张问的是「谁**拥有**底盘」→ 用 `session_desired`（survive `suspend()`）。
- 锁内那张问的是「追随现在是不是**正在驱动**底盘」→ 用 `active`。锁被**别的**行为
  持有时追随已被挂起，那时放白名单进来会误抢一个跟追随无关的行为。

**修法.** 新增 `_follow_is_driving`（`ros_node.py:163`）+ 锁内分支的 elif
（`ros_node.py:1819`）。测试
`tests/test_follow_priority_gate.py::test_locked_gate_ignores_the_allowlist_when_not_driving`
钉住「挂起的会话不发抢占权」这条边界。

### A4 5 秒预算被误读成「跟随时长」

**现象.** 文档和我的报告里写着「追随最多只跑 5 秒」。实际上跟随有**两条路径**：

| 路径 | 触发 | 跟随时长 |
|---|---|---|
| **会话路径** | 上游发 `/behavior/attention_tracking` → `update_control` → `start(timeout_sec=0.0)` | **无界**（协议里 0 = 跟到取消为止） |
| **单发路径** | 页面直接发 `follow_owner` Goal、没有会话消息 | `min(ACT_INTERACT_FOLLOW_OWNER.timeout_sec, Goal 剩余预算)` |

会话路径上 5 秒只是「确认 Stage 起来了」的预算：调用点
`uwb_follow_action_adapter.py:493` 传 `cancel_on_budget=standalone`，而
`_run_until_deadline`（`:719`）在 `cancel_on_budget=False` 时到点返回 True 但
**不取消 goal**。单发路径才是真的跟 5 秒——**5 秒走不到任何地方**，
主人还在走、狗停在半路。

**修法.** 两处都改：

- `config/action_catalog.yaml:1470`：`timeout_sec: 5.0 → 15.0`，附注释说明两条路径。
- 文档更正（`docs/UWB_FOLLOW_ROS2_INTERFACE.md` §2、`docs/SIMULATION_PAGE_INTEGRATION.md`）。

### A5 `_current_priority` 不能当追随的水位

追随会话**不持有执行锁**（`_on_attention_control` → `update_control` 不取锁），所以
`_current_priority` 读到的是**上一个前台行为留下的残留值**，不是追随的。因此水位用
独立配置值 `follow_priority`（`ros_node.py:779`），不复用它。

### A6 被打断之后不自动恢复

用户明确选择 `auto_resume: false`（`config/uwb_follow.yaml:36`）：**打断即结束**，狗停
原地等主人重新下令。理由是「刚被紧急情况打断、没人看着，狗自己又开始走」不可接受。

实现上两个标志的语义被刻意分开：

- `session_desired`（`uwb_follow_action_adapter.py:420`、`uwb_follow_adapter.py:88`）
  在 `suspend()` 前后**都为真** → 暂停期间其它低优先级 Goal 照样被拒。
- `_suspended` 必须清零（`resume()`，`uwb_follow_action_adapter.py:644`），否则
  `update_control` 里 `if self._suspended: return True` 会把「主人重新下令」的入口
  永久堵死。

### A7 跨仓库缺口（仅记录）

上游另一个 L0 行为 `avoid_danger` **不在本仓 73 个行为白名单里**，发过来会被
`_on_goal` 当作表外行为拒绝——也就是说它现在**拦不住追随**。要修得按
`config-set-equality-invariants` 那套 7 文件同步规则补 `ACT_*`，本次只记录不改。

## B. Lite3 趴下态死循环

### B1 `lite3_posture_mismatch:require=stand,basic_state=1`（用户报的真机 bug）

**现象.** 真机日志里每 10 秒重复一轮：

```text
Accepted goal: cand_224a26a23e72 -> expressCalmAlone (priority=5)
Navigation: expressCalmAlone -> waypoint __random__ (timeout=30s)
Chassis rejected navigation preflight
Lite3 failure: lite3_posture_mismatch:require=stand,basic_state=1
```

用户的问题原话是「之前这个问题不是修复了么」。

**狗为什么趴着.** `expressCalmAlone` 的 `expression` Stage 是 `random_one`，5 个候选里
`ACT_SPLOOT` 是趴下类（`config/behavior_tree_actions.yaml:709-722`）→ 1/5 概率选中后
`basic_state` 停在 1（`STATE_LIE`）。这正是 2026-09-18 记录在
`behavior-posture-invariant` 里的病例。

**为什么这次没被上次的修复兜住 —— 完整链条.**

```text
expressCalmAlone 在 random_navigation_behaviors 里            ← 关键
  config/navigation_waypoints.yaml:109-112
且这次 launch 带 navigation_enabled:=true                     ← 关键
  → 导航先于任何 step 执行
  → NavigationAdapter._navigate_fixed_with_chassis
      → chassis.prepare_navigation()          lite3_backend.py:386
          → self._motion_ready()              lite3_backend.py:406
              → _simple_precondition_reason({"require": "stand"}, status)
                                                  lite3_backend.py:1197   ← 裸检查，直接报错
          → return False
      → logger.error("Chassis rejected navigation preflight")
  → Stage 失败，**一个 step 都没跑到**
  → 步骤入口那套 _ensure_entry_posture 永远够不着
  → 狗保持趴下，下一次选中该行为重复同一条
```

**根因一句话.** 2026-09-18 的修复只补了**步骤入口**（`_run_simple` / `_run_pose` /
`_run_twist` / `_run_hold`），**漏了导航预检**；而导航预检跑在所有 step **之前**，
所以它一失败，恢复逻辑就再也没机会执行。**失败得越早，恢复越够不着。**

**修法.** `lite3_backend.prepare_navigation`（`lite3_backend.py:403`）入口接上同一套
恢复，和 `_run_twist`（`:882`）同款：

```python
self._clear_error()
if not self._ensure_entry_posture({"require": "stand"}, None, None):
    self._publish_stop()
    return False
if not self._motion_ready(allow_uwb_chain=allow_uwb_chain):
    ...
```

传 `None` 是安全的：`ctx` 只被 `getattr(ctx, "cancel_requested", False)` 读；
`deadline=None` 时恢复由 `posture_recovery_timeout_sec`（默认 6.0）兜底，而
`self._cancel_requested` / `self._should_stop()` 两条中断照旧生效。行为顺序仍是
「先站起来 → 再进 Vision Mode」，`_motion_ready` 本身一个字没动。

### B2 恢复**不能**放进 `_motion_ready`

`_motion_ready`（`lite3_backend.py:1186`）有两个身份，这是最容易踩的一脚：

| 调用点 | 身份 | 趴着的狗应该怎样 |
|---|---|---|
| `prepare_navigation` :406、`_run_twist` :885 | **入口闸门** | 允许恢复 |
| `publish_velocity` :371 | **每 tick 流式检查** | **必须拒绝**——动作执行途中姿势变了就中止，不能中途站起来 |

所以恢复只能接在调用点上，不能下沉进 `_motion_ready`。`test_lite3_backend.py` 里的
`test_raw_motion_check_stays_strict_for_a_lying_dog` 把这条线钉住了（它直接断言
`_motion_ready()` 对趴着的狗仍然报
`lite3_posture_mismatch:require=stand,basic_state=1`）。

`_runtime_motion_ready`（`:1230`）本来就是每 tick 的廉价检查，不含所有权检查，
不需要改。

### B3 「入口」清单——补漏时要数全

这次补漏时把所有可能成为「入口」的地方过了一遍：

| 位置 | 函数 | 处理 |
|---|---|---|
| `lite3_backend.py:599` | `_run_simple`（:563） | 已接（2026-09-18） |
| `lite3_backend.py:805` | `_run_pose`（:777） | 已接（2026-09-18） |
| `lite3_backend.py:882` | `_run_twist`（:867） | 已接（2026-09-18） |
| `lite3_backend.py:940` | `_run_hold`（:933） | 已接（2026-09-18） |
| **`lite3_backend.py:403`** | **`prepare_navigation`（:386）** | **本次补上** |
| `lite3_backend.py:525` | `hold_position` | **故意不接**：用 `{"require": "none"}`，不要求姿势 |
| `lite3_backend.py:1382` | `_wait_for_stable_idle` | **故意不接**：运动**之后**等稳定姿势，那时趴着就是失败，不该起床 |
| `lite3_backend.py:371` | `publish_velocity` | **故意不接**：流式（见 §B2） |

判据：**这条路径上第一个需要站立的检查在哪？** 那个地方必须能恢复，否则整条路径
就是死路——无论后面还有多少步能恢复。

## C. 工程陷阱

### C1 `.py` 是真实副本，不是符号链接

`--symlink-install` 只对 `install(DIRECTORY ...)` 装的东西做符号链接。`config/*.yaml`
是指向源码的符号链接（改内容立刻生效），但 **Python 模块是复制过去的普通文件**。
所以改 `.py` 必须**两棵 install 树各 build 一次**（`/home/cat/xbb/20260707_MarsDogAction`
与 `/home/cat/ros2_ws`），而失败是**静默**的：正在跑的节点照常启动、照常收 Goal，
只是跑的是旧代码。

判据：比字节数，或 `grep -c <本次新增的符号>`。本次验证输出：

```text
install/.../adapters/lite3_backend.py            63557 字节  2026-09-20 17:05:01
ros2_ws/install/.../adapters/lite3_backend.py    63557 字节  2026-09-20 17:05:01
源码                                             63557 字节    ← 三者一致
```

详见记忆 `two-symlink-install-trees`。

### C2 改完代码，运行中的节点不会自动跟上

**判据**：`ps -o lstart`（节点启动时刻）对比 install 树文件 mtime。

本次实例：用户的 `action_executor.launch.py` PID 30572 起于 **16:54:18**，我的 build
完成于 **17:05** → 它加载的是 14:32 那版，**必须重启**。顺带一提，16:54 那次重启已经
带上了追随优先级闸门（14:32 的 build），但**没有**本次的预检修复。

一个更细的旁证：install 树里 `__pycache__/lite3_backend.cpython-310.pyc` 的 mtime 是
14:35 —— 说明节点导入时复用的是 14:32 版源码编译出来的字节码。

### C3 `--symlink-install` 的 build **不会**清掉悬空符号链接

**我上次猜错了。** 上次说「一次 `--packages-select` build 就把它们清掉了」，本次
build 完复查，4 个悬空链接**仍然在** `ros2_ws/install/.../share/.../config/`：

```text
behavior_templates.yaml / emotion_action_pools.yaml
behavior_aliases.yaml   / behaviors.yaml
```

它们指向源码里已删除的文件。运行时没人读（加载器明确不加载这四个），但留着误导人，
得显式删。

### C4 `ps | grep` 会匹配到自己

`echo`/注释里的字面量（`pytest`、`ros2`）会命中自己的 `grep` 进程，看起来像残留。
用拼接模式绕开（`PAT="py""test"`），或写到临时脚本里跑完即删。

### C5 `git stash` 对未跟踪文件无效

想用「stash 掉修复 → 新测试应该失败」做反证时才发现 `lite3_backend.py` 是 `??`
（未跟踪），`git stash push <path>` 直接报 `未匹配任何 git 已知文件`。
改用**把失败点直接写成断言**：

```python
def test_raw_motion_check_stays_strict_for_a_lying_dog():
    ...
    assert not backend._motion_ready()
    assert backend.last_error == "lite3_posture_mismatch:require=stand,basic_state=1"
```

比一次性反证更值——它把这个 bug 的**失败点**永久记在测试里了。

### C6 静默残废的历史样本（CLAUDE.md 的来源）

`jetson2motion` 占着 UDP 43897 **63 分钟**没收 → 用户随后自己起的
`lite3_bringup.launch.py` 里该节点 `bind()` 失败 `exit(1)`；而 `ros2 launch`
**不会因为单个子节点死掉而退出** → 那条 launch 变成「`twist_bridge` /
`lite3_action` / `robot_state_publisher` 都在跑，唯独狗的 UDP 链路没起来」的无声
残废状态。**留下的进程不是「无害占位」，它会静默废掉别人后面整条链路。**

### C7 测试桩的三个坑（本次都踩了）

| 现象 | 根因 | 修法 |
|---|---|---|
| `FileExistsError`（copytree） | 同一 `tmp_path` 目录被多次 `copytree` | 每次 `mkdtemp(dir=tmp_path)` 取新目录 |
| `AttributeError: '_FakeNode' object has no attribute '_follow_session_holds_chassis'` | 辅助函数写成了闭包，桩对象上找不到 | 改成模块级纯函数，调用点显式传参 |
| 断言莫名 `True is False` | 桩的 `emergency_stop()` 没清 `session_desired`，与真实适配器契约不符 | 桩必须照真实契约写（急停结束会话） |

**教训**：桩要照真实对象的契约写，否则测试通过/失败都在说谎。

### C8 不可达死代码

`_control_ownership_ready`（`lite3_backend.py:1208`）在 `:1224` 已经 `return False`，
`:1225-1228` 又重复了 `:1221-1224` 那一整段（`if not conflict: return True` /
`_set_error(...)` / `return False`）。行为无影响，纯噪音。

## D. 仍待办

1. **重启 `action_executor.launch.py`**（PID 30572，16:54:18 起）——本次与上次两批
   修复才会真正生效。按 CLAUDE.md，正在跑的进程不由我关。
2. **真狗验收**（需操作员在场）：
   - 追随闸门：起追随 → 派 `sit_down`（L1）期望 REJECT 且继续跟；派
     `respond_stop_gesture` 期望追随暂停、狗停下、**结束后不自动恢复**；重新下
     「跟着我」期望重新起来；按 `emergency_stop` 期望停下且不再恢复。
   - 姿势恢复的导航路线：把狗趴下 → 触发 `expressCalmAlone` 之类在
     `random_navigation_behaviors` 里的行为 → 期望看到起立发生在**进入 Vision Mode
     之前**，而不是 `Chassis rejected navigation preflight`。
   - 清单见 `docs/LITE3_ROS2_INTEGRATION.md` §10 第 10 条。
3. `avoid_danger` 跨仓库缺口（§A7）——要按 7 文件同步规则补 `ACT_*`。
4. 删 4 个悬空符号链接（§C3）。
5. 删 `lite3_backend.py:1225-1228` 死代码（§C8）。
6. 是否 `git commit`——`lite3_backend.py` 等文件目前仍是未跟踪状态。

## 附：本次改动的文件

| 文件 | 改动 |
|---|---|
| `marsdog_action_executor/adapters/lite3_backend.py` | `prepare_navigation` 接入口期姿势恢复（`§B1`） |
| `marsdog_action_executor/ros_node.py` | `_follow_yields_to` / `_follow_session_holds_chassis` / `_follow_is_driving` + `_on_goal` 两路门 |
| `marsdog_action_executor/config_loader.py` | `_validate_uwb_session_arbitration`（启动期校验） |
| `marsdog_action_executor/adapters/uwb_follow_action_adapter.py` | `session_desired` / `auto_resume` |
| `marsdog_action_executor/adapters/uwb_follow_adapter.py` | 同上（go2/agv 旧适配器） |
| `config/uwb_follow.yaml` | `follow_priority` / `follow_interrupt_allowlist` / `auto_resume` |
| `config/action_catalog.yaml` | `ACT_INTERACT_FOLLOW_OWNER.timeout_sec: 5.0 → 15.0` |
| `tests/test_follow_priority_gate.py` | 新增（14 个用例） |
| `tests/test_uwb_follow_adapter.py` | 会话仲裁配置块 |
| `tests/test_uwb_follow_action_adapter.py` | `session_desired` / `auto_resume` 用例 |
| `tests/test_ros_goal_params_boundary.py` | 桩补 `_uwb_follow_adapter = None` |
| `tests/test_lite3_backend.py` | 新增 3 个（61 个用例） |
| `docs/LITE3_ROS2_INTEGRATION.md` §4.1 / §10 | 导航预检也是入口 |
| `docs/UWB_FOLLOW_ROS2_INTERFACE.md` §2 / §3 / §6 | 「谁会打断追随」+ 结果码更正 |
| `docs/ARCHITECTURE.md` §7、`docs/HANDOFF.md` §4、`docs/SIMULATION_PAGE_INTEGRATION.md` | 同步 |

测试：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/ -q`
→ **412 passed, 9 skipped**（改动前基线 376 passed）。

相关文档：[LITE3_ROS2_INTEGRATION.md](LITE3_ROS2_INTEGRATION.md)、
[UWB_FOLLOW_ROS2_INTERFACE.md](UWB_FOLLOW_ROS2_INTERFACE.md)、
[HANDOFF.md](HANDOFF.md)
