# 唤醒后的随机表现与重复唤醒

Tree 先执行 `respond_owner_call` 转向，再使用本次 `wake_id` 对应的声纹结果。
主人或家人匹配成功且绑定有效人体时，先执行 `approach_voice_caller`，成功后执行
`respond_wake_in_place`。陌生人、身份不确定、声纹超时，或无法绑定人体时，直接在
转向后执行 `respond_wake_in_place`。转向或已开始的靠近失败、取消、超时，不接表现动作。

`respond_wake_in_place` 是单独的 30 秒 Goal；不占用靠近 Goal 的 160 秒预算。
`expression` Stage 每次以 `random_one` 从下列候选中抽取一项，允许连续两次相同。
单动作执行一次；动作组执行完整序列，不从组内再抽一个动作。

| 类型 | Action | 表现 |
|---|---|---|
| 单动作 | `ACT_RAISE_HEAD` | 抬身俯仰表现 |
| 单动作 | `ACT_TWIST_WAIST_LEFT_RIGHT` | 左右扭腰 |
| 单动作 | `ACT_STRETCH` | 舒展 |
| 动作组 | `ACT_ROAM_LOOK_DOWN_UP` | 低头 → 抬头 |
| 动作组 | `ACT_ROAM_SCAN_HEAD_UP` | 左右观察 → 抬头 |
| 动作组 | `ACT_ROAM_STRETCH_SCAN` | 舒展 → 左右观察 |

候选池在 `config/behavior_tree_actions.yaml` 的 `respond_wake_in_place` 中配置。
Action 要求 `mobility_policy=in_place`，只允许已审核的姿态动作和实体底盘路由，
拒绝导航前置、移动动作和 mock 回退。Go2 的姿态、观察、扭腰主要是机身 Euler
表达，不代表独立头部运动；已有 Lite3 物理计划和验收开关继续生效。

Tree 的 `EXPRESSING` 阶段保持 Voice 的有限租约并关闭 attention 底盘转向。
表现完成或失败后释放租约并重置空闲计时；只有靠近成功且表现成功时恢复原有
到达后 attention。收到正式语音指令时可抢占表现，并清除未执行的唤醒表现候选。

会话内可以再次硬件唤醒。Voice 刷新同一 `interaction_id` 的空闲计时，生成新
`wake_id`，重新识别声纹。当前 `idle_timeout_sec=20.0`，`max_duration_sec=0.0`，
同词 0.8 秒内重复通知被去重。Tree 取消旧唤醒行为，等待真实终态后允许新行为执行；
旧声纹、视觉和动作结果不能继续旧流程。每次唤醒使用独立 hold token，避免迟到的
旧租约释放影响新唤醒。

部署需重建 Tree、Action 并重启相应节点。软件测试覆盖随机池、实际 Go2 API 序列、
身份分流、取消、租约和重复唤醒；新增唤醒编排仍需真机验收。
