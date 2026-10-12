#!/usr/bin/env python3
"""Regenerate the complete Go2 action review table from validated config."""
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.adapters.go2_sport_backend import SPORT_API_IDS


def main():
    loader = ConfigLoader(ROOT / "config")
    loader.load_all()
    plans = loader.get_go2_action_plans()
    routes = loader.get_controller_routes("go2")
    unsupported = loader.go2_sport_config["unsupported_actions"]
    references = {unit_id: set() for unit_id in loader.action_catalog}
    for name in loader.get_behavior_names():
        behavior = loader.get_behavior_template(name, chassis_type="go2")
        for stage in behavior["stages"]:
            for candidate in stage.get("candidates", []):
                references[candidate["unit_id"]].add(name + "/" + stage["stage_id"])
    target = ROOT / "docs" / "GO2_ACTION_AUDIT.csv"
    with target.open("w", encoding="utf-8-sig", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["动作 ID", "通用语义", "Go2 分类", "控制路由", "执行序列", "命令姿态",
                         "动作时限秒", "收尾准备", "完成依据及限制", "行为及阶段"])
        for unit_id in sorted(references):
            catalog = loader.action_catalog[unit_id]
            plan = plans.get(unit_id, {})
            route = routes[unit_id]
            sequence = plan.get("sequence", [])
            steps = []
            for item in sequence:
                command = item["command"]
                if command == "nav2_spin":
                    steps.append(f"/spin turns={item['turns']} allowance={item['duration_sec']}s")
                else:
                    args = ",".join(f"{key}={item[key]}" for key in ("x", "y", "z", "data") if key in item)
                    steps.append(f"{command}({SPORT_API_IDS[command]};{args}) {item['duration_sec']}s")
            if route == "go2":
                kind, detail = plan["fidelity"], plan["description"]
                detail += " 导航只认真实 Result；SportMode 仅发布及等待，无固件 ACK/完成反馈。"
            elif route == "unsupported":
                kind, detail = "unsupported", unsupported[unit_id]
            elif route == "mock":
                kind, detail = "audio_policy", "quiet 音频策略，无身体动作。"
            elif route == "uwb_roam":
                kind, detail = "external_unavailable", "当前 Go2 未注册 uwb_roam 控制器，必需阶段会明确失败。"
            else:
                kind, detail = "external", "沿用独立控制器；可用性取决于对应服务、导航和运行开关。"
            cleanup = (
                "StopMove -> BalanceStand；完整行为链再统一收尾"
                if route == "go2" and plan.get("commanded_posture") == "standing"
                else "单元保持目标姿态；完整行为链成功后 StandUp(必要时等待2.5s) -> BalanceStand；显式姿态/停止行为除外"
                if route == "go2" else "由独立控制器负责；成功行为链统一收尾，导航 Result 不改判"
            )
            writer.writerow([unit_id, catalog.get("description", ""), kind, route, " -> ".join(steps),
                             plan.get("commanded_posture", ""), plan.get("timeout_sec", catalog.get("timeout_sec", 0)),
                             cleanup, detail, ";".join(sorted(references[unit_id]))])
    print(f"{len(references)} actions -> {target}")


if __name__ == "__main__":
    main()
