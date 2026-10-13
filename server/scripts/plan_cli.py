"""Offline planning/preview CLI for the dispensing algorithms (no hardware access).

    python plan_cli.py feedback     ...  preview the production feedback controller
    python plan_cli.py window-pid   ...  preview the window-PID experiment planner
    python plan_cli.py taper        ...  preview the continuous-taper planner
    python plan_cli.py profile      ...  preview/write a draft control profile
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, replace
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from powder_sampling_control.dispensing_algorithm.core import (  # noqa: E402
    ContinuousFeedbackController,
    FeedbackControllerSettings,
    FeedbackInitialParameters,
    FeedbackObservation,
    DISPENSE_ALLOWED_OVERWEIGHT_MG,
    DISPENSE_ALLOWED_UNDERWEIGHT_MG,
    DISPENSE_TARGET_PROFILES,
    build_candidate_control_profile,
    dispense_plan,
)
from powder_sampling_control.dispensing_algorithm.calibration import (  # noqa: E402
    ContinuousTaperConfig,
    ContinuousTaperPlanner,
    WindowPidConfig,
    WindowPidPlanner,
    WindowPidState,
)


# ── feedback (was plan_feedback_dispense.py) ─────────────────────────────────


def cmd_feedback(args: argparse.Namespace) -> int:
    predicted_mass = args.current_mass_mg if args.predicted_mass_mg is None else args.predicted_mass_mg
    predicted_rate = args.rate_mg_s if args.predicted_rate_mg_s is None else args.predicted_rate_mg_s
    target_mg, profile = dispense_plan(args.target_mg)
    settings = FeedbackControllerSettings(
        coarse_rate_mg_s=profile["coarse_rate_mg_s"],
        fine_rate_mg_s=profile["fine_rate_mg_s"],
        precision_rate_mg_s=profile["precision_rate_mg_s"],
        maximum_flow_rate_mg_s=profile["maximum_flow_rate_mg_s"],
        min_window_position_units=100,
        max_window_position_units=750,
        allowed_underweight_mg=DISPENSE_ALLOWED_UNDERWEIGHT_MG,
        allowed_overweight_mg=DISPENSE_ALLOWED_OVERWEIGHT_MG,
        precision_start_remaining_mg=profile["precision_start_remaining_mg"],
        stop_prediction_horizon_s=1.0,
        fixed_tail_mass_mg=2.0,
        stop_target_offset_mg=5.0,
        position_feedback_enabled=False,
        tail_taper_enabled=True,
        tail_taper_start_remaining_mg=profile["tail_taper_start_remaining_mg"],
        tail_taper_end_remaining_mg=profile["tail_taper_end_remaining_mg"],
        tail_duty_reduction_permyriad=200,
        tail_window_reduction_units=50,
    )
    controller = ContinuousFeedbackController(
        target_mass_mg=target_mg,
        initial=FeedbackInitialParameters(
            frequency_hz=args.frequency_hz,
            duty_permyriad=args.duty_permyriad,
            window_position_units=args.window_position_units,
        ),
        settings=settings,
    )
    decision = controller.step(
        FeedbackObservation(
            mass_mg=args.current_mass_mg,
            predicted_mass_mg=predicted_mass,
            rate_mg_s=args.rate_mg_s,
            predicted_rate_mg_s=predicted_rate,
        ),
        timestamp_s=0.0,
    )
    result = asdict(decision)
    result["stage"] = decision.stage.value
    result["target_mg"] = target_mg
    result["coarse_fraction"] = controller.coarse_fraction()
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(f"target={target_mg:.0f} mg, coarse={controller.coarse_threshold_mg():.1f} mg")
        print(f"stage={decision.stage.value}, rate error={decision.rate_error_mg_s:.2f} mg/s")
        print(
            f"frequency={decision.frequency_hz} Hz (locked), "
            f"duty={decision.duty_before_permyriad}->{decision.duty_after_permyriad}, "
            f"window={decision.window_before_units}->{decision.window_after_units}"
        )
        print(f"reason: {decision.reason}")
    return 0


def add_feedback_parser(sub) -> None:
    p = sub.add_parser("feedback", help="preview the generic mass-feedback strategy")
    p.add_argument("target_mg", type=int, choices=tuple(DISPENSE_TARGET_PROFILES))
    p.add_argument("current_mass_mg", type=float)
    p.add_argument("--predicted-mass-mg", type=float, default=None)
    p.add_argument("--rate-mg-s", type=float, default=0.0)
    p.add_argument("--predicted-rate-mg-s", type=float, default=None)
    p.add_argument("--frequency-hz", type=int, required=True)
    p.add_argument("--duty-permyriad", type=int, required=True)
    p.add_argument("--window-position-units", type=int, required=True)
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_feedback)


# ── window-pid (was plan_window_pid_dispense.py) ─────────────────────────────


def _action_to_dict(action):
    if action is None:
        return None
    return asdict(action)


def cmd_window_pid(args: argparse.Namespace) -> int:
    config = replace(WindowPidConfig(), target_mass_mg=args.target_mg)
    planner = WindowPidPlanner(config)
    state = WindowPidState(
        integral_error=args.integral_error,
        previous_error_mg_s=args.previous_error_mg_s,
        no_flow_seconds=args.no_flow_seconds,
    )
    decision = planner.next_decision(
        current_mass_mg=args.current_mass_mg,
        observed_rate_mg_s=args.rate_mg_s,
        state=state,
        dt_s=args.dt_s,
    )
    output = {
        "stage": decision.stage.value.upper(),
        "decision": "FEED" if decision.action is not None else "STOP",
        "action": _action_to_dict(decision.action),
        "current_mass_mg": args.current_mass_mg,
        "target_mass_mg": args.target_mg,
        "remaining_mg": round(decision.remaining_mg, 3),
        "predicted_tail_mg": round(decision.predicted_tail_mg, 3),
        "target_rate_mg_s": round(decision.target_rate_mg_s, 3),
        "observed_rate_mg_s": decision.observed_rate_mg_s,
        "expected_mass_mg": round(decision.expected_mass_mg, 3),
        "stop_mass_mg": round(decision.stop_mass_mg, 3),
        "pid_adjust_units": decision.pid_adjust_units,
        "next_state": asdict(decision.next_state),
        "reason": decision.reason,
    }
    if args.json:
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0

    print(f"当前重量: {args.current_mass_mg:.1f} mg")
    print(f"目标重量: {args.target_mg:.1f} mg")
    if args.rate_mg_s is not None:
        print(f"最近速度: {args.rate_mg_s:.2f} mg/s")
        print(f"预测尾粉: {decision.predicted_tail_mg:.2f} mg")
    print(f"状态: {output['stage']}")
    print(f"决策: {output['decision']}")
    print(f"预测剩余: {decision.remaining_mg:.2f} mg")
    print(f"目标速度: {decision.target_rate_mg_s:.2f} mg/s")
    print(f"本次计划加粉: {decision.expected_mass_mg:.2f} mg")
    print(f"本次动作停止重量: {decision.stop_mass_mg:.2f} mg")
    print(f"PID window 修正: {decision.pid_adjust_units:+d} units")
    print(f"原因: {decision.reason}")
    if decision.action is not None:
        action = decision.action
        print("下一步动作:")
        print(f"  window_position_units = {action.window_position_units}")
        print(f"  frequency_hz = {action.frequency_hz}")
        print(f"  duty_permyriad = {action.duty_permyriad}")
        print(f"  duration_ms = {action.duration_ms}")
        print(f"  duration_s = {action.duration_ms / 1000:.3f}")
    print("下一轮 PID 状态:")
    print(f"  --integral-error {decision.next_state.integral_error:.3f}")
    print(f"  --previous-error-mg-s {decision.next_state.previous_error_mg_s:.3f}")
    print(f"  --no-flow-seconds {decision.next_state.no_flow_seconds:.3f}")
    return 0


def add_window_pid_parser(sub) -> None:
    p = sub.add_parser("window-pid", help="window-position feed action with feedforward plus PID trim")
    p.add_argument("current_mass_mg", type=float, help="latest cumulative mass in mg")
    p.add_argument("--target-mg", type=float, default=500.0, help="target cumulative mass in mg")
    p.add_argument("--rate-mg-s", type=float, default=None, help="recent flow rate in mg/s")
    p.add_argument("--dt-s", type=float, default=1.0, help="time span used by the rate estimate")
    p.add_argument("--integral-error", type=float, default=0.0, help="PID integral state from previous output")
    p.add_argument("--previous-error-mg-s", type=float, default=0.0, help="PID previous error from previous output")
    p.add_argument("--no-flow-seconds", type=float, default=0.0, help="no-flow state from previous output")
    p.add_argument("--json", action="store_true", help="print JSON")
    p.set_defaults(func=cmd_window_pid)


# ── taper (was plan_continuous_taper_dispense.py) ────────────────────────────


def cmd_taper(args: argparse.Namespace) -> int:
    config = ContinuousTaperConfig(target_mass_mg=args.target_mg)
    decision = ContinuousTaperPlanner(config).decide(
        raw_mass_mg=args.raw_mass_mg,
        filtered_mass_mg=args.filtered_mass_mg,
        rate_mg_s=args.rate_mg_s,
        acceleration_mg_s2=args.acceleration_mg_s2,
        predicted_mass_mg=args.predicted_mass_mg,
        current_window_position_units=args.current_window,
        current_frequency_hz=args.current_frequency,
        current_duty_permyriad=args.current_duty,
    )
    result = {
        "stage": decision.stage.value.upper(),
        "command": decision.command,
        "action": asdict(decision.action) if decision.action else None,
        "raw_mass_mg": decision.raw_mass_mg,
        "filtered_mass_mg": round(decision.filtered_mass_mg, 3),
        "predicted_mass_mg": round(decision.predicted_mass_mg, 3),
        "remaining_predicted_mg": round(decision.remaining_predicted_mg, 3),
        "reason": decision.reason,
        "rules": {
            "coarse": "mass < 420 mg -> 80 Hz / window 200 / duty 2000",
            "tail": "420 mg+ -> 65 Hz / window 100 / duty 1600, then hold continuously",
            "stop": "raw >= 498, or raw >= 485 and predicted >= 500, or filtered >= 495",
        },
    }
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0

    print(f"stage: {result['stage']}")
    print(f"command: {result['command']}")
    print(f"raw_mass = {result['raw_mass_mg']:.1f} mg")
    print(f"filtered_mass = {result['filtered_mass_mg']:.1f} mg")
    print(f"predicted_mass = {result['predicted_mass_mg']:.1f} mg")
    print(f"predicted_remaining = {result['remaining_predicted_mg']:.1f} mg")
    print(f"reason: {result['reason']}")
    if result["action"] is not None:
        action = result["action"]
        print("continuous action:")
        print(f"  window_position_units = {action['window_position_units']}")
        print(f"  frequency_hz = {action['frequency_hz']}")
        print(f"  duty_permyriad = {action['duty_permyriad']}")
    return 0


def add_taper_parser(sub) -> None:
    p = sub.add_parser("taper", help="preview the production continuous-taper planner")
    p.add_argument("raw_mass_mg", type=float, help="latest balance mass in mg")
    p.add_argument("--target-mg", type=float, default=500.0)
    p.add_argument("--filtered-mass-mg", type=float, default=None)
    p.add_argument("--rate-mg-s", type=float, default=0.0)
    p.add_argument("--acceleration-mg-s2", type=float, default=0.0)
    p.add_argument("--predicted-mass-mg", type=float, default=None)
    p.add_argument("--current-window", type=int, default=None)
    p.add_argument("--current-frequency", type=int, default=None)
    p.add_argument("--current-duty", type=int, default=None)
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_taper)


# ── profile (was generate_control_profile.py) ────────────────────────────────


def _find_workspace(powder_id: str) -> Path:
    for path in (PROJECT_ROOT / "data" / "powder_library").glob("*.json"):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if data.get("powder_id") == powder_id:
            return path
    raise FileNotFoundError(f"powder workspace not found: {powder_id}")


def cmd_profile(args: argparse.Namespace) -> int:
    workspace_path = _find_workspace(args.powder_id)
    workspace = json.loads(workspace_path.read_text(encoding="utf-8"))
    candidate = build_candidate_control_profile(workspace)
    rendered = json.dumps(candidate, ensure_ascii=False, indent=2) + "\n"

    if not args.write:
        print(rendered, end="")
        return 0

    output = PROJECT_ROOT / "data" / "control_profiles" / f"{args.powder_id}.json"
    if output.exists() and not args.force:
        raise FileExistsError(f"control profile already exists; use --force: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered, encoding="utf-8")
    print(output)
    return 0


def add_profile_parser(sub) -> None:
    p = sub.add_parser("profile", help="preview or write a draft runtime control profile")
    p.add_argument("powder_id")
    p.add_argument(
        "--write",
        action="store_true",
        help="write data/control_profiles/{powder_id}.json instead of previewing",
    )
    p.add_argument(
        "--force",
        action="store_true",
        help="allow replacing an existing draft control profile",
    )
    p.set_defaults(func=cmd_profile)


# ── entry point ──────────────────────────────────────────────────────────────


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)
    add_feedback_parser(sub)
    add_window_pid_parser(sub)
    add_taper_parser(sub)
    add_profile_parser(sub)
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
