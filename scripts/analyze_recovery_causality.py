#!/usr/bin/env python3
"""Link recovery forecasts to measured execution evidence without replaying flight."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import Counter
from pathlib import Path


def _jsonl(path: Path) -> list[dict]:
    rows = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"{path}:{number}: expected a JSON object")
        rows.append(value)
    return rows


def _distance(left, right, dimensions=3) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left[:dimensions], right[:dimensions])))


def _observation_intent(telemetry: list[dict], sim_time: float) -> str | None:
    # The supervisor requests the mission again in the same tick as NEW_OBSERVATION,
    # so the nearest sample is commonly already labelled mission.  Use the dominant
    # state in the observation interval immediately preceding the event.
    intents = [str(row["intent"]) for row in telemetry
               if isinstance(row.get("sim_time"), (int, float)) and row.get("intent")
               and sim_time - 1.25 <= float(row["sim_time"]) < sim_time]
    if not intents:
        return None
    counts = Counter(intents)
    return max(counts, key=lambda value: (counts[value], intents[::-1].index(value) * -1))


def analyze(events: list[dict], telemetry: list[dict], *, reserve_m: float,
            estimator_memory_horizon_s: float) -> dict:
    if not math.isfinite(reserve_m) or reserve_m < 0.0:
        raise ValueError("reserve_m must be finite and nonnegative")
    if not math.isfinite(estimator_memory_horizon_s) or estimator_memory_horizon_s < 0.0:
        raise ValueError("estimator memory horizon must be finite and nonnegative")

    ordered = sorted(events, key=lambda row: (float(row.get("sim_time", math.inf)),
                                               float(row.get("wall_time", math.inf))))
    plans = {int(row["request_id"]): row for row in ordered
             if row.get("event") == "PLAN_REQUEST" and "request_id" in row}
    forecasts = [row for row in ordered if row.get("event") == "RECOVERY_FORECAST"]
    cycles = []
    prediction_gaps = []
    predicted_feasible_actual_rejects = 0

    for index, forecast in enumerate(forecasts):
        start = float(forecast["sim_time"])
        stop = (float(forecasts[index + 1]["sim_time"])
                if index + 1 < len(forecasts) else math.inf)
        window = [row for row in ordered if start <= float(row.get("sim_time", -math.inf)) < stop]
        predicted = {str(row["name"]): float(row["predicted_margin"])
                     for row in forecast.get("candidates", [])}
        certifications = [row for row in window if row.get("event") == "CERTIFY"]
        recovery_certifications = []
        mission_certifications = []
        for certification in certifications:
            plan = plans.get(int(certification.get("request_id", -1)), {})
            intent = str(plan.get("intent", "unknown"))
            item = {
                "request_id": certification.get("request_id"),
                "intent": intent,
                "accepted": bool(certification.get("accepted")),
                "actual_margin_m": certification.get("margin"),
            }
            if intent == "mission":
                mission_certifications.append(item)
            else:
                if intent in predicted and isinstance(certification.get("margin"), (int, float)):
                    item["predicted_margin_m"] = predicted[intent]
                    item["prediction_minus_actual_m"] = predicted[intent] - float(certification["margin"])
                    prediction_gaps.append(item["prediction_minus_actual_m"])
                    if predicted[intent] >= reserve_m and float(certification["margin"]) < reserve_m:
                        predicted_feasible_actual_rejects += 1
                recovery_certifications.append(item)

        observations = [row for row in window if row.get("event") == "NEW_OBSERVATION"]
        step_done = [row for row in window if row.get("event") == "RECOVERY_STEP_DONE"]
        confirmed = [row for row in window if row.get("event") == "RECOVERY_CONFIRMED"]
        observation_intent = (_observation_intent(telemetry, float(observations[0]["sim_time"]))
                              if observations else None)
        cycle_telemetry = [row for row in telemetry
                           if start <= float(row.get("sim_time", -math.inf)) < stop and row.get("truth")]
        maximum_displacement = None
        maximum_xy_displacement = None
        if cycle_telemetry:
            origin = cycle_telemetry[0]["truth"]
            maximum_displacement = max(_distance(origin, row["truth"]) for row in cycle_telemetry)
            maximum_xy_displacement = max(_distance(origin, row["truth"], 2) for row in cycle_telemetry)
        cycles.append({
            "index": index + 1,
            "forecast_sim_time": start,
            "mission_request_id": forecast.get("request_id"),
            "predicted_candidates": forecast.get("candidates", []),
            "recovery_certifications": recovery_certifications,
            "mission_certifications": mission_certifications,
            "accepted_recovery_certifications": sum(row["accepted"] for row in recovery_certifications),
            "recovery_step_done_events": len(step_done),
            "new_observation_events": len(observations),
            "recovery_confirmed_events": len(confirmed),
            "observation_intent": observation_intent,
            "maximum_truth_displacement_m": maximum_displacement,
            "maximum_truth_xy_displacement_m": maximum_xy_displacement,
        })

    all_truth = [row["truth"] for row in telemetry if row.get("truth")]
    total_displacement = _distance(all_truth[0], all_truth[-1]) if all_truth else None
    maximum_displacement = (max(_distance(all_truth[0], point) for point in all_truth)
                            if all_truth else None)
    observations = [row for row in ordered if row.get("event") == "NEW_OBSERVATION"]
    step_done = [row for row in ordered if row.get("event") == "RECOVERY_STEP_DONE"]
    confirmed = [row for row in ordered if row.get("event") == "RECOVERY_CONFIRMED"]
    beneficial_confirmations = [
        row for row in confirmed
        if isinstance(row.get("delta_margin"), (int, float))
        and math.isfinite(float(row["delta_margin"]))
        and float(row["delta_margin"]) > 0.0
    ]
    short_hover_observations = sum(
        cycle["new_observation_events"] > 0 and cycle["observation_intent"] == "short_hover"
        for cycle in cycles
    )
    observed_cycles = sum(cycle["new_observation_events"] > 0 for cycle in cycles)
    mission_retries_accepted = sum(
        row["accepted"] for cycle in cycles for row in cycle["mission_certifications"]
    )
    accepted_recovery = sum(cycle["accepted_recovery_certifications"] for cycle in cycles)
    gap_summary = {
        "samples": len(prediction_gaps),
        "mean_m": statistics.fmean(prediction_gaps) if prediction_gaps else None,
        "minimum_m": min(prediction_gaps) if prediction_gaps else None,
        "maximum_m": max(prediction_gaps) if prediction_gaps else None,
    }
    observations_summary = {
        "forecast_cycles": len(cycles),
        "forecast_candidates": sum(len(cycle["predicted_candidates"]) for cycle in cycles),
        "recovery_certifications": sum(len(cycle["recovery_certifications"]) for cycle in cycles),
        "accepted_recovery_certifications": accepted_recovery,
        "predicted_feasible_actual_rejects": predicted_feasible_actual_rejects,
        "new_observation_events": len(observations),
        "short_hover_observation_cycles": short_hover_observations,
        "recovery_step_done_events": len(step_done),
        "recovery_confirmed_events": len(confirmed),
        "positive_margin_recovery_confirmations": len(beneficial_confirmations),
        "accepted_mission_retries": mission_retries_accepted,
        "prediction_minus_actual_margin": gap_summary,
        "total_truth_displacement_m": total_displacement,
        "maximum_truth_displacement_from_start_m": maximum_displacement,
    }
    findings = {
        "forecast_and_online_certification_disagree": (
            predicted_feasible_actual_rejects > 0 and gap_summary["mean_m"] is not None
            and gap_summary["mean_m"] > 0.0
        ),
        "new_observation_without_completed_recovery_step": bool(observations and not step_done),
        "short_hover_closes_recovery_cycles": bool(
            observed_cycles and short_hover_observations == observed_cycles
        ),
        "measured_recovery_never_confirmed": not confirmed,
        "mission_retry_never_authorized": mission_retries_accepted == 0,
    }
    mechanism_checks = {
        "online_recovery_triggered": bool(cycles),
        "recovery_trajectory_authorized": accepted_recovery > 0,
        "recovery_step_completed": bool(step_done),
        "post_step_observation_recorded": any(
            cycle["recovery_step_done_events"] > 0 and cycle["new_observation_events"] > 0
            for cycle in cycles
        ),
        "mission_reauthorized_after_observation": mission_retries_accepted > 0,
        "measured_margin_improved": bool(beneficial_confirmations),
    }
    hypotheses = [
        {
            "name": "forecast_estimator_model_contract_mismatch",
            "supported_by": [
                "forecast_and_online_certification_disagree",
                "measured_recovery_never_confirmed",
            ],
            "not_proven": "Map-derived forecasts and final EGO splines use different paths and critical directions.",
        },
        {
            "name": "short_hover_treats_fresh_timestamp_as_recovery_observation",
            "supported_by": [
                "new_observation_without_completed_recovery_step",
                "short_hover_closes_recovery_cycles",
            ],
            "not_proven": "A fresh timestamp can contain real information; benefit must be checked separately.",
        },
    ]
    return {
        "schema_version": 1,
        "gate": "RECOVERY_CAUSAL_AUDIT",
        "status": "DIAGNOSTIC_COMPLETE",
        "mechanism_status": "PASS" if all(mechanism_checks.values()) else "NOT_DEMONSTRATED",
        "mechanism_checks": mechanism_checks,
        "configuration": {
            "margin_reserve_m": reserve_m,
            "estimator_information_memory_horizon_s": estimator_memory_horizon_s,
        },
        "observations": observations_summary,
        "findings": findings,
        "hypotheses": hypotheses,
        "cycles": cycles,
        "limitations": [
            "This audit links recorded events and truth-only evaluation telemetry; it does not alter flight evidence.",
            "Forecast and final certification margins may use different trajectories and critical directions.",
            "A recovery hypothesis is not success without RECOVERY_CONFIRMED and a subsequently authorized mission trajectory.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--reserve-m", type=float, default=0.10)
    parser.add_argument("--estimator-memory-horizon-s", type=float, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--require-mechanism-pass", action="store_true")
    args = parser.parse_args()
    output = args.output or args.run_dir / "recovery-causal-audit.json"
    report = analyze(_jsonl(args.run_dir / "events.jsonl"),
                     _jsonl(args.run_dir / "telemetry.jsonl"),
                     reserve_m=args.reserve_m,
                     estimator_memory_horizon_s=args.estimator_memory_horizon_s)
    report["run"] = str(args.run_dir.resolve())
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"],
                      "mechanism_status": report["mechanism_status"],
                      "mechanism_checks": report["mechanism_checks"],
                      "output": str(output), "findings": report["findings"]}, indent=2))
    return int(args.require_mechanism_pass and report["mechanism_status"] != "PASS")


if __name__ == "__main__":
    raise SystemExit(main())
