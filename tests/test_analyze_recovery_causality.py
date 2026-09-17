from analyze_recovery_causality import analyze


def event(kind, time, request, **values):
    return {"event": kind, "sim_time": time, "wall_time": time,
            "request_id": request, **values}


def test_audit_links_forecast_rejection_hover_and_failed_mission_retry():
    events = [
        event("PLAN_REQUEST", 1.0, 1, intent="mission"),
        event("CERTIFY", 1.1, 1, accepted=False, margin=-0.3),
        event("RECOVERY_FORECAST", 1.2, 1,
              candidates=[{"name": "up_offset", "predicted_margin": 0.8}]),
        event("PLAN_REQUEST", 1.3, 2, intent="up_offset"),
        event("CERTIFY", 1.4, 2, accepted=False, margin=-0.2),
        event("NEW_OBSERVATION", 2.0, 2, observed_PL=0.1),
        event("PLAN_REQUEST", 2.1, 3, intent="mission"),
        event("CERTIFY", 2.2, 3, accepted=False, margin=-0.1),
    ]
    telemetry = [
        {"sim_time": 1.2, "truth": [0.0, 0.0, 2.0], "intent": "up_offset"},
        {"sim_time": 1.7, "truth": [0.01, 0.0, 2.0], "intent": "short_hover"},
        {"sim_time": 2.0, "truth": [0.01, 0.0, 2.0], "intent": "mission"},
    ]
    report = analyze(events, telemetry, reserve_m=0.1, estimator_memory_horizon_s=0.0)
    assert report["observations"]["predicted_feasible_actual_rejects"] == 1
    assert report["observations"]["accepted_mission_retries"] == 0
    assert report["findings"]["forecast_and_online_certification_disagree"]
    assert report["findings"]["new_observation_without_completed_recovery_step"]
    assert report["findings"]["short_hover_closes_recovery_cycles"]
    assert report["cycles"][0]["observation_intent"] == "short_hover"


def test_audit_does_not_call_confirmed_recovery_a_failure():
    events = [
        event("RECOVERY_FORECAST", 1.0, 1,
              candidates=[{"name": "up_offset", "predicted_margin": 0.3}]),
        event("PLAN_REQUEST", 1.1, 2, intent="up_offset"),
        event("CERTIFY", 1.2, 2, accepted=True, margin=0.2),
        event("RECOVERY_STEP_DONE", 2.0, 2),
        event("NEW_OBSERVATION", 2.1, 2),
        event("RECOVERY_CONFIRMED", 2.3, 3),
        event("PLAN_REQUEST", 2.2, 3, intent="mission"),
        event("CERTIFY", 2.3, 3, accepted=True, margin=0.15),
    ]
    telemetry = [{"sim_time": 1.0, "truth": [0, 0, 0], "intent": "up_offset"},
                 {"sim_time": 2.1, "truth": [0, 0.4, 0], "intent": "up_offset"}]
    report = analyze(events, telemetry, reserve_m=0.1, estimator_memory_horizon_s=3.0)
    assert not report["findings"]["measured_recovery_never_confirmed"]
    assert not report["findings"]["mission_retry_never_authorized"]
    assert not report["findings"]["new_observation_without_completed_recovery_step"]
