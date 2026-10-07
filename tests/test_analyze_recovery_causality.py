from analyze_recovery_causality import analyze
from copy import deepcopy


def event(kind, time, request, **values):
    return {"event": kind, "sim_time": time, "wall_time": time,
            "request_id": request, **values}


def information_evidence():
    return dict(information_improved=True,
                information_before=dict(direction=[1.0, 0.0, 0.0], stamp_s=1.0,
                                        variance_m2=1e-4, geometric_information=20.0),
                information_after=dict(direction=[1.0, 0.0, 0.0], stamp_s=2.05,
                                       variance_m2=8e-5, geometric_information=30.0))


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
    assert report["mechanism_status"] == "NOT_DEMONSTRATED"
    assert not report["mechanism_checks"]["recovery_trajectory_authorized"]


def test_audit_does_not_call_confirmed_recovery_a_failure():
    events = [
        event("RECOVERY_FORECAST", 1.0, 1,
              candidates=[{"name": "up_offset", "predicted_margin": 0.3}]),
        event("PLAN_REQUEST", 1.1, 2, intent="up_offset"),
        event("CERTIFY", 1.2, 2, accepted=True, margin=0.2),
        event("RECOVERY_STEP_DONE", 2.0, 2),
        event("NEW_OBSERVATION", 2.1, 2, **information_evidence()),
        event("PLAN_REQUEST", 2.2, 3, intent="mission"),
        event("CERTIFY", 2.3, 3, accepted=True, margin=0.15),
        event("RECOVERY_CONFIRMED", 2.3, 3, delta_margin=0.05,
              observation_request_id=2, **information_evidence()),
    ]
    telemetry = [{"sim_time": 1.0, "truth": [0, 0, 0], "intent": "up_offset"},
                 {"sim_time": 2.1, "truth": [0, 0.4, 0], "intent": "up_offset"}]
    report = analyze(events, telemetry, reserve_m=0.1, estimator_memory_horizon_s=3.0)
    assert not report["findings"]["measured_recovery_never_confirmed"]
    assert not report["findings"]["mission_retry_never_authorized"]
    assert not report["findings"]["new_observation_without_completed_recovery_step"]
    assert report["mechanism_status"] == "PASS"
    assert all(report["mechanism_checks"].values())


def test_audit_rejects_confirmation_without_measured_benefit():
    events = [
        event("RECOVERY_FORECAST", 1.0, 1,
              candidates=[{"name": "up_offset", "predicted_margin": 0.3}]),
        event("PLAN_REQUEST", 1.1, 2, intent="up_offset"),
        event("CERTIFY", 1.2, 2, accepted=True, margin=0.2),
        event("RECOVERY_STEP_DONE", 2.0, 2),
        event("NEW_OBSERVATION", 2.1, 2),
        event("PLAN_REQUEST", 2.2, 3, intent="mission"),
        event("CERTIFY", 2.3, 3, accepted=True, margin=0.15),
        event("RECOVERY_CONFIRMED", 2.3, 3, delta_margin=-0.05),
    ]
    telemetry = [{"sim_time": 1.0, "truth": [0, 0, 0], "intent": "up_offset"},
                 {"sim_time": 2.1, "truth": [0, 0.4, 0], "intent": "up_offset"}]
    report = analyze(events, telemetry, reserve_m=0.1, estimator_memory_horizon_s=3.0)
    assert report["mechanism_status"] == "NOT_DEMONSTRATED"
    assert not report["mechanism_checks"]["measured_margin_improved"]


def test_missing_information_confirmation_does_not_pass():
    report = analyze([event("RECOVERY_CONFIRMED", 2.3, 3, delta_margin=0.05)], [],
                     reserve_m=0.1, estimator_memory_horizon_s=3.0)
    assert not report["mechanism_checks"]["measured_margin_improved"]
    assert report["mechanism_status"] == "NOT_DEMONSTRATED"


def test_information_chain_requires_same_request_session_and_post_step_stamp():
    events = [
        event("RECOVERY_FORECAST", 1.0, 1, candidates=[]),
        event("PLAN_REQUEST", 1.1, 2, intent="up_offset"),
        event("CERTIFY", 1.2, 2, accepted=True, margin=0.2),
        event("RECOVERY_STEP_DONE", 2.0, 2),
        event("NEW_OBSERVATION", 2.1, 2, **information_evidence()),
        event("PLAN_REQUEST", 2.2, 3, intent="mission"),
        event("CERTIFY", 2.3, 3, accepted=True, margin=0.15),
        event("RECOVERY_CONFIRMED", 2.3, 3, delta_margin=0.05,
              observation_request_id=2, **information_evidence()),
    ]
    assert analyze(events, [], reserve_m=0.1, estimator_memory_horizon_s=3.0)["mechanism_status"] == "PASS"
    for mutation in ("session", "request", "stamp", "information", "variance", "direction", "cross_cycle"):
        broken = deepcopy(events)
        if mutation == "session":
            broken[2]["session_id"] = "other"
        elif mutation == "request":
            broken[3]["request_id"] = 99
        elif mutation == "cross_cycle":
            broken.insert(4, event("RECOVERY_FORECAST", 2.05, 2, candidates=[]))
        else:
            after = broken[4]["information_after"]
            if mutation == "stamp":
                after["stamp_s"] = 1.9
            elif mutation == "information":
                after["geometric_information"] = 15.0
            elif mutation == "variance":
                after["variance_m2"] = 1e-4 - 5e-7
            elif mutation == "direction":
                after["direction"] = [0.0, 1.0, 0.0]
        report = analyze(broken, [], reserve_m=0.1, estimator_memory_horizon_s=3.0)
        assert report["mechanism_status"] == "NOT_DEMONSTRATED", mutation
