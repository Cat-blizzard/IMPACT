from analyze_external_nav_health import classify_status_text


def test_lane_switch_is_visible_without_becoming_a_blocking_fault():
    fault, observation = classify_status_text("EKF3 lane switch 1")
    assert fault is None
    assert observation == "ekf3 lane switch"


def test_primary_change_is_visible_without_becoming_a_blocking_fault():
    fault, observation = classify_status_text("EKF primary changed:1")
    assert fault is None
    assert observation == "ekf primary changed"


def test_ekf_failsafe_remains_a_blocking_fault():
    fault, observation = classify_status_text("EKF Failsafe: changed to LAND Mode")
    assert fault == "ekf failsafe"
    assert observation is None
