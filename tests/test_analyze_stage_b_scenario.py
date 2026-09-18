import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from analyze_stage_b_scenario import _axis, classify_hypothesis, static_observability


def scenario(boxes):
    return {
        "start_world_m": [0.0, 0.0, 0.195],
        "goal_lio_m": [12.0, 0.0, 2.0],
        "boxes": boxes,
    }


def test_axis_uses_largest_absolute_component():
    assert _axis([-0.8, 0.1, 0.2]) == "x"
    assert _axis([0.1, -0.9, 0.2]) == "y"
    assert _axis([0.1, 0.2, -0.9]) == "z"


def test_static_observability_flags_visible_cap_and_missing_ceiling():
    result = static_observability(scenario([
        {"name": "floor", "center": [20, 0, -0.1], "size": [80, 16, 0.2]},
        {"name": "start_wall", "center": [-20, 0, 2], "size": [0.2, 4.4, 4]},
    ]), lidar_range_m=40.0, vertical_min_rad=math.radians(-7),
       vertical_max_rad=math.radians(52))
    assert result["longitudinal_cap_visible_from_route"]
    cap = result["longitudinal_caps"][0]
    assert cap["visible_route_interval_m"] == [0.0, 12.0]
    assert cap["covers_route_start"] and cap["covers_route_end"]
    assert not result["ceiling_present"]
    assert result["approximate_floor_intersection_range_m"] > 15.0


def test_static_observability_accepts_distant_caps_and_ceiling():
    result = static_observability(scenario([
        {"name": "floor", "center": [20, 0, -0.1], "size": [160, 16, 0.2]},
        {"name": "ceiling", "center": [20, 0, 4.1], "size": [160, 4.4, 0.2]},
        {"name": "start_wall", "center": [-60, 0, 2], "size": [0.2, 4.4, 4]},
        {"name": "end_wall", "center": [100, 0, 2], "size": [0.2, 4.4, 4]},
    ]), lidar_range_m=40.0, vertical_min_rad=math.radians(-7),
       vertical_max_rad=math.radians(52))
    assert not result["longitudinal_cap_visible_from_route"]
    assert all(cap["visible_route_interval_m"] is None
               for cap in result["longitudinal_caps"])
    assert result["ceiling_present"]


def test_static_observability_reports_entry_only_longitudinal_anchor():
    result = static_observability(scenario([
        {"name": "start_wall", "center": [-35, 0, 2], "size": [0.2, 4.4, 4]},
        {"name": "end_wall", "center": [100, 0, 2], "size": [0.2, 4.4, 4]},
    ]), lidar_range_m=40.0, vertical_min_rad=math.radians(-7),
       vertical_max_rad=math.radians(52))
    cap = next(item for item in result["longitudinal_caps"]
               if item["name"] == "start_wall")
    assert cap["visible_route_interval_m"] == [0.0, 5.0]
    assert cap["covers_route_start"]
    assert not cap["covers_route_end"]


def test_smoke_geometry_is_partial_support_not_mission_contradiction():
    result = classify_hypothesis("unrecoverable", {
        "certifications": 0,
        "certifications_rejected": 0,
        "recovery_forecasts": 0,
    }, "x")
    assert result["overall"] == "PARTIALLY_SUPPORTED"
    assert result["localization_condition"] == "SUPPORTED"
    assert result["mission_condition"] == "NOT_EVALUATED"
    assert result["checks"]["mission_certification_rejected"] is None


def test_mission_evidence_can_contradict_scenario():
    result = classify_hypothesis("unrecoverable", {
        "certifications": 3,
        "certifications_rejected": 0,
        "recovery_forecasts": 0,
    }, "x")
    assert result["overall"] == "CONTRADICTED"
    assert result["mission_condition"] == "CONTRADICTED"
