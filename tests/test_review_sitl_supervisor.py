"""Regression checks for the live execution fail-closed gate."""

from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("rclpy")

from xq_autonomy.sitl_supervisor_node import (  # noqa: E402
    EXECUTION_MIN_PROGRESS_M,
    EXECUTION_PROGRESS_TIMEOUT_S,
    SITLSupervisor,
)


def _supervisor():
    node = object.__new__(SITLSupervisor)
    node.execution_fault = None
    node.active = object()
    node.cycle = SimpleNamespace(intent="mission", phase="EXECUTING")
    node.goal = np.array([12.0, 0.0, 2.0])
    node.goal_tolerance = 0.45
    node.integrity = SimpleNamespace(weak_direction_map=[1.0, 0.0, 0.0])
    node.progress_anchor_sim = None
    node.progress_anchor_distance = None
    node.progress_anchor_position = None
    node.revocations = []
    node.events = []
    node.revoke = lambda reason: node.revocations.append(reason)
    node.event = lambda kind, **fields: node.events.append((kind, fields))
    return node


def test_execution_stall_fail_closes_with_directional_evidence():
    node = _supervisor()
    position = np.array([6.0, 0.0, 2.0])
    node._check_execution_progress(100.0, position)
    node._check_execution_progress(100.0 + EXECUTION_PROGRESS_TIMEOUT_S + 0.1, position)

    assert node.execution_fault["reason"] == "EXECUTION_STALLED"
    assert node.execution_fault["weak_axis_aligned"] is True
    assert node.revocations == ["EXECUTION_STALLED"]
    assert node.events[-1][0] == "EXECUTION_FAIL_CLOSED"


def test_measurable_progress_restarts_stall_window():
    node = _supervisor()
    node._check_execution_progress(10.0, np.array([0.0, 0.0, 2.0]))
    node._check_execution_progress(
        10.0 + EXECUTION_PROGRESS_TIMEOUT_S - 0.1,
        np.array([EXECUTION_MIN_PROGRESS_M + 0.1, 0.0, 2.0]),
    )
    node._check_execution_progress(
        10.0 + EXECUTION_PROGRESS_TIMEOUT_S + 0.1,
        np.array([EXECUTION_MIN_PROGRESS_M + 0.1, 0.0, 2.0]),
    )

    assert node.execution_fault is None
    assert node.revocations == []

