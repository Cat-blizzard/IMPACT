import math
import numpy as np
import pytest
from xq_autonomy.sitl_integrity import (Authorization, ExecutionGuard, RecoveryCycle,
    brake_samples, certify_final, derivative_bounds)


def authorization(**values):
    data=dict(session="run-a",request=1,trajectory=1,issued=10.,expires=10.3,accepted=True)
    data.update(values)
    return Authorization(**data)


def test_requires_matching_session_trajectory_freshness_and_frame():
    g=ExecutionGuard("run-a")
    assert not g.update(authorization(session="old"),10.)
    assert g.update(authorization(),10.)
    assert g.allows(1,10.,10.1,"xq_lio_map",np.zeros(4),.1)
    assert not g.allows(2,10.,10.1,"xq_lio_map",np.zeros(4),.1)
    assert not g.allows(1,10.,10.1,"world",np.zeros(4),.1)
    assert not g.allows(1,10.,10.1,"xq_lio_map",np.array([math.nan]),.1)
    assert not g.allows(1,10.,10.1,"xq_lio_map",np.zeros(4),.6)
    assert not g.allows(1,10.,10.31,"xq_lio_map",np.zeros(4),.1)


def test_revocation_is_immediate_and_late_accept_cannot_resurrect():
    g=ExecutionGuard("run-a")
    assert g.update(authorization(),10.)
    assert g.update(authorization(issued=10.1,expires=10.1,accepted=False),10.1)
    assert g.current is None
    assert not g.update(authorization(),10.1)
    assert not g.allows(1,10.1,10.1,"xq_lio_map",np.zeros(3),.01)


def test_rejected_replacement_keeps_still_valid_old_trajectory():
    g=ExecutionGuard("run-a")
    g.update(authorization(),10.)
    g.update(authorization(request=2,trajectory=2,issued=10.1,expires=10.1,accepted=False),10.1)
    assert g.current.trajectory == 1
    g.update(authorization(request=2,trajectory=3,issued=10.2,expires=10.5),10.2)
    g.update(authorization(issued=10.21,expires=10.21,accepted=False),10.21)
    assert g.current.trajectory == 3


def test_same_tick_revoke_dominates_lease_and_old_track_cannot_return():
    g=ExecutionGuard("run-a")
    assert g.update(authorization(trajectory=3),10.)
    assert g.update(authorization(trajectory=3,accepted=False,expires=10.),10.)
    assert g.current is None
    assert not g.update(authorization(trajectory=3),10.)
    assert not g.update(authorization(trajectory=2,issued=10.1,expires=10.4),10.1)


def test_clock_reset_latches_until_new_run_and_clears_all_state():
    g=ExecutionGuard("run-a")
    g.update(authorization(),10.)
    assert not g.clock(0.)
    assert g.current is None and not g.revisions
    assert not g.update(authorization(issued=0.,expires=.3),0.)
    assert not g.update(authorization(issued=10.,expires=10.3),10.)


def test_brake_target_accounts_for_stopping_distance_and_holds_endpoint():
    p=np.array([1.,2.,2.]); v=np.array([.7,0,0])
    assert np.allclose(brake_samples(p,v,0),p)
    assert np.allclose(brake_samples(p,v,1),[1.35,2,2])
    assert np.allclose(brake_samples(p,v,20),[1.35,2,2])
    assert brake_samples(p,v,.5)[0] < brake_samples(p,v,1)[0]
    with pytest.raises(ValueError): brake_samples(p,v,-1)


def spline():
    return np.column_stack((np.linspace(0,.3,4),np.zeros(4),np.full(4,2.))),np.array([0.]*4+[1.]*4)


def test_final_spline_has_hard_dynamics_check_even_without_integrity():
    p,k=spline()
    speed,acc=derivative_bounds(p,k,3)
    assert speed == pytest.approx(.3) and acc == pytest.approx(0)
    result=certify_final(p*10,k,3,np.array([[0,4,2.]]),np.eye(3)*.01,
                         strategy="baseline",k_alpha=2.)
    assert not result.accepted and result.reason == "DYNAMICS"


def test_current_covariance_not_forecast_controls_authorization():
    p,k=spline(); obstacle=np.array([[.15,1.,2.]])
    safe=certify_final(p,k,3,obstacle,np.eye(3)*1e-5,strategy="recovery",k_alpha=2.)
    unsafe=certify_final(p,k,3,obstacle,np.eye(3),strategy="recovery",k_alpha=2.)
    baseline=certify_final(p,k,3,obstacle,np.eye(3),strategy="baseline",k_alpha=2.)
    assert safe.accepted and not unsafe.accepted and baseline.accepted
    assert unsafe.alert-unsafe.protection == pytest.approx(unsafe.margin)
    with pytest.raises(ValueError):
        certify_final(p,k,3,obstacle,np.eye(3),strategy="baseline",k_alpha=2.,input_age=.6)


def test_recovery_requires_correlated_request_and_new_observation():
    cycle=RecoveryCycle()
    first=cycle.issue("left")
    second=cycle.issue("right")
    assert not cycle.result(first,True)
    assert cycle.result(second,False) and cycle.phase == "WAITING"
    third=cycle.issue("up")
    assert cycle.result(third,True)
    cycle.arrived(12.)
    assert not cycle.observed(11.9)
    assert cycle.observed(12.1)
