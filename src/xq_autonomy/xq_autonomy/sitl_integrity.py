"""ROS-free execution contracts for the P16 SITL integration.

Forecast information is used to rank recovery intents, never to authorize motion.
Authorization always uses the current measured covariance and final EGO spline.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from .alert_limit import compute_alert_limit, sample_bspline
from .integrity_margin import certify_trajectory

STRATEGIES = ("baseline", "conservative", "hard_gate", "recovery")


@dataclass(frozen=True)
class Authorization:
    session: str
    request: int
    trajectory: int
    issued: float
    expires: float
    accepted: bool


class ExecutionGuard:
    """Fail closed on resets, stale inputs, late revocations and ID reuse."""
    def __init__(self, session: str, max_age: float = 0.5):
        if not session or not math.isfinite(max_age) or max_age <= 0:
            raise ValueError("session and positive max_age are required")
        self.session, self.max_age = session, max_age
        self.current: Authorization | None = None
        self.last_time: float | None = None
        self.reset_latched = False
        self.revisions: dict[tuple[int, int], float] = {}
        self.accepted_highwater = (0, 0)

    def clock(self, now: float) -> bool:
        if not math.isfinite(now) or (self.last_time is not None and now < self.last_time - 1e-6):
            self.current = None
            self.revisions.clear()
            self.reset_latched = True
        self.last_time = now
        return not self.reset_latched

    def update(self, auth: Authorization, now: float) -> bool:
        if not self.clock(now) or auth.session != self.session:
            return False
        if not np.isfinite((auth.issued, auth.expires)).all() or auth.request < 1 or auth.trajectory < 1:
            return False
        if not 0 <= now - auth.issued <= self.max_age or auth.expires < auth.issued:
            return False
        key = (auth.request, auth.trajectory)
        previous = self.revisions.get(key, -math.inf)
        # A revoke emitted in the same simulation tick must dominate its lease.
        if auth.issued < previous or (auth.issued == previous and auth.accepted):
            return False
        self.revisions[key] = auth.issued
        if len(self.revisions) > 512:
            self.revisions = dict(sorted(self.revisions.items(), key=lambda item: item[1])[-256:])
        if not auth.accepted:
            if self.current and key == (self.current.request, self.current.trajectory):
                self.current = None
            return True
        if auth.expires <= now:
            return False
        if key < self.accepted_highwater:
            return False
        self.accepted_highwater = max(key, self.accepted_highwater)
        self.current = auth
        return True

    def allows(self, trajectory: int, stamp: float, now: float, frame: str,
               values: np.ndarray, input_age_wall: float) -> bool:
        if not self.clock(now):
            return False
        auth = self.current
        return bool(auth and auth.accepted and trajectory == auth.trajectory
                    and auth.issued <= now < auth.expires
                    and frame == "xq_lio_map" and np.isfinite(values).all()
                    and 0 <= now - stamp <= self.max_age
                    and 0 <= input_age_wall <= self.max_age)


def derivative_bounds(points, knots, degree):
    """Convex-hull bounds of spline velocity and acceleration (not finite differences)."""
    p, k = np.asarray(points, float), np.asarray(knots, float)
    sample_bspline(p, k, degree, 0.1)  # validate domain, shape and finiteness
    bounds = []
    for order in range(2):
        d = degree - order
        if d <= 0:
            raise ValueError("executable spline must be at least quadratic")
        denominator = k[d+1:d+len(p)] - k[1:len(p)]
        if np.any(denominator <= 0):
            raise ValueError("degenerate derivative knots")
        p = d * np.diff(p, axis=0) / denominator[:, None]
        k = k[1:-1]
        bounds.append(float(np.linalg.norm(p, axis=1).max()))
    return tuple(bounds)


@dataclass(frozen=True)
class Certification:
    accepted: bool
    reason: str
    alert: float
    protection: float
    margin: float
    speed: float
    acceleration: float
    direction: tuple[float, float, float] = (0., 0., 0.)


def certify_final(points, knots, degree, obstacles, covariance, *, strategy,
                  k_alpha, elapsed=0.0, input_age=0.0, tracking_error=0.0,
                  speed_limit=0.65, acceleration_limit=1.0, reserve=0.10,
                  body_radius=0.35, latency=0.15, braking_acceleration=0.7):
    if strategy not in STRATEGIES:
        raise ValueError("unknown strategy")
    if not np.isfinite((elapsed, input_age, tracking_error)).all() or not 0 <= input_age <= 0.5:
        raise ValueError("stale or invalid certification inputs")
    if tracking_error < 0 or tracking_error > 0.35:
        raise ValueError("tracking envelope exceeded")
    speed, acceleration = derivative_bounds(points, knots, degree)
    if speed > speed_limit * 1.05 or acceleration > acceleration_limit * 1.05:
        return Certification(False, "DYNAMICS", 0., 0., 0., speed, acceleration)
    # Bound inter-sample displacement and emergency stopping on every final spline.
    sample_interval = min(0.05, 0.025 / max(speed, 0.01))
    samples = sample_bspline(points, knots, degree, sample_interval,
                             minimum_parameter_s=float(knots[degree]) + max(0., elapsed))
    stopping = speed * speed / (2 * braking_acceleration)
    alert = compute_alert_limit(samples, obstacles, speed_mps=speed,
        latency_p99_s=latency + input_age, maximum_acceleration_mps2=acceleration_limit,
        body_radius_m=body_radius, base_reserve_m=0.10 + speed * sample_interval,
        tracking_reserve_m=max(0.10, tracking_error), dynamic_reserve_m=stopping)
    result = certify_trajectory(alert.alert_limits, alert.obstacle_directions, covariance,
                                k_alpha=k_alpha, margin_reserve=reserve)
    i = result.critical_index
    collision_ok = bool(np.min(alert.alert_limits) >= 0)
    accepted = collision_ok and (strategy in ("baseline", "conservative") or result.accepted)
    return Certification(accepted, "ACCEPT" if accepted else "MARGIN" if collision_ok else "CLEARANCE",
        float(alert.alert_limits[i]), float(result.protection_levels[i]), result.minimum_margin,
        speed, acceleration, tuple(float(v) for v in alert.obstacle_directions[i]))


def brake_samples(position, velocity, elapsed, deceleration=0.7):
    """A bounded deceleration target, then a persistent hold target."""
    p, v = np.asarray(position, float), np.asarray(velocity, float)
    if p.shape != (3,) or v.shape != (3,) or not np.isfinite((p, v)).all():
        raise ValueError("invalid measured brake state")
    if not math.isfinite(elapsed) or elapsed < 0 or deceleration <= 0:
        raise ValueError("invalid brake timing")
    speed = float(np.linalg.norm(v))
    if speed < 1e-9:
        return p.copy()
    t = min(elapsed, speed / deceleration)
    return p + v * t - 0.5 * deceleration * t*t * v / speed


class RecoveryCycle:
    """Correlates planning requests; an observed update is mandatory after movement."""
    def __init__(self):
        self.request = 0
        self.intent = "mission"
        self.phase = "MISSION"
        self.observation_after = -math.inf
        self.remaining: list[tuple[str, np.ndarray, float]] = []

    def issue(self, intent):
        self.request += 1
        self.intent = intent
        self.phase = "PLANNING"
        return self.request

    def result(self, request, accepted):
        if request != self.request or self.phase not in ("PLANNING", "EXECUTING"):
            return False
        self.phase = "EXECUTING" if accepted else "WAITING"
        return True

    def arrived(self, now):
        self.phase = "OBSERVING"
        self.observation_after = now

    def observed(self, stamp):
        return self.phase == "OBSERVING" and stamp > self.observation_after
