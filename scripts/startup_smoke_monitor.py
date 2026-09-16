#!/usr/bin/env python3
"""Observe startup topics for a bounded window without arming or commanding flight."""
import argparse, json, time
import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import EstimatorStatus, State, StatusText, SysStatus
from mavros_msgs.srv import CommandLong, StreamRate
from std_msgs.msg import String

PREARM_CHECK = 1 << 28
VISION_POSITION = 1 << 7


def stamp_s(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def sample_metric(values, start, end, header=False, age=False):
    times = [x[0] for x in values]
    gaps = [b - a for a, b in zip(times, times[1:])]
    result = {
        'count': len(times),
        'rate_hz': (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0,
        'first_delay_s': times[0] - start if times else None,
        'last_silence_s': end - times[-1] if times else None,
        'max_receive_gap_s': max(gaps, default=None),
        'observation_span_s': times[-1] - times[0] if len(times) > 1 else 0.0,
    }
    if header:
        stamps = [stamp_s(x[2].header.stamp) for x in values]
        result.update(
            first_stamp_s=stamps[0] if stamps else None,
            last_stamp_s=stamps[-1] if stamps else None,
            max_stamp_gap_s=max((b - a for a, b in zip(stamps, stamps[1:])), default=None),
            nonincreasing_stamps=sum(b <= a for a, b in zip(stamps, stamps[1:])),
        )
        if age:
            ages = [sim - stamp for (_, sim, _), stamp in zip(values, stamps) if sim is not None]
            result.update(min_data_age_s=min(ages, default=None), max_data_age_s=max(ages, default=None))
    return result


def service_event(kind, future, **metadata):
    event = {'kind': kind, **metadata, 'completed': bool(future and future.done()),
             'success': False}
    if not event['completed']:
        return event
    try:
        response = future.result()
    except Exception as exc:
        event['error'] = f'{type(exc).__name__}: {exc}'
        return event
    if response is None:
        event['error'] = 'service completed without a response'
        return event
    # StreamRate has an empty response; completion without an exception is its acknowledgement.
    event['success'] = bool(getattr(response, 'success', True))
    if hasattr(response, 'result'):
        event['result'] = int(response.result)
    return event


class Monitor(Node):
    def __init__(self):
        super().__init__('impact_startup_smoke_monitor')
        self.samples = {'odom': [], 'extnav': [], 'extnav_output': [], 'fcu': [],
                        'sys_status': [], 'estimator': [], 'statustext': [], 'fcu_imu': []}
        self.sim_time = None
        self.create_subscription(Clock, '/clock', self._clock, 100)
        self.create_subscription(Odometry, '/localization/odom', lambda m: self._record('odom', m), 20)
        self.create_subscription(String, '/xq/p4/extnav/status', lambda m: self._record('extnav', m), 20)
        self.create_subscription(Odometry, '/uav1/mavros/odometry/out', lambda m: self._record('extnav_output', m), 20)
        self.create_subscription(State, '/uav1/mavros/state', lambda m: self._record('fcu', m), 20)
        best_effort = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(SysStatus, '/uav1/mavros/sys_status', lambda m: self._record('sys_status', m), best_effort)
        self.create_subscription(EstimatorStatus, '/uav1/mavros/estimator_status', lambda m: self._record('estimator', m), best_effort)
        self.create_subscription(StatusText, '/uav1/mavros/statustext/recv', lambda m: self._record('statustext', m), best_effort)
        self.create_subscription(Imu, '/uav1/mavros/imu/data', lambda m: self._record('fcu_imu', m), best_effort)
        self.stream_client = self.create_client(StreamRate, '/uav1/mavros/set_stream_rate')
        self.prearm_client = self.create_client(CommandLong, '/uav1/mavros/cmd/command')

    def _clock(self, message):
        self.sim_time = stamp_s(message.clock)

    def _record(self, key, message):
        self.samples[key].append((time.monotonic(), self.sim_time, message))

    def request_streams(self):
        if not self.stream_client.wait_for_service(timeout_sec=5.0):
            return None
        request = StreamRate.Request(stream_id=StreamRate.Request.STREAM_ALL,
                                     message_rate=20, on_off=True)
        return self.stream_client.call_async(request)

    def request_prearm_check(self):
        if not self.prearm_client.service_is_ready():
            return None
        return self.prearm_client.call_async(CommandLong.Request(command=401))

def stream_continuous(metric, maximum_gap_s):
    """Cover both observation boundaries, as well as gaps between samples."""
    return metric['count'] > 1 and all(
        isinstance(metric[name], (int, float)) and 0.0 <= metric[name] <= maximum_gap_s
        for name in ('first_delay_s', 'max_receive_gap_s', 'last_silence_s')
    )


def analyze_samples(samples, start, finished, window_s, service_events=()):
    """Evaluate captured messages without ROS services, clocks or side effects."""
    ext=[]
    for _,_,m in samples['extnav']:
        try:
            value = json.loads(m.data)
            if isinstance(value, dict):
                ext.append(value)
        except (TypeError, ValueError):
            pass
    fcu=[m for _,_,m in samples['fcu']]
    texts=[{'elapsed_s':t-start,'severity':int(m.severity),'text':m.text} for t,_,m in samples['statustext']]
    estimator=[m for _,_,m in samples['estimator']]
    sys_status=[{'elapsed_s':t-start, 'present':int(m.sensors_present),
                 'enabled':int(m.sensors_enabled), 'health':int(m.sensors_health),
                 'prearm_enabled':bool(m.sensors_enabled & PREARM_CHECK),
                 'prearm_healthy':bool(m.sensors_health & PREARM_CHECK),
                 'vision_enabled':bool(m.sensors_enabled & VISION_POSITION),
                 'vision_healthy':bool(m.sensors_health & VISION_POSITION)}
                for t,_,m in samples['sys_status']]
    metrics={k:sample_metric(v,start,finished,header=k in ('odom','extnav_output','fcu_imu'),
                              age=k == 'odom') for k,v in samples.items()}
    continuity = {
        key: stream_continuous(metrics[key], gap)
        for key, gap in {'odom': 0.35, 'extnav': 0.50, 'extnav_output': 0.35,
                         'fcu': 2.5, 'sys_status': 2.5}.items()
    }
    continuity['odom'] &= (metrics['odom']['max_stamp_gap_s'] is not None
                           and metrics['odom']['max_stamp_gap_s'] <= 0.35
                           and metrics['odom']['nonincreasing_stamps'] == 0)
    continuity['extnav_output'] &= metrics['extnav_output']['nonincreasing_stamps'] == 0
    result={'schema_version':2,'window_s':window_s,'actual_window_s':finished-start,
            'metrics':metrics, 'continuity':continuity,
            'extnav_healthy_count':sum(x.get('healthy') is True for x in ext),
            'extnav_unhealthy_count':sum(x.get('healthy') is False for x in ext),
            'extnav_last':ext[-1] if ext else None,
            'fcu_connected_count':sum(bool(x.connected) for x in fcu),
            'fcu_armed_count':sum(bool(x.armed) for x in fcu),
            'fcu_last':({'connected':bool(fcu[-1].connected),'armed':bool(fcu[-1].armed),'mode':fcu[-1].mode} if fcu else None),
            'estimator_last':({name:bool(getattr(estimator[-1],name)) for name in (
                'attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag',
                'pos_horiz_rel_status_flag','pos_vert_abs_status_flag','accel_error_status_flag')} if estimator else None),
            'sys_status':sys_status,
            'prearm_final_healthy':bool(sys_status and sys_status[-1]['prearm_healthy']),
            'vision_final_healthy':bool(sys_status and sys_status[-1]['vision_healthy']),
            'first_prearm_healthy_elapsed_s':next((x['elapsed_s'] for x in sys_status if x['prearm_healthy']),None),
            'first_vision_healthy_elapsed_s':next((x['elapsed_s'] for x in sys_status if x['vision_healthy']),None),
            'statustext':texts,
            'arm_or_takeoff_texts':[x for x in texts if 'arming motors' in x['text'].lower() or 'takeoff' in x['text'].lower()],
            'service_events':service_events}
    result['criteria'] = {
        'observation_window_completed': finished - start >= window_s,
        'continuous_required_streams': all(continuity.values()),
        'external_nav_continuously_healthy': bool(ext)
            and result['extnav_healthy_count'] == len(samples['extnav']),
        'fcu_connected_and_never_armed': bool(fcu) and result['fcu_connected_count'] == len(fcu)
                                         and result['fcu_armed_count'] == 0,
        'no_arm_or_takeoff_evidence': not result['arm_or_takeoff_texts'],
        'fcu_prearm_final_healthy': result['prearm_final_healthy'],
        'fcu_vision_final_healthy': result['vision_final_healthy'],
    }
    result['passed'] = all(result['criteria'].values())
    return result


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--seconds',type=float,default=30); ap.add_argument('--output',required=True); a=ap.parse_args()
    rclpy.init(); n=Monitor(); service_events=[]
    stream=n.request_streams()
    start=time.monotonic(); end=start+a.seconds; next_prearm=start
    pending=[]
    while time.monotonic()<end and rclpy.ok():
        now=time.monotonic()
        if now >= next_prearm:
            future=n.request_prearm_check()
            if future is not None: pending.append((now-start,future))
            next_prearm += 10.0
        rclpy.spin_once(n, timeout_sec=0.2)
    finished=time.monotonic()
    service_events.append(service_event('stream_request', stream))
    for elapsed,future in pending:
        service_events.append(service_event('prearm_check', future, elapsed_s=elapsed))
    result = analyze_samples(n.samples, start, finished, a.seconds, service_events)
    open(a.output,'w').write(json.dumps(result,indent=2)+'\n'); n.destroy_node(); rclpy.shutdown()
    return 0 if result['passed'] else 1
if __name__=='__main__': raise SystemExit(main())
