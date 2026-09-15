#!/usr/bin/env python3
"""Observe startup topics for a bounded window without arming or commanding flight."""
import argparse, json, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from std_msgs.msg import String

class Monitor(Node):
    def __init__(self):
        super().__init__('impact_startup_smoke_monitor')
        self.samples = {'odom': [], 'extnav': [], 'fcu': []}
        self.create_subscription(Odometry, '/localization/odom', lambda m: self.samples['odom'].append((time.monotonic(), m)), 20)
        self.create_subscription(String, '/xq/p4/extnav/status', lambda m: self.samples['extnav'].append((time.monotonic(), m)), 20)
        self.create_subscription(State, '/uav1/mavros/state', lambda m: self.samples['fcu'].append((time.monotonic(), m)), 20)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--seconds',type=float,default=30); ap.add_argument('--output',required=True); a=ap.parse_args()
    rclpy.init(); n=Monitor(); start=time.monotonic(); end=start+a.seconds
    while time.monotonic()<end and rclpy.ok(): rclpy.spin_once(n, timeout_sec=0.2)
    def metric(values):
        times=[x[0] for x in values]; span=(max(times)-min(times)) if len(times)>1 else 0
        return {'count':len(values),'rate_hz':(len(times)-1)/span if span else 0.0,
                'first_monotonic_s':times[0] if times else None,'last_monotonic_s':times[-1] if times else None,
                'window_s':span}
    ext=[]
    for _,m in n.samples['extnav']:
        try: ext.append(json.loads(m.data))
        except Exception: pass
    fcu=[m for _,m in n.samples['fcu']]
    result={'schema_version':1,'window_s':a.seconds,'metrics':{k:metric(v) for k,v in n.samples.items()},
            'extnav_healthy_count':sum(x.get('healthy') is True for x in ext),
            'extnav_unhealthy_count':sum(x.get('healthy') is False for x in ext),
            'extnav_last':ext[-1] if ext else None,
            'fcu_connected_count':sum(bool(x.connected) for x in fcu),
            'fcu_armed_count':sum(bool(x.armed) for x in fcu),
            'fcu_last':({'connected':bool(fcu[-1].connected),'armed':bool(fcu[-1].armed),'mode':fcu[-1].mode} if fcu else None)}
    open(a.output,'w').write(json.dumps(result,indent=2)+'\n'); n.destroy_node(); rclpy.shutdown()
    return 0 if result['metrics']['odom']['count']>0 and result['metrics']['extnav']['count']>0 and result['metrics']['fcu']['count']>0 and result['fcu_armed_count']==0 else 1
if __name__=='__main__': raise SystemExit(main())
