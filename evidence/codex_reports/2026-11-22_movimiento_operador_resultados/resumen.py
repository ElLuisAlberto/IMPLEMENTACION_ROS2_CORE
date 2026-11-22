import sqlite3,sys,json,math,numpy as np
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray
from thesis_interfaces.msg import ProximityStatus
bag=Path(sys.argv[1]); c=sqlite3.connect(str(next(bag.glob('*.db3'))))
def get(t):return c.execute('select m.timestamp,m.data from messages m join topics t on t.id=m.topic_id where t.name=? order by m.timestamp',(t,))
a=[]; names=['j2n6s300_joint_'+str(i) for i in range(1,7)]
for ts,b in get('/joint_states'):
 m=deserialize_message(b,JointState)
 try:a.append((m.header.stamp.sec*1e9+m.header.stamp.nanosec,[m.position[m.name.index(n)] for n in names]))
 except (ValueError,IndexError):continue
t=np.array([x[0] for x in a]);p=np.unwrap(np.array([x[1] for x in a]),axis=0);v=np.diff(p,axis=0)/((np.diff(t)*1e-9)[:,None]);states={};bad=0
for ts,b in get('/thesis/proximity_status'):
 m=deserialize_message(b,ProximityStatus);states[m.state]=states.get(m.state,0)+1;bad+=not m.geometry_valid
faults=0;armed=0
for ts,b in get('/thesis/hardware/diagnostics'):
 m=deserialize_message(b,DiagnosticArray)
 for s in m.status:
  if 'JACO2' not in s.name:continue
  d={x.key:x.value for x in s.values};faults+=d.get('fault')=='true';armed+=d.get('armed')=='true'
res={'joint_names':names,'samples':len(t),'ranges_deg':np.round(np.ptp(p,axis=0)*180/math.pi,3).tolist(),'max_abs_finite_difference_deg_s':np.round(np.max(abs(v),axis=0)*180/math.pi,2).tolist(),'time_gap_ms_p50_p95_max':np.percentile(np.diff(t)/1e6,[50,95,100]).round(2).tolist(),'proximity_states':states,'invalid_geometry_messages':bad,'diagnostic_fault_samples':faults,'armed_diagnostic_samples':armed}
Path(sys.argv[2]).write_text(json.dumps(res,indent=2));print(json.dumps(res,indent=2))
