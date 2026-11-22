import sqlite3,json,math,numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import ExecutionTrajectory,CommandDecision
p='evidence/codex_reports/2026-11-22_filtro_j1_movimiento_10grados/2026-11-22_filtro_j1_movimiento_10grados_0.db3';db=sqlite3.connect(p)
def rows(t):return db.execute('select m.timestamp,m.data from messages m join topics t on t.id=m.topic_id where t.name=? order by m.timestamp',(t,))
for name,typ in [('/thesis/execution_trajectory',ExecutionTrajectory),('/thesis/command_decision',CommandDecision)]:
 for timestamp,data in rows(name):
  m=deserialize_message(data,typ);print(name,'recorded_ns',timestamp,'message',str(m),flush=True)
a=[]
for ts,data in rows('/joint_states'):
 m=deserialize_message(data,JointState);s=m.header.stamp.sec*10**9+m.header.stamp.nanosec;a.append([s,m.position[m.name.index('j2n6s300_joint_1')]])
a=np.asarray(a);t=(a[:,0]-a[0,0])/1e9;q=a[:,1];base=np.median(q[:40]);delta=(q-base)*180/math.pi;v=np.diff(q)/np.diff(t)*180/math.pi;moving=np.flatnonzero(np.abs(v)>.1)
print('n',len(q),'base_rad',base,'final_rad',np.median(q[-40:]),'delta_deg',delta[-1],'minmax_delta',delta.min(),delta.max(),'max_fd_deg_s',np.max(abs(v)),'motion_start_unix',a[moving[0],0]/1e9,'end',a[moving[-1]+1,0]/1e9,flush=True)
