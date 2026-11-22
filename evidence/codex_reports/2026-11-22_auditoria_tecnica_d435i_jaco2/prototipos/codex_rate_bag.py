import sqlite3,numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,Image,JointState
from tf2_msgs.msg import TFMessage
p='evidence/codex_reports/2026-11-22_segmentacion_brazo_10s/2026-11-22_segmentacion_brazo_10s_0.db3';c=sqlite3.connect(p)
for topic,typ in [('/camera/d435i/depth/color/points',PointCloud2),('/camera/d435i/color/image_raw',Image),('/codex/segmentacion/brazo_candidato',PointCloud2),('/codex/segmentacion/margen_incierto',PointCloud2),('/codex/segmentacion/resto_candidato',PointCloud2),('/joint_states',JointState),('/tf',TFMessage)]:
 rows=c.execute('select m.timestamp,m.data from messages m join topics t on t.id=m.topic_id where t.name=? order by m.timestamp',(topic,)).fetchall();arrival=[];stamps=[];ages=[]
 for ts,b in rows:
  msg=deserialize_message(b,typ)
  h=msg.transforms[0].header if typ==TFMessage and msg.transforms else (msg.header if typ!=TFMessage else None)
  if not h:continue
  s=h.stamp.sec*1_000_000_000+h.stamp.nanosec
  if s==0:continue
  arrival.append(ts);stamps.append(s);ages.append((ts-s)/1e6)
 if len(arrival)<2:continue
 d=np.diff(arrival)/1e6;ds=np.diff(stamps)/1e6
 print(topic,'n',len(arrival),'hz',round(1000/np.mean(d),2),'arrival_gap_ms p50/p95/p99/max',np.round(np.percentile(d,[50,95,99,100]),1),'stamp_gap_ms p50/p95/p99/max',np.round(np.percentile(ds,[50,95,99,100]),1),'receipt_age_ms p50/p95/p99/max',np.round(np.percentile(ages,[50,95,99,100]),1),'gaps>100ms',sum(d>100))
