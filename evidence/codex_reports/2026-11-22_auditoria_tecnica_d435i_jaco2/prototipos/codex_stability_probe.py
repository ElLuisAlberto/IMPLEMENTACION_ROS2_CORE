import sys,time,statistics,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Image,JointState
from tf2_msgs.msg import TFMessage
D={'cloud':PointCloud2,'rgb':Image,'joint':JointState,'tf':TFMessage}
class P(Node):
 def __init__(self):
  super().__init__('codex_stability_probe',enable_rosout=False,start_parameter_services=False);self.t=[];self.s=[];self.age=[];self.create_subscription(D[sys.argv[2]],sys.argv[1],self.cb,qos_profile_sensor_data)
 def cb(self,m):
  h=m.transforms[0].header if isinstance(m,TFMessage) and m.transforms else (m.header if not isinstance(m,TFMessage) else None)
  if h is None:return
  now=self.get_clock().now().nanoseconds;stamp=h.stamp.sec*1000000000+h.stamp.nanosec;self.t.append(time.monotonic_ns());self.s.append(stamp);self.age.append((now-stamp)/1e6)
rclpy.init();p=P();end=time.monotonic()+float(sys.argv[3])
while time.monotonic()<end:rclpy.spin_once(p,timeout_sec=.1)
if len(p.t)>2:
 d=np.diff(p.t)/1e6;sd=np.diff(p.s)/1e6
 print(sys.argv[1], 'samples',len(p.t),'hz',round(1000/np.mean(d),2),'gaps p50/p95/p99/max ms',np.round(np.percentile(d,[50,95,99,100]),1),'stamp gaps',np.round(np.percentile(sd,[50,95,99,100]),1),'age p50/p95/p99/max ms',np.round(np.percentile(p.age,[50,95,99,100]),1),'gaps>100ms',int(np.sum(d>100)),flush=True)
p.destroy_node();rclpy.shutdown()
