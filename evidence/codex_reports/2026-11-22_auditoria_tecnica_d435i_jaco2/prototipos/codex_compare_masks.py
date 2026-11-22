import rclpy,time,numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
class N(Node):
 def __init__(self):
  super().__init__('codex_comp_mascaras');self.d={}
  for key,topic in [('old','/codex/segmentacion/resto_candidato'),('new','/codex/segmentacion_cpp/resto')]:self.create_subscription(PointCloud2,topic,lambda m,k=key:self.cb(k,m),qos_profile_sensor_data)
 def cb(self,k,m):
  stamp=(m.header.stamp.sec,m.header.stamp.nanosec);self.d.setdefault(stamp,{})[k]=m
  if len(self.d)>100:self.d.pop(next(iter(self.d)))
rclpy.init();n=N();end=time.monotonic()+15;pair=None
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1);pair=next((v for v in n.d.values() if len(v)==2),None)
 if pair:break
if not pair:raise RuntimeError('no matching frame')
a,b=pair['old'],pair['new'];print('old',a.width,'new',b.width,'same fields',a.fields==b.fields,'identical bytes',bytes(a.data)==bytes(b.data));
if bytes(a.data)!=bytes(b.data):
 old=np.frombuffer(a.data,dtype='V'+str(a.point_step));new=np.frombuffer(b.data,dtype='V'+str(b.point_step));print('only_old',len(np.setdiff1d(old,new)),'only_new',len(np.setdiff1d(new,old)))
n.destroy_node();rclpy.shutdown()
