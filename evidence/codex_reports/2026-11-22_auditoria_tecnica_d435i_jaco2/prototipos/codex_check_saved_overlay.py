import rclpy,time,numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
class N(Node):
 def __init__(self):
  super().__init__('codex_verificacion_nube');self.d={};self.create_subscription(PointCloud2,'/thesis/perception/points_filtered',lambda x:self.cb('in',x),qos_profile_sensor_data);self.create_subscription(PointCloud2,'/thesis/visualization/points_experimental',lambda x:self.cb('out',x),qos_profile_sensor_data)
 def cb(self,k,m):
  key=(m.header.stamp.sec,m.header.stamp.nanosec);self.d.setdefault(key,{})[k]=m
  if len(self.d)>100:self.d.pop(next(iter(self.d)))
rclpy.init();n=N();end=time.monotonic()+10
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 match=next((v for v in n.d.values() if len(v)==2),None)
 if match:break
if not match:raise RuntimeError('no synchronized matching clouds')
a,b=match['in'],match['out'];fields={x.name:x.offset for x in a.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in ('x','y','z')],'itemsize':a.point_step});aa=np.frombuffer(a.data,dtype=dt);bb=np.frombuffer(b.data,dtype=dt);xyz=np.column_stack([aa[k] for k in ('x','y','z')]);out=np.column_stack([bb[k] for k in ('x','y','z')]);p=np.load('/tmp/codex_fit_holdout_result.npz')['parameters'];expected=xyz@Rotation.from_rotvec(p[3:]).as_matrix().T+p[:3];print('points_in',len(aa),'points_out',len(bb),'max_error_m',float(np.max(abs(expected-out))),'frame',b.header.frame_id);assert len(aa)==len(bb) and np.max(abs(expected-out))<1e-6
n.destroy_node();rclpy.shutdown()
