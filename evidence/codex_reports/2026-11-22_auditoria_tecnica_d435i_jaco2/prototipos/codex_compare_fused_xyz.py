import rclpy,time,numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
class N(Node):
 def __init__(self):
  super().__init__('codex_comp_mascaras');self.d={}
  for key,topic in [('old','/codex/segmentacion/resto_candidato'),('new','/codex/segmentacion_fused/resto')]:self.create_subscription(PointCloud2,topic,lambda m,k=key:self.cb(k,m),qos_profile_sensor_data)
 def cb(self,k,m):
  stamp=(m.header.stamp.sec,m.header.stamp.nanosec);self.d.setdefault(stamp,{})[k]=m
  if len(self.d)>100:self.d.pop(next(iter(self.d)))
rclpy.init();n=N();end=time.monotonic()+15;pair=None
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1);pair=next((v for v in n.d.values() if len(v)==2),None)
 if pair:break
if not pair:raise RuntimeError('no matching frame')
a,b=pair['old'],pair['new']
def xyz(m):
 f={x.name:x.offset for x in m.fields};dt=np.dtype({'names':['x','y','z','rgb'],'formats':['<f4','<f4','<f4','<u4'],'offsets':[f[k] for k in ('x','y','z','rgb')],'itemsize':m.point_step});ar=np.frombuffer(m.data,dtype=dt);return np.column_stack([ar[k] for k in ('x','y','z')]),ar['rgb']
x,c=xyz(a);y,d=xyz(b);print('old',len(x),'new',len(y),'old_step',a.point_step,'new_step',b.point_step)
if len(x)==len(y):print('maximum_coordinate_diff_m',np.max(np.abs(x-y)),'rgb_mismatches',np.count_nonzero(c!=d))
else:
 from scipy.spatial import cKDTree
 dd=cKDTree(x).query(y)[0];print('median_nearest_mm',np.median(dd)*1000,'max_nearest_mm',np.max(dd)*1000)
n.destroy_node();rclpy.shutdown()
