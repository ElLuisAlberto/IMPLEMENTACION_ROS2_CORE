import rclpy,time,numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
class N(Node):
 def __init__(self):
  super().__init__('codex_segmentacion_control');self.d={}
  for key,topic in [('input','/thesis/visualization/points_experimental'),('uncertain','/codex/segmentacion/margen_incierto'),('rest','/codex/segmentacion/resto_candidato')]:self.create_subscription(PointCloud2,topic,lambda m,k=key:self.cb(k,m),qos_profile_sensor_data)
 def cb(self,k,m):
  stamp=(m.header.stamp.sec,m.header.stamp.nanosec);self.d.setdefault(stamp,{})[k]=m
  if len(self.d)>100:self.d.pop(next(iter(self.d)))
rclpy.init();n=N();end=time.monotonic()+15;pair=None
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1);pair=next((v for v in n.d.values() if len(v)==3),None)
 if pair:break
if not pair:raise RuntimeError('No synchronized messages')
v=np.load('/tmp/codex_new_fit.npz');ct=v['color_depth'];T=np.load('/tmp/codex_board_camera.npy');p=np.load('/tmp/codex_fit_holdout_result.npz')['parameters'];R=Rotation.from_rotvec(p[3:]).as_matrix()
for name,msg in pair.items():
 f={x.name:x.offset for x in msg.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[f[k] for k in ('x','y','z')],'itemsize':msg.point_step});a=np.frombuffer(msg.data,dtype=dt); xyz=np.column_stack([a[k] for k in ('x','y','z')]);depth=(xyz-p[:3])@R;col=depth@ct[:3,:3].T+ct[:3,3];board=(col-T[:3,3])@T[:3,:3];match=(board[:,0]>.01)&(board[:,0]<.19)&(board[:,1]>.01)&(board[:,1]<.13)&(abs(board[:,2])<.02)
 print(name,'total',len(a),'board_roi',match.sum())
n.destroy_node();rclpy.shutdown()
