import time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from scipy.spatial.transform import Rotation
class Probe(Node):
 def __init__(self):
  super().__init__('codex_observacion_ajuste');self.cloud=None;self.urdf=None;self.buf=Buffer();self.tf=TransformListener(self.buf,self)
  self.create_subscription(PointCloud2,'/thesis/perception/points_filtered',self.cb,qos_profile_sensor_data)
  self.create_subscription(String,'/robot_description',self.desc,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 def cb(self,m):self.cloud=m
 def desc(self,m):self.urdf=m.data
rclpy.init();n=Probe();end=time.monotonic()+15
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 if n.cloud is not None and n.urdf:
  try:
   names=['j2n6s300_link_'+str(i) for i in range(1,7)]+['j2n6s300_link_base']
   tfs={}
   for name in names:
    if name==names[-1]:tfs[name]=np.eye(4);continue
    t=n.buf.lookup_transform('j2n6s300_link_base',name,Time()).transform
    mat=np.eye(4);mat[:3,:3]=Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix();mat[:3,3]=[t.translation.x,t.translation.y,t.translation.z];tfs[name]=mat
   tc=n.buf.lookup_transform('d435i_link',n.cloud.header.frame_id,Time()).transform
   col=n.buf.lookup_transform('d435i_color_optical_frame',n.cloud.header.frame_id,Time()).transform
   color_depth=np.eye(4);color_depth[:3,:3]=Rotation.from_quat([col.rotation.x,col.rotation.y,col.rotation.z,col.rotation.w]).as_matrix();color_depth[:3,3]=[col.translation.x,col.translation.y,col.translation.z]
   cam=np.eye(4);cam[:3,:3]=Rotation.from_quat([tc.rotation.x,tc.rotation.y,tc.rotation.z,tc.rotation.w]).as_matrix();cam[:3,3]=[tc.translation.x,tc.translation.y,tc.translation.z]
   f={p.name:p for p in n.cloud.fields};c=n.cloud; dt=np.dtype({'names':['x','y','z','rgb'],'formats':['<f4']*3+['<u4'],'offsets':[f[k].offset for k in ('x','y','z','rgb')],'itemsize':c.point_step});a=np.frombuffer(c.data,dtype=dt);xyz=np.column_stack([a[k] for k in ('x','y','z')]);rgb=a['rgb']; colors=np.column_stack([(rgb>>16)&255,(rgb>>8)&255,rgb&255]);
   with open('/tmp/codex_live_robot.urdf','w') as out:out.write(n.urdf)
   np.savez('/tmp/codex_new_fit.npz',xyz=xyz,colors=colors,camera_transform=cam,color_depth=color_depth,**tfs)
   print('points',len(xyz),'links',len(tfs),'cloud_frame',c.header.frame_id);break
  except Exception as e:err=str(e)
else:print('snapshot failed',locals().get('err','no data'))
n.destroy_node();rclpy.shutdown()
