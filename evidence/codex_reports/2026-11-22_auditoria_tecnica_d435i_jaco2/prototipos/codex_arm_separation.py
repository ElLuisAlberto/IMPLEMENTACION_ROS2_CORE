import time, copy, xml.etree.ElementTree as ET
import numpy as np,trimesh,rclpy
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from sensor_msgs.msg import PointCloud2
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

class Separate(Node):
 def __init__(self):
  super().__init__('codex_separacion_solo_visualizacion')
  self.buffer=Buffer();self.listener=TransformListener(self.buffer,self);self.pending=[];self.last_stamp=None
  self.pub_arm=self.create_publisher(PointCloud2,'/codex/segmentacion/brazo_candidato',qos_profile_sensor_data)
  self.pub_other=self.create_publisher(PointCloud2,'/codex/segmentacion/resto_candidato',qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/thesis/visualization/points_experimental',self.recv,qos_profile_sensor_data)
  self.create_timer(.1,self.process)
  np.random.seed(42);self.meshes={};root=ET.parse('/tmp/codex_live_robot.urdf').getroot();share=Path(get_package_share_directory('thesis_description'))
  for link in root.findall('link'):
   name=link.get('name','')
   if not (name=='j2n6s300_link_base' or name in ['j2n6s300_link_'+str(i) for i in range(1,7)] or name.startswith('j2n6s300_link_finger')):continue
   parts=[]
   for visual in link.findall('visual'):
    mesh=visual.find('geometry/mesh')
    if mesh is None:continue
    path=str(share / mesh.get('filename').removeprefix('package://thesis_description/'))
    obj=trimesh.load(path,force='mesh'); num=min(6000,max(400,int(obj.area*100000))); p=obj.sample(num)
    origin=visual.find('origin')
    if origin is not None:
     rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ');xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ');p=p@Rotation.from_euler('xyz',rpy).as_matrix().T+xyz
    parts.append(p)
   if parts:self.meshes[name]=np.concatenate(parts)
  self.get_logger().warn('SOLO VISUALIZACION: proximidad a malla 20 mm, calibracion no validada; no para seguridad')
  self.get_logger().info('Mallas: %d enlaces' % len(self.meshes))
 def recv(self,m):
  self.pending.append(m);self.pending=self.pending[-8:]
 def process(self):
  if not self.pending:return
  now=self.get_clock().now().nanoseconds
  eligible=[m for m in self.pending if 150_000_000 < now-Time.from_msg(m.header.stamp).nanoseconds < 1_000_000_000]
  if not eligible:return
  m=eligible[-1];stamp=Time.from_msg(m.header.stamp)
  if self.last_stamp==stamp.nanoseconds:return
  try:
   meshes=[]
   for name,pts in self.meshes.items():
    if name=='j2n6s300_link_base':meshes.append(pts);continue
    t=self.buffer.lookup_transform('j2n6s300_link_base',name,stamp).transform;q=t.rotation;p=t.translation
    R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();meshes.append(pts@R.T+np.array([p.x,p.y,p.z]))
   offsets={x.name:x.offset for x in m.fields}
   dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k] for k in ('x','y','z')],'itemsize':m.point_step})
   data=np.frombuffer(m.data,dtype=dt); xyz=np.column_stack([data[k] for k in ('x','y','z')]);valid=np.isfinite(xyz).all(axis=1)
   near=np.zeros(len(xyz),dtype=bool);near[valid]=cKDTree(np.concatenate(meshes)).query(xyz[valid],distance_upper_bound=.020,workers=1)[0]<.020
   raw=np.frombuffer(m.data,dtype=np.uint8).reshape(-1,m.point_step)
   for mask,pub in ((near,self.pub_arm),(valid&~near,self.pub_other)):
    out=PointCloud2();out.header=m.header;out.height=1;out.width=int(mask.sum());out.fields=m.fields;out.is_bigendian=m.is_bigendian;out.point_step=m.point_step;out.row_step=out.width*out.point_step;out.is_dense=True;out.data=raw[mask].tobytes();pub.publish(out)
   self.last_stamp=stamp.nanoseconds
   self.get_logger().info('Separados %d brazo y %d restantes' % (near.sum(),(valid&~near).sum()),throttle_duration_sec=8)
  except Exception as e:self.get_logger().warn('No se publica sin TF de la captura: '+str(e),throttle_duration_sec=5)

rclpy.init();node=Separate()
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
