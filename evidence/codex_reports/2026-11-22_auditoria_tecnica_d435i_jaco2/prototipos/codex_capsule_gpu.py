import time, copy, xml.etree.ElementTree as ET
import os, struct, subprocess
import yaml
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
  env=os.environ.copy();env.update({'__NV_PRIME_RENDER_OFFLOAD':'1','__GLX_VENDOR_LIBRARY_NAME':'nvidia'})
  self.gpu=subprocess.Popen(['/tmp/codex_gpu_mask'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,env=env);self.compared=False
  self.pub_arm=self.create_publisher(PointCloud2,'/codex/segmentacion/brazo_candidato',qos_profile_sensor_data)
  self.pub_other=self.create_publisher(PointCloud2,'/codex/segmentacion/resto_candidato',qos_profile_sensor_data)
  self.pub_margin=self.create_publisher(PointCloud2,'/codex/segmentacion/margen_incierto',qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/thesis/visualization/points_experimental',self.recv,qos_profile_sensor_data)
  self.create_timer(.1,self.process)
  profile=yaml.safe_load(Path('src/thesis_core/config/jaco_capsules.yaml').read_text())['capsule_visualizer']['ros__parameters']
  self.cap_frames=list(zip(profile['segment_start_frames'],profile['segment_end_frames'],profile['segment_radii']))
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
  self.get_logger().warn('SOLO VISUALIZACION: malla 20 mm y capsulas preventivas SOLO para incertidumbre, sin calibracion validada; no para seguridad')
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
   meshes=[];joint_centers=[];centers={'j2n6s300_link_base':np.zeros(3)}
   for name,pts in self.meshes.items():
    if name=='j2n6s300_link_base':meshes.append(pts);continue
    t=self.buffer.lookup_transform('j2n6s300_link_base',name,stamp).transform;q=t.rotation;p=t.translation
    R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();meshes.append(pts@R.T+np.array([p.x,p.y,p.z]))
    centers[name]=np.array([p.x,p.y,p.z])
    if name in ['j2n6s300_link_'+str(i) for i in range(1,7)]:joint_centers.append([p.x,p.y,p.z])
   ef=self.buffer.lookup_transform('j2n6s300_link_base','j2n6s300_end_effector',stamp).transform.translation
   centers['j2n6s300_end_effector']=np.array([ef.x,ef.y,ef.z])
   offsets={x.name:x.offset for x in m.fields}
   dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k] for k in ('x','y','z')],'itemsize':m.point_step})
   data=np.frombuffer(m.data,dtype=dt); xyz=np.column_stack([data[k] for k in ('x','y','z')]);valid=np.isfinite(xyz).all(axis=1)
   packed_points=np.zeros((len(xyz),4),dtype=np.float32);packed_points[:,:3]=xyz
   parts=[];bounds=[];first=0
   for points in meshes:
    a=np.zeros((len(points),4),dtype=np.float32);a[:,:3]=points;parts.append(a)
    bounds.extend([list(a[:,:3].min(axis=0))+[first],list(a[:,:3].max(axis=0))+[first+len(a)]]);first+=len(a)
   packed_mesh=np.concatenate(parts);bounds=np.asarray(bounds,dtype=np.float32)
   caps=np.zeros((len(self.cap_frames)*2+6,4),dtype=np.float32)
   for i,(start,end,radius) in enumerate(self.cap_frames):caps[2*i,:3]=centers[start];caps[2*i,3]=radius;caps[2*i+1,:3]=centers[end]
   caps[len(self.cap_frames)*2:,:3]=np.asarray(joint_centers,dtype=np.float32)
   stream=self.gpu.stdin;stream.write(struct.pack('<4I',len(xyz),len(packed_mesh),len(meshes),len(self.cap_frames)))
   for part in (packed_points,packed_mesh,bounds,caps):stream.write(part.tobytes())
   stream.flush();rawlabels=self.gpu.stdout.read(len(xyz)*4)
   if len(rawlabels)!=len(xyz)*4:raise RuntimeError('GPU output incomplete')
   labels=np.frombuffer(rawlabels,dtype=np.uint32)
   if not self.compared:
    dist=np.full(len(xyz),np.inf);dist[valid]=cKDTree(np.concatenate(meshes)).query(xyz[valid],distance_upper_bound=.060,workers=1)[0]
    jointdist=np.full(len(xyz),np.inf);jointdist[valid]=cKDTree(np.asarray(joint_centers)).query(xyz[valid],distance_upper_bound=.140,workers=1)[0]
    cpu_near=valid&(dist<.020);cpu_margin=valid&~cpu_near&(dist<.060)&(jointdist<.140)
    for start,end,radius in self.cap_frames:
     a=centers[start];d=centers[end]-a;den=np.dot(d,d);fraction=np.clip(((xyz-a)@d)/den,0,1) if den>1e-12 else np.zeros(len(xyz));closest=a+fraction[:,None]*d;cpu_margin|=valid&~cpu_near&(np.linalg.norm(xyz-closest,axis=1)<=float(radius))
    cpulabel=np.where(cpu_near,0,np.where(cpu_margin,1,2));diff=int(np.count_nonzero((labels!=cpulabel)&valid));self.get_logger().info('Comparacion GPU/CPU: %d diferencias de %d puntos' % (diff,valid.sum()))
    if diff>5:raise RuntimeError('GPU/CPU classification discrepancy')
    self.compared=True
   near=valid&(labels==0);margin=valid&(labels==1);other=valid&(labels==2)
   raw=np.frombuffer(m.data,dtype=np.uint8).reshape(-1,m.point_step)
   for mask,pub in ((near,self.pub_arm),(margin,self.pub_margin),(other,self.pub_other)):
    out=PointCloud2();out.header=m.header;out.height=1;out.width=int(mask.sum());out.fields=m.fields;out.is_bigendian=m.is_bigendian;out.point_step=m.point_step;out.row_step=out.width*out.point_step;out.is_dense=True;out.data=raw[mask].tobytes();pub.publish(out)
   self.last_stamp=stamp.nanoseconds
   self.get_logger().info('GPU: %d brazo, %d inciertos y %d restantes' % (near.sum(),margin.sum(),other.sum()),throttle_duration_sec=8)
  except Exception as e:self.get_logger().warn('No se publica sin TF de la captura: '+str(e),throttle_duration_sec=5)

rclpy.init();node=Separate()
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.gpu.stdin.close();node.gpu.wait(timeout=3)
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
