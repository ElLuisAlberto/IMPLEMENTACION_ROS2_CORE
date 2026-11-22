import struct,xml.etree.ElementTree as ET,numpy as np,trimesh
from scipy.spatial.transform import Rotation
from pathlib import Path
np.random.seed(42);root=ET.parse('/tmp/codex_live_robot.urdf').getroot();share=Path('install/thesis_description/share/thesis_description');out=[]
for link in root.findall('link'):
 name=link.get('name','')
 if not (name=='j2n6s300_link_base' or name in ['j2n6s300_link_'+str(i) for i in range(1,7)] or name.startswith('j2n6s300_link_finger')):continue
 parts=[]
 for visual in link.findall('visual'):
  mesh=visual.find('geometry/mesh')
  if mesh is None:continue
  path=share / mesh.get('filename').removeprefix('package://thesis_description/');obj=trimesh.load(str(path),force='mesh');pts=obj.sample(min(6000,max(400,int(obj.area*100000))));origin=visual.find('origin')
  if origin is not None:
   r=np.fromstring(origin.get('rpy','0 0 0'),sep=' ');t=np.fromstring(origin.get('xyz','0 0 0'),sep=' ');pts=pts@Rotation.from_euler('xyz',r).as_matrix().T+t
  parts.append(pts)
 if parts:out.append((name,np.concatenate(parts).astype('<f4')))
with open('/tmp/codex_mesh_points.bin','wb') as f:
 f.write(struct.pack('<I',len(out)))
 for name,points in out:
  n=name.encode();f.write(struct.pack('<II',len(n),len(points)));f.write(n);f.write(points.tobytes())
print('links',len(out),'points',sum(len(p) for _,p in out))
