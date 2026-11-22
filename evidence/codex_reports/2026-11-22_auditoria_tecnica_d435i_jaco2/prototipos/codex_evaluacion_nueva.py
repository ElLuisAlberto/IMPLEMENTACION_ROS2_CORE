import xml.etree.ElementTree as ET
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
D=np.load('/tmp/codex_new_fit.npz');root=ET.parse('/tmp/codex_live_robot.urdf').getroot();rng=np.random.default_rng(2026);np.random.seed(2026);allpts=[];labels=[]
for link in root.findall('link'):
 name=link.get('name')
 if name not in D:continue
 for visual in link.findall('visual'):
  m=visual.find('geometry/mesh')
  if m is None or 'ring' in m.get('filename'):continue
  path=m.get('filename').replace('package://thesis_description/','src/thesis_description/');mesh=trimesh.load(path,force='mesh'); pts=mesh.sample(max(500,int(mesh.area*120000)));tf=D[name];pts=pts@tf[:3,:3].T+tf[:3,3];allpts.append(pts);labels.extend([name]*len(pts))
target=np.concatenate(allpts);labels=np.array(labels);ix=rng.choice(len(target),min(18000,len(target)),replace=False);target=target[ix];labels=labels[ix];tree=cKDTree(target)
xyz=D['xyz'];rgb=D['colors'];ct=D['color_depth'];col=xyz@ct[:3,:3].T+ct[:3,3];bt=np.load('/tmp/codex_board_camera.npy');b=(col-bt[:3,3])@bt[:3,:3]
board=(b[:,0]>-.04)&(b[:,0]<.25)&(b[:,1]>-.04)&(b[:,1]<.20)&(abs(b[:,2])<.07)
valid=np.isfinite(xyz).all(1)&(rgb.mean(1)<85)&(~board)
source=xyz[valid];print('board excluded',board.sum(),'remaining',len(source),flush=True)

a=np.load('/tmp/codex_overlay_fit_result.npz')['parameters'];b=np.load('/tmp/codex_fit_holdout_result.npz')['parameters']
def transform(p):return source@Rotation.from_rotvec(p[3:]).as_matrix().T+p[:3]
da,ia=tree.query(transform(a));db,ib=tree.query(transform(b));roi=(da<.05)&(db<.05)
print('independent frame, common selection',int(roi.sum()),flush=True)
for name,dist,idx in [('anterior',da,ia),('actual',db,ib)]:
 for group,mask in [('todos',roi),('extremo',roi&np.isin(labels[ib],['j2n6s300_link_4','j2n6s300_link_5','j2n6s300_link_6'])),('eslabones',roi&np.isin(labels[ib],['j2n6s300_link_2','j2n6s300_link_3']))]:
  v=dist[mask]
  print(name,group,len(v),'median_mm',round(np.median(v)*1000,2),'p90_mm',round(np.percentile(v,90)*1000,2),'rms_mm',round(np.sqrt(np.mean(v*v))*1000,2),flush=True)
