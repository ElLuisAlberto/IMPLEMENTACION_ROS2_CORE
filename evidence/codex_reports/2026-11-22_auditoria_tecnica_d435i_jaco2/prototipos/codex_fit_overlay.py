import xml.etree.ElementTree as ET
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
D=np.load('/tmp/codex_live_fit.npz');root=ET.parse('/tmp/codex_live_robot.urdf').getroot();rng=np.random.default_rng(42);np.random.seed(42)
allpoints=[]
for link in root.findall('link'):
 name=link.get('name')
 if name not in D:continue
 for visual in link.findall('visual'):
  m=visual.find('geometry/mesh')
  if m is None or 'ring' in m.get('filename'):continue
  path=m.get('filename').replace('package://thesis_description/','src/thesis_description/')
  mesh=trimesh.load(path,force='mesh');sample=mesh.sample(max(500,int(mesh.area*100000)))
  mat=D[name];allpoints.append(sample@mat[:3,:3].T+mat[:3,3])
target=np.concatenate(allpoints); target=target[rng.choice(len(target),min(12000,len(target)),replace=False)]
xyz=D['xyz'];colors=D['colors']; good=np.isfinite(xyz).all(1)&(colors.mean(axis=1)<85)&(xyz[:,0]<0.09)
source=xyz[good];print('model',len(target),'source',len(source),flush=True)
if len(source)<1000:raise ValueError('not enough points')
source=source[rng.choice(len(source),min(3500,len(source)),replace=False)];r0=Rotation.from_euler('ZYX',[np.pi/2,-np.pi/6,0]).as_matrix()@D['camera_transform'][:3,:3];rot0=Rotation.from_matrix(r0).as_rotvec();tree=cKDTree(target)
# centroid translation alone; 3D cloud segmentation deliberately imperfect, results diagnostic only
best=None
for dyaw in [-.35,0,.35]:
 rot=Rotation.from_euler('z',dyaw).as_matrix()@r0; trans=np.median(target,axis=0)-np.median(source@rot.T,axis=0);p=np.r_[trans,Rotation.from_matrix(rot).as_rotvec()]
 for i in range(30):
  src=source@Rotation.from_rotvec(p[3:]).as_matrix().T+p[:3];dist,idx=tree.query(src,workers=1);keep=dist<min(.12,max(.035,np.percentile(dist,65)))
  if keep.sum()<100:break
  a=source[keep];b=target[idx[keep]]
  def residual(q):return ((a@Rotation.from_rotvec(q[3:]).as_matrix().T+q[:3]-b)).ravel()
  out=least_squares(residual,p,max_nfev=12,loss='soft_l1',f_scale=.025);new=out.x
  if np.linalg.norm(new-p)<1e-5:break
  p=new
 src=source@Rotation.from_rotvec(p[3:]).as_matrix().T+p[:3];dist,_=tree.query(src);metrics={'median_m':float(np.median(dist)),'p90_m':float(np.percentile(dist,90)),'within_20mm':float((dist<.02).mean()),'within_50mm':float((dist<.05).mean())};print('candidate',dyaw,metrics,'translation',p[:3],flush=True)
 if best is None or metrics['median_m']<best[0]:best=(metrics['median_m'],p,metrics)
np.savez('/tmp/codex_overlay_fit_result.npz',parameters=best[1]);print('BEST',best[2],best[1],flush=True)
