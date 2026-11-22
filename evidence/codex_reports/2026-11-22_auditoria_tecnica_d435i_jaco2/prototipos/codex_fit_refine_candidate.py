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
# Reject distant room contamination from the black-arm selection using the existing candidate.
p0=np.load('/tmp/codex_fit_holdout_result.npz')['parameters']; q0=source@Rotation.from_rotvec(p0[3:]).as_matrix().T+p0[:3];nearest,_=tree.query(q0);source=source[nearest<.08]
perm=rng.permutation(len(source)); train=source[perm[:min(3500,len(perm)//2)]]; held=source[perm[min(3500,len(perm)//2):min(8500,len(perm))]]
print('training',len(train),'heldout',len(held),flush=True)
def stats(p):
 points=held@Rotation.from_rotvec(p[3:]).as_matrix().T+p[:3];dist,idx=tree.query(points);result={}
 for group,mask in [('all',np.ones(len(dist),bool)),('hand',np.isin(labels[idx],['j2n6s300_link_4','j2n6s300_link_5','j2n6s300_link_6'])),('arm',np.isin(labels[idx],['j2n6s300_link_2','j2n6s300_link_3']))]:
  v=dist[mask];result[group]={'count':len(v),'median_mm':round(float(np.median(v))*1000,2),'p90_mm':round(float(np.percentile(v,90))*1000,2),'within20':round(float(np.mean(v<.02))*100,1)} if len(v)>10 else {'count':len(v)}
 return result
print('BASE',stats(p0),flush=True)
best=None
for angle in [-.15,0,.15]:
 p=p0.copy(); p[3:]=Rotation.from_matrix(Rotation.from_rotvec([0,0,angle]).as_matrix()@Rotation.from_rotvec(p0[3:]).as_matrix()).as_rotvec()
 for i in range(35):
  pts=train@Rotation.from_rotvec(p[3:]).as_matrix().T+p[:3];dist,idx=tree.query(pts);keep=dist<min(.05,max(.015,np.percentile(dist,75)));a=train[keep];b=target[idx[keep]]
  if len(a)<100:break
  def fun(q):return (a@Rotation.from_rotvec(q[3:]).as_matrix().T+q[:3]-b).ravel()
  q=least_squares(fun,p,max_nfev=12,loss='soft_l1',f_scale=.015).x
  if np.linalg.norm(q-p)<1e-5:break
  p=q
 st=stats(p);print('CANDIDATE',angle,st,'pos',p[:3],flush=True)
 if best is None or st['all']['p90_mm']<best[0]:best=(st['all']['p90_mm'],p,st)
np.savez('/tmp/codex_fit_refine_candidate.npz',parameters=best[1]);print('BEST',best[2],flush=True)
