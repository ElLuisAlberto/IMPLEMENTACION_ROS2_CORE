import numpy as np,struct,subprocess,os
from pathlib import Path
from scipy.spatial import cKDTree
rng=np.random.default_rng(4);n=8000;nc=6;nl=3;ms=[];bounds=[]
for i in range(nl):
 a=rng.uniform(-.5,.5,(220,3)).astype('f4');ms.append(a);bounds.extend([list(a.min(0))+[i*220],list(a.max(0))+[(i+1)*220]])
mesh=np.concatenate(ms);xyz=rng.uniform(-.6,.6,(n,3)).astype('f4');xyz[:250]=mesh[:250]+rng.normal(0,.015,(250,3));pts=np.column_stack([xyz,np.zeros(n)]).astype('f4');bounds=np.array(bounds,dtype='f4');caps=np.zeros((18,4),dtype='f4');joints=rng.uniform(-.5,.5,(6,3)).astype('f4');caps[12:,:3]=joints
for i in range(nc):caps[2*i,:3]=rng.uniform(-.4,.4,3);caps[2*i,3]=.08;caps[2*i+1,:3]=rng.uniform(-.4,.4,3)
dist=cKDTree(mesh).query(xyz)[0];near=dist<.02;jd=cKDTree(joints).query(xyz)[0];marg=(dist<.06)&(jd<.14)
for i in range(nc):
 a=caps[2*i,:3];b=caps[2*i+1,:3];d=b-a;dot=np.dot(d,d);t=np.clip((xyz-a)@d/dot,0,1);marg|=(np.linalg.norm(xyz-(a+t[:,None]*d),axis=1)<=caps[2*i,3])
expected=np.where(near,0,np.where(marg,1,2)).astype('u4');E=os.environ.copy();E.update({'__NV_PRIME_RENDER_OFFLOAD':'1','__GLX_VENDOR_LIBRARY_NAME':'nvidia'});proc=subprocess.Popen([str(Path(__file__).resolve().parents[3] / 'build/experimental_arm_segmentation/gpu_mask')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,env=E);m=np.column_stack([mesh,np.zeros(len(mesh))]).astype('f4');proc.stdin.write(struct.pack('<4I',n,len(m),nl,nc)+pts.tobytes()+m.tobytes()+bounds.tobytes()+caps.tobytes());proc.stdin.flush();out=np.frombuffer(proc.stdout.read(n*4),dtype='u4');print('points',n,'differences',np.count_nonzero(out!=expected));assert np.array_equal(out,expected);proc.stdin.close();proc.wait(timeout=3)
