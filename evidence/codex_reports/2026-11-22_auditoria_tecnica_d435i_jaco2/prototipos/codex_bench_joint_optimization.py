import struct, time
import numpy as np
from scipy.spatial import cKDTree
rng=np.random.default_rng(42)
with open('/tmp/codex_mesh_points.bin','rb') as f:
 n=struct.unpack('<I',f.read(4))[0]; links=[]
 for i in range(n):
  a,b=struct.unpack('<II',f.read(8));f.read(a);links.append(np.frombuffer(f.read(b*12),dtype='<f4').reshape(-1,3))
mesh=np.concatenate(links); joints=mesh[rng.choice(len(mesh),6,replace=False)]; points=np.vstack([rng.uniform(-.8,.8,(30000,3)), mesh[rng.choice(len(mesh),25000)]+rng.normal(0,.018,(25000,3))]); rng.shuffle(points)
tree=cKDTree(mesh); jt=cKDTree(joints)
def old():
 d=tree.query(points,distance_upper_bound=.06,workers=1)[0];j=jt.query(points,distance_upper_bound=.14,workers=1)[0];a=d<.02; m=~a&(d<.06)&(j<.14);return a,m
def new():
 j=jt.query(points,distance_upper_bound=.14,workers=1)[0]<.14;d=np.full(len(points),np.inf);d[j]=tree.query(points[j],distance_upper_bound=.06,workers=1)[0];d[~j]=tree.query(points[~j],distance_upper_bound=.02,workers=1)[0];a=d<.02;return a,~a&(d<.06)&j
for f in (old,new):
 result=f();t=time.perf_counter();
 for _ in range(5): result=f()
 print(f.__name__,round((time.perf_counter()-t)*200,2),'ms',result[0].sum(),result[1].sum())
a=old();b=new();print('differences',sum(np.count_nonzero(x!=y) for x,y in zip(a,b)))
