exec(open('/tmp/codex_bench_joint_optimization.py').read().split('for f in (old,new):')[0])
points=np.vstack([points,[[float('nan'),0,0],[float('inf'),0,0]],joints+[.02,0,0],joints+[.06,0,0]])
valid=np.isfinite(points).all(axis=1)
def baseline():
 d=np.full(len(points),np.inf);d[valid]=tree.query(points[valid],distance_upper_bound=.06,workers=1)[0]
 j=np.full(len(points),np.inf);j[valid]=jt.query(points[valid],distance_upper_bound=.14,workers=1)[0]
 a=valid&(d<.02);m=valid&~a&(d<.06)&(j<.14);return a,m,valid&~a&~m
def fast():
 d=np.full(len(points),np.inf);d[valid]=tree.query(points[valid],distance_upper_bound=.06,workers=4)[0]
 a=valid&(d<.02);c=valid&~a&(d<.06);m=np.zeros(len(points),dtype=bool)
 if np.any(c):m[c]=jt.query(points[c],distance_upper_bound=.14,workers=1)[0]<.14
 return a,m,valid&~a&~m
x=baseline();y=fast();print('points',len(points),'differences per class',[int(np.count_nonzero(a!=b)) for a,b in zip(x,y)])
assert all(np.array_equal(a,b) for a,b in zip(x,y))
for fn in (baseline,fast):
 times=[]
 for i in range(10):
  t=time.perf_counter();fn();times.append((time.perf_counter()-t)*1000)
 print(fn.__name__,'median_ms',round(float(np.median(times)),1),'min_ms',round(min(times),1),'max_ms',round(max(times),1))
