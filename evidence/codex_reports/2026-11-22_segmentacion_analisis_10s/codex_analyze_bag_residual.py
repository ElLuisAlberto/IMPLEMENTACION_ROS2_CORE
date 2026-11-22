import sys,sqlite3,struct,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rclpy.duration import Duration
from rclpy.time import Time
p=Path(sys.argv[1]);conn=sqlite3.connect(str(next(p.glob('*.db3'))));topics={name:i for i,name in conn.execute('select id,name from topics')};buf=Buffer(cache_time=Duration(seconds=60)); counts={}
for name in ['/tf_static','/tf']:
 rows=conn.execute('select data from messages where topic_id=? order by timestamp',(topics[name],));n=0
 for (blob,) in rows:
  for t in deserialize_message(blob,TFMessage).transforms:
   try:
    (buf.set_transform_static if name=='/tf_static' else buf.set_transform)(t,'bag');n+=1
   except Exception as e: print('tf error',e)
 counts[name]=n
print('transforms',counts,flush=True)
links=[]
with open('/tmp/codex_mesh_points.bin','rb') as f:
 for _ in range(struct.unpack('<I',f.read(4))[0]):
  n,k=struct.unpack('<II',f.read(8));name=f.read(n).decode();pts=np.frombuffer(f.read(k*12),dtype='<f4').reshape(-1,3).copy();links.append((name,pts))
rows=conn.execute('select id,timestamp from messages where topic_id=? order by timestamp',(topics['/codex/segmentacion/resto_candidato'],)).fetchall();idx=np.linspace(0,len(rows)-1,11,dtype=int);output=[]
for index in idx:
 mid,t=rows[index];blob=conn.execute('select data from messages where id=?',(mid,)).fetchone()[0];m=deserialize_message(blob,PointCloud2);offs={x.name:x.offset for x in m.fields};dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offs[x] for x in 'xyz'],'itemsize':m.point_step});data=np.frombuffer(m.data,dtype=dtype);pnt=np.column_stack([data[x] for x in 'xyz']);pnt=pnt[np.isfinite(pnt).all(axis=1)];stamp=Time.from_msg(m.header.stamp)
 mesh=[]; labels=[]
 try:
  for name,pts in links:
   if name=='j2n6s300_link_base':v=pts
   else:
    tr=buf.lookup_transform('j2n6s300_link_base',name,stamp).transform;q=tr.rotation;t0=tr.translation;v=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(pts)+[t0.x,t0.y,t0.z]
   mesh.append(v);labels.extend([name]*len(v))
 except Exception as e:
  print('SKIP',index,str(e),flush=True);continue
 mesh=np.concatenate(mesh);labels=np.array(labels);dist,nearest=cKDTree(mesh).query(pnt,workers=4);pairs=cKDTree(pnt).query_pairs(.012,output_type='ndarray');n=len(pnt)
 graph=coo_matrix((np.ones(len(pairs)*2,dtype=np.uint8),(np.r_[pairs[:,0],pairs[:,1]],np.r_[pairs[:,1],pairs[:,0]])),shape=(n,n)).tocsr();num,lab=connected_components(graph,directed=False);sizes=np.bincount(lab,minlength=num);clusters=[]
 for cid in np.flatnonzero(sizes>=3):
  sel=lab==cid;pts=pnt[sel];d=dist[sel];nearest_names=labels[nearest[sel]];names,ncount=np.unique(nearest_names,return_counts=True);clusters.append({'n':int(sizes[cid]),'center_m':np.round(np.median(pts,axis=0),3).tolist(),'extent_m':np.round(np.ptp(pts,axis=0),3).tolist(),'mesh_mm_min':round(float(d.min()*1000),1),'mesh_mm_med':round(float(np.median(d)*1000),1),'link':str(names[np.argmax(ncount)])})
 clusters.sort(key=lambda x:x['n'],reverse=True);small=[c for c in clusters if c['n']<=500];near=[c for c in small if c['mesh_mm_min']<120];r={'frame':int(index),'stamp_ns':stamp.nanoseconds,'total':len(pnt),'cluster_count_3_500':len(small),'points_in_small':sum(c['n'] for c in small),'under_60mm':int((dist<.06).sum()),'under_100mm':int((dist<.1).sum()),'largest':clusters[:5],'small_near':near[:15]};output.append(r);print(json.dumps(r),flush=True)
Path(sys.argv[2]).write_text(json.dumps({'tf_counts':counts,'frames':output},indent=2))
