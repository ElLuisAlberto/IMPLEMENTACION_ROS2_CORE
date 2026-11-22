import sys,sqlite3,struct,json
from pathlib import Path
from collections import Counter
import numpy as np,yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rclpy.duration import Duration
from rclpy.time import Time
bag=Path(sys.argv[1]);out=Path(sys.argv[2]);db=sqlite3.connect(str(next(bag.glob('*.db3'))));topics={name:i for i,name in db.execute('select id,name from topics')}
get=lambda t: db.execute('select id,timestamp,data from messages where topic_id=? order by timestamp',(topics[t],))
buf=Buffer(cache_time=Duration(seconds=150))
for topic in ['/tf_static','/tf']:
 for _,_,data in get(topic):
  for t in deserialize_message(data,TFMessage).transforms:
   (buf.set_transform_static if topic=='/tf_static' else buf.set_transform)(t,'bag')
links=[];meshfile=Path('evidence/codex_reports/2026-11-22_auditoria_tecnica_d435i_jaco2/prototipos/codex_mesh_points.bin')
with meshfile.open('rb') as f:
 for _ in range(struct.unpack('<I',f.read(4))[0]):
  n,k=struct.unpack('<II',f.read(8));name=f.read(n).decode();links.append((name,np.frombuffer(f.read(k*12),dtype='<f4').reshape(-1,3).copy()))
caps=yaml.safe_load(Path('evidence/codex_reports/2026-11-22_auditoria_tecnica_d435i_jaco2/configuracion/jaco_capsules.yaml').read_text())['capsule_visualizer']['ros__parameters'];caprows=list(zip(caps['segment_start_frames'],caps['segment_end_frames'],caps['segment_radii']))
T=np.array([[.99678283379185739,-.030613137601291703,.074073058962612362,-.049989844477949082],[-.063846257727633193,.25541372412541591,.9647214545673598,-.65888731019659497],[-.04845242648259121,-.96634707291665667,.25263748144992021,.24566380982590308]])
names=['/camera/d435i/depth/color/points','/codex/segmentacion/brazo_candidato','/codex/segmentacion/margen_incierto','/codex/segmentacion/resto_candidato'];lookup={};stats={}
for name in names:
 d={};sizes=[];timestamps=[]
 for mid,arrival,data in get(name):
  m=deserialize_message(data,PointCloud2);s=m.header.stamp.sec*10**9+m.header.stamp.nanosec;d[s]=mid;sizes.append(m.width*m.height);timestamps.append(arrival)
 lookup[name]=d;gaps=np.diff(timestamps)/1e6
 stats[name]={'mensajes':len(sizes),'puntos_min_med_max':[int(min(sizes)),float(np.median(sizes)),int(max(sizes))],'hz_recepcion':float(1e9/np.mean(np.diff(timestamps))),'gap_ms_p50_p95_max':np.percentile(gaps,[50,95,100]).round(2).tolist()}
common_all=sorted(set.intersection(*(set(d) for d in lookup.values())))
js=[]
for _,_,data in get('/joint_states'):
 m=deserialize_message(data,JointState);s=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 if len(m.position)>=6:js.append((s,list(m.position[:6])))
jt=np.array([a[0] for a in js],dtype=np.int64); jp=np.unwrap(np.array([a[1] for a in js]),axis=0);speed=np.max(np.abs(np.diff(jp,axis=0))/(np.diff(jt)[:,None]*1e-9),axis=1)
common=[s for s in common_all if 0<=np.searchsorted(jt,s)-1<len(speed) and speed[np.searchsorted(jt,s)-1] > np.deg2rad(2)]
selected=[common[i] for i in np.linspace(0,len(common)-1,min(20,len(common)),dtype=int)] if common else []
print('common',len(common),'selected',len(selected),flush=True)
def msg(name,s):return deserialize_message(db.execute('select data from messages where id=?',(lookup[name][s],)).fetchone()[0],PointCloud2)
def xyz(m):
 f={i.name:i.offset for i in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[f[x] for x in 'xyz'],'itemsize':m.point_step});a=np.frombuffer(m.data,dtype=dt);return np.column_stack([a[x] for x in 'xyz'])
def asrecords(m):return np.frombuffer(m.data,dtype=np.dtype('V'+str(m.point_step)))
results=[]
for s in selected:
 raw=msg(names[0],s);outs=[msg(n,s) for n in names[1:]];q=xyz(raw).astype(np.float64);d=(q*q).sum(axis=1);valid=np.isfinite(d)&(d>=.01)&(d<=2.25);q=q[valid];expected_count=len(q);stamp=Time(nanoseconds=s);mesh=[];centers={'j2n6s300_link_base':np.zeros(3)}
 for name,p in links:
  if name=='j2n6s300_link_base':v=p
  else:
   tr=buf.lookup_transform('j2n6s300_link_base',name,stamp).transform;r=tr.rotation;t=tr.translation;v=Rotation.from_quat([r.x,r.y,r.z,r.w]).apply(p)+[t.x,t.y,t.z];centers[name]=np.array([t.x,t.y,t.z])
  mesh.append(v)
 tf=buf.lookup_transform('j2n6s300_link_base','j2n6s300_end_effector',stamp).transform.translation;centers['j2n6s300_end_effector']=np.array([tf.x,tf.y,tf.z]);tree=cKDTree(np.concatenate(mesh));joints=np.array([centers['j2n6s300_link_'+str(i)] for i in range(1,7)]);allpoints=np.concatenate([xyz(m) for m in outs]);given=np.concatenate([np.full(m.width*m.height,i,dtype=np.uint8) for i,m in enumerate(outs)]);dist=tree.query(allpoints,distance_upper_bound=.06,workers=4)[0];jd=cKDTree(joints).query(allpoints,distance_upper_bound=.14,workers=1)[0];near=dist<.02;unc=(dist<.06)&(jd<.14)
 for a,b,r in caprows:
  aa=centers[a];axis=centers[b]-aa;den=np.dot(axis,axis);z=np.clip(((allpoints-aa)@axis)/den,0,1) if den>1e-12 else np.zeros(len(allpoints));unc|=np.linalg.norm(allpoints-(aa+z[:,None]*axis),axis=1)<=float(r)
 ref=np.where(near,0,np.where(unc,1,2));diff=np.flatnonzero(ref!=given)
 # Recreate input's float32 XYZ and retain the original other field bytes.
 transformed=np.asarray(q@T[:,:3].T+T[:,3],dtype='<f4');source=np.frombuffer(raw.data,dtype=np.uint8).reshape(-1,raw.point_step)[valid].copy();f={i.name:i.offset for i in raw.fields}
 for j,k in enumerate('xyz'):source[:,f[k]:f[k]+4]=transformed[:,j].copy().view(np.uint8).reshape(-1,4)
 source=np.ascontiguousarray(source).view(np.dtype('V'+str(raw.point_step))).ravel();rec=np.concatenate([asrecords(m) for m in outs]);src_sorted=np.sort(source);rec_sorted=np.sort(rec)
 exact=bool(len(src_sorted)==len(rec_sorted) and np.array_equal(src_sorted,rec_sorted));r={'stamp_ns':s,'input_valid':expected_count,'class_counts':[int(o.width*o.height) for o in outs],'count_partition_ok':expected_count==len(allpoints),'byte_multiset_equal':exact,'cpu_gpu_differences':len(diff),'distance_mm_of_differences':(np.round(dist[diff[:10]]*1000,3).tolist() if len(diff) else [])};results.append(r);print(json.dumps(r),flush=True)
joints=[];jnames=None
for _,_,data in get('/joint_states'):
 m=deserialize_message(data,JointState)
 if len(m.position)>=6:
  if jnames is None:jnames=list(m.name)
  if list(m.name)==jnames:joints.append(m.position[:6])
j=np.asarray(joints);summary={'joint_names':jnames[:6] if jnames else [],'samples':len(j),'range_rad':np.ptp(j,axis=0).tolist(),'all_common_frames':len(common),'topics':stats,'comparison':results}
out.write_text(json.dumps(summary,indent=2))
