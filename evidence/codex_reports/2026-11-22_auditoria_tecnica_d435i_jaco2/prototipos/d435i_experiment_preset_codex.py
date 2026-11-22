import rclpy, numpy as np, time, json, sys, subprocess
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import Image, CameraInfo

class Experiment(Node):
 def __init__(self):
  super().__init__('d435i_quality_experiment')
  self.frames=[]; self.stamps=[]; self.info=None; self.capture=False
  self.create_subscription(Image,'/camera/d435i/depth/image_rect_raw',self.on_image,qos_profile_sensor_data)
  self.create_subscription(CameraInfo,'/camera/d435i/depth/camera_info',self.on_info,qos_profile_sensor_data)
  self.setter=self.create_client(SetParameters,'/camera/d435i/set_parameters')
  self.getter=self.create_client(GetParameters,'/camera/d435i/get_parameters')
 def on_info(self,m): self.info=m
 def on_image(self,m):
  if not self.capture: return
  if m.encoding not in ('16UC1','mono16'): return
  d=np.frombuffer(m.data,dtype=np.dtype('>u2' if m.is_bigendian else '<u2')).reshape(m.height,m.step//2)[:,:m.width].copy()
  self.frames.append(d); self.stamps.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9)
 def call(self,client,request):
  if not client.wait_for_service(timeout_sec=10): raise RuntimeError('parameter service unavailable')
  f=client.call_async(request); rclpy.spin_until_future_complete(self,f,timeout_sec=8)
  if not f.done() or f.exception(): raise RuntimeError('parameter service failure')
  return f.result()
 def get(self,names):
  q=GetParameters.Request();q.names=names; vals=self.call(self.getter,q).values
  return {n:(v.bool_value if v.type==1 else v.integer_value if v.type==2 else v.double_value) for n,v in zip(names,vals)}
 def set(self,values):
  q=SetParameters.Request()
  for name,val in values.items():
   p=Parameter();p.name=name;p.value=ParameterValue()
   if isinstance(val,bool): p.value.type=ParameterType.PARAMETER_BOOL;p.value.bool_value=val
   elif isinstance(val,int):p.value.type=ParameterType.PARAMETER_INTEGER;p.value.integer_value=val
   else:p.value.type=ParameterType.PARAMETER_DOUBLE;p.value.double_value=float(val)
   q.parameters.append(p)
  a=self.call(self.setter,q)
  for name,result in zip(values,a.results):
   if not result.successful: raise RuntimeError(name+': '+result.reason)
 def spin(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=0.1)
 def collect(self,seconds):
  self.frames=[];self.stamps=[];self.capture=True;self.spin(seconds);self.capture=False
  if len(self.frames)<20 or self.info is None:raise RuntimeError('insufficient depth frames or camera info')
  return np.stack(self.frames),np.array(self.stamps)

rclpy.init(); n=Experiment()
names=['spatial_filter.enable','spatial_filter.filter_smooth_alpha','spatial_filter.filter_smooth_delta','spatial_filter.filter_magnitude','spatial_filter.holes_fill','temporal_filter.enable','depth_module.visual_preset','depth_module.laser_power','depth_module.exposure','depth_module.enable_auto_exposure']
original=n.get(names); print('ORIGINAL',json.dumps(original),flush=True)
configs=[('base',{}),('high_accuracy',{'depth_module.visual_preset':3}),('high_density',{'depth_module.visual_preset':4}),('medium_density',{'depth_module.visual_preset':5}),('default_preset',{'depth_module.visual_preset':1}),('base_repeat',{})]
results=[]; baseline_mask=None; fringe_mask=None
try:
 for label,changes in configs:
  subprocess.run(['/tmp/realsense_restore_codex'],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); n.set(original); n.set(changes); n.spin(4)
  actual=n.get(names); a,st=n.collect(7)
  h,w=a.shape[1:];k=n.info.k; yy,xx=np.indices((h,w)); radial=np.sqrt(1+((xx-k[2])/k[0])**2+((yy-k[5])/k[4])**2)
  # 16-bit depth values are millimetres for the ROS driver. Selection reproduces spherical distance cut.
  valid=(a>0)&(a*radial[None,:,:]>=100)&(a*radial[None,:,:]<=1500)
  freq=valid.mean(axis=0)
  if baseline_mask is None:
   # Evaluate a common fixed population with mostly reliable original depth; avoids coverage-driven bias.
   baseline_mask=freq>=0.90
   fringe_mask=(freq>=0.05)&(freq<0.90)
   np.save('/tmp/d435i_baseline_mask_codex.npy',baseline_mask)
  masked=np.where(valid,a,np.nan).astype(np.float32)
  with np.errstate(invalid='ignore',divide='ignore'):
   import warnings
   with warnings.catch_warnings():
    warnings.simplefilter('ignore',RuntimeWarning)
    median=np.nanmedian(masked,axis=0); mad=np.nanmedian(np.abs(masked-median),axis=0)
  common=baseline_mask & (freq>=0.8) & np.isfinite(mad)
  # Compare fixed population coverage and depth jitter; values are mm, not absolute accuracy.
  frame_counts=valid.sum(axis=(1,2)); dt=np.diff(st);dt=dt[dt>0]
  result={'name':label,'actual':actual,'frames':len(a),'rate_hz':round(float(1/np.median(dt)),2) if len(dt) else None,'roi_pixels':int(baseline_mask.sum()),'fringe_pixels':int(fringe_mask.sum()),'fringe_valid_pct':round(float(valid[:,fringe_mask].mean()*100),2),'roi_valid_pct':round(float(valid[:,baseline_mask].mean()*100),2),'common_pct':round(float(common.sum()/max(1,baseline_mask.sum())*100),2),'jitter_median_mm':round(float(np.nanmedian(mad[common])),3),'jitter_p90_mm':round(float(np.nanpercentile(mad[common],90)),3),'valid_points_median':round(float(np.median(frame_counts)),0),'valid_points_std':round(float(np.std(frame_counts)),1),'counts_min':int(frame_counts.min()),'counts_max':int(frame_counts.max()),'counts_p05':int(np.percentile(frame_counts,5)),'counts_p95':int(np.percentile(frame_counts,95)),'median_depth_mm':round(float(np.nanmedian(median[common])),2)}
  results.append(result); print('RESULT',json.dumps(result),flush=True)
finally:
 subprocess.run(['/tmp/realsense_restore_codex'],check=True); n.set(original);print('RESTORED',json.dumps(n.get(names)),flush=True)
 with open('/tmp/d435i_experiment_preset_codex.json','w') as f:json.dump(results,f,indent=2)
 n.destroy_node();rclpy.shutdown()
