import sys,time,statistics,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
class Probe(Node):
 def __init__(self):
  super().__init__('codex_rate_probe',enable_rosout=False,start_parameter_services=False);self.stamps=[];self.arrivals=[];self.points=[];self.create_subscription(PointCloud2,sys.argv[1],self.cb,qos_profile_sensor_data)
 def cb(self,m):self.arrivals.append(time.monotonic());self.stamps.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9);self.points.append(m.width*m.height)
rclpy.init();n=Probe();end=time.monotonic()+7
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
if len(n.arrivals)>2:
 da=[y-x for x,y in zip(n.arrivals,n.arrivals[1:])]; ds=[y-x for x,y in zip(n.stamps,n.stamps[1:]) if y>x];print(sys.argv[1],'frames',len(n.arrivals),'received_hz',round((len(da))/(n.arrivals[-1]-n.arrivals[0]),2),'stamp_hz',round(1/statistics.median(ds),2),'median_points',statistics.median(n.points),'max_gap_ms',round(max(da)*1000))
else:print('insufficient frames',len(n.arrivals))
n.destroy_node();rclpy.shutdown()
