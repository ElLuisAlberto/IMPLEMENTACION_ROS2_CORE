import rclpy,time,statistics
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
class N(Node):
 def __init__(self):
  super().__init__('codex_latencia');self.age=[];self.create_subscription(PointCloud2,'/codex/segmentacion/resto_candidato',self.cb,qos_profile_sensor_data)
 def cb(self,m):self.age.append(self.get_clock().now().nanoseconds/1e9-(m.header.stamp.sec+m.header.stamp.nanosec/1e9))
rclpy.init();n=N();end=time.monotonic()+6
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
if n.age:print('samples',len(n.age),'median_age_ms',round(statistics.median(n.age)*1000),'max_age_ms',round(max(n.age)*1000))
n.destroy_node();rclpy.shutdown()
