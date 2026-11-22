import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation

class Overlay(Node):
 def __init__(self):
  super().__init__('codex_visualizacion_aproximada')
  self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
  self.publisher=self.create_publisher(PointCloud2,'/codex/visualizacion/points_aproximados',qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/thesis/perception/points_filtered',self.on_cloud,qos_profile_sensor_data)
  self.last=0
  self.rot=Rotation.from_euler('ZYX',[math.pi/2,-math.pi/6,0]).as_matrix()
  self.origin=np.array([0.,-0.0675,0.031])
  self.get_logger().info('Solo visualizacion: posicion mm y camara hacia Y+, inclinacion +30 grados; sin TF ni control')
 def on_cloud(self,msg):
  now=self.get_clock().now().nanoseconds
  if now-self.last<100_000_000:return
  self.last=now
  try:
   tf=self.buffer.lookup_transform('d435i_link',msg.header.frame_id,Time())
   tr=tf.transform.translation;q=tf.transform.rotation
   rt=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
   rot=self.rot@rt; trans=self.rot@np.array([tr.x,tr.y,tr.z])+self.origin
   fields={f.name:f for f in msg.fields}
   if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):return
   msg.data=bytes(msg.data)
   data=bytearray(msg.data); n=msg.width*msg.height
   dt=np.dtype({'names':['x','y','z'],'formats':['>f4' if msg.is_bigendian else '<f4']*3,'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':msg.point_step})
   arr=np.ndarray((n,),dtype=dt,buffer=data)
   xyz=np.column_stack((arr['x'],arr['y'],arr['z'])); p=xyz@rot.T+trans
   for i,k in enumerate(('x','y','z')):arr[k]=p[:,i]
   msg.data=bytes(data);msg.header.frame_id='j2n6s300_link_base';self.publisher.publish(msg)
  except Exception as e:
   self.get_logger().warn(str(e),throttle_duration_sec=5)

rclpy.init();node=Overlay()
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.shutdown()
