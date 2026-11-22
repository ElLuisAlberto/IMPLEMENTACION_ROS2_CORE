import math,time,rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import JointCommand,CommandDecision,ExecutionTrajectory
class Test(Node):
 def __init__(self):
  super().__init__('codex_prueba_sim_j1',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
  self.joint=None;self.decision=None;self.out=None;self.pub=self.create_publisher(JointCommand,'/thesis/candidate_command',10)
  self.create_subscription(JointState,'/joint_states',self.cb,10);self.create_subscription(CommandDecision,'/thesis/command_decision',self.dec,10);self.create_subscription(ExecutionTrajectory,'/thesis/execution_trajectory',self.exec,10)
 def cb(self,m):self.joint=m
 def dec(self,m):
  if m.command_id=='codex_sim_j1_10deg':self.decision=m;print('decision',m.accepted,m.state,m.reason_code,m.reason,flush=True)
 def exec(self,m):
  if m.command_id=='codex_sim_j1_10deg':self.out=m;print('execution',m.status,m.detail,flush=True)
rclpy.init();n=Test();end=time.monotonic()+15
while time.monotonic()<end and (n.joint is None or n.pub.get_subscription_count()==0):rclpy.spin_once(n,timeout_sec=.1)
if n.joint is None or not n.pub.get_subscription_count():raise RuntimeError('simulation unavailable')
names=['j2n6s300_joint_'+str(i) for i in range(1,7)];q=[n.joint.position[n.joint.name.index(k)] for k in names];m=JointCommand();m.stamp=n.get_clock().now().to_msg();m.command_id='codex_sim_j1_10deg';m.joint_names=names;m.positions=q.copy();m.positions[0]+=math.radians(10);m.duration.sec=2;print('simulation start',q,'target',m.positions,flush=True);n.pub.publish(m)
end=time.monotonic()+25
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 if n.out and n.out.status in ['SUCCEEDED','FAILED','CANCELED','REJECTED']:break
if not n.out or n.out.status!='SUCCEEDED':raise RuntimeError('simulation did not complete successfully')
print('final J1',n.joint.position[n.joint.name.index(names[0])],flush=True);n.destroy_node();rclpy.shutdown()
