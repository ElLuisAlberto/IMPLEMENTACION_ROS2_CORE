import math,time,rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus
from std_msgs.msg import Bool
from thesis_interfaces.msg import JointCommand,CommandDecision,ExecutionTrajectory,ProximityStatus
class Test(Node):
 def __init__(self):
  super().__init__('codex_prueba_j1_fisico');self.joint=None;self.diag=None;self.prox=None;self.ready=None;self.received={};self.decision=None;self.out=None;self.p=self.create_publisher(JointCommand,'/thesis/candidate_command',10)
  self.create_subscription(JointState,'/joint_states',lambda m:self.store('joint',m),10);self.create_subscription(DiagnosticArray,'/thesis/hardware/diagnostics',lambda m:self.store('diag',m),10);self.create_subscription(ProximityStatus,'/thesis/proximity_status',lambda m:self.store('prox',m),10);self.create_subscription(Bool,'/thesis/system_ready',lambda m:self.store('ready',m),10);self.create_subscription(CommandDecision,'/thesis/command_decision',self.dec,10);self.create_subscription(ExecutionTrajectory,'/thesis/execution_trajectory',self.exe,10)
 def store(self,k,m):setattr(self,k,m);self.received[k]=time.monotonic()
 def dec(self,m):
  if m.command_id=='codex_filtro_j1_10deg_20261122':self.decision=m;print('decision',m.accepted,m.state,m.reason_code,m.reason,flush=True)
 def exe(self,m):
  if m.command_id=='codex_filtro_j1_10deg_20261122':self.out=m;print('execution',m.status,m.detail,flush=True)
rclpy.init();n=Test();end=time.monotonic()+12
while time.monotonic()<end and (len(n.received)<4 or n.p.get_subscription_count()<1):rclpy.spin_once(n,timeout_sec=.1)
if len(n.received)<4 or n.p.get_subscription_count()<1:raise RuntimeError('Missing or ambiguous safety stack')
if any(time.monotonic()-v>1.0 for v in n.received.values()):raise RuntimeError('Stale status')
s=next((s for s in n.diag.status if 'JACO2' in s.name),None)
if s is None:raise RuntimeError('Missing hardware diagnostics')
d={x.key:x.value for x in s.values};required={'connected':'true','armed':'true','state_valid':'true','system_ready':'true','proximity_valid':'true','safety_stop_active':'false','fault':'false','active_goal':'false','jog_active':'false'}
if s.level!=DiagnosticStatus.OK or any(d.get(k)!=v for k,v in required.items()) or not n.ready.data or not n.prox.geometry_valid or n.prox.state!='ALLOW':raise RuntimeError('Safety preconditions not met: '+str(d))
if not n.joint.velocity or any(abs(v)>.01 for v in n.joint.velocity[:6]):raise RuntimeError('Joint velocities not near zero or unavailable')
names=['j2n6s300_joint_'+str(i) for i in range(1,7)];q=[n.joint.position[n.joint.name.index(k)] for k in names];cmd=JointCommand();cmd.stamp=n.get_clock().now().to_msg();cmd.command_id='codex_filtro_j1_10deg_20261122';cmd.joint_names=names;cmd.positions=q.copy();cmd.positions[0]+=math.radians(10);cmd.duration.sec=2;print('initial',q,'target',list(cmd.positions),'duration_s',2,'proximity_m',n.prox.minimum_clearance,flush=True);n.p.publish(cmd)
end=time.monotonic()+25
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 if n.out and n.out.status in ('SUCCEEDED','FAILED','CANCELED','REJECTED'):break
if n.out is None or n.out.status!='SUCCEEDED':print('Trajectory did not report success. Do not retry automatically.',flush=True)
else:print('final_positions',list(n.joint.position[:6]),flush=True)
n.destroy_node();rclpy.shutdown()
