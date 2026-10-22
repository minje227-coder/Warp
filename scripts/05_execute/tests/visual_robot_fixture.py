"""Run the actual agent main against a virtual clock and simulated ROS IO."""
import importlib.util
import json
import math
import numpy as np
import sys
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from kuavo_execution import first_state as contract

PATH=Path(__file__).resolve().parents[1]/'kuavo_replay.py'


def module():
    spec=importlib.util.spec_from_file_location('virtual_alignment',PATH)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
    return value


class Message:
    def __init__(self):
        self.header=NS(stamp=None)
        self.linear=NS(x=0.,y=0.,z=0.);self.angular=NS(x=0.,y=0.,z=0.)
        self.data=NS(position=[],velocity=[],effort=[])

class Stamp(float):
    def to_sec(self):return float(self)
    def to_nsec(self):return int(self*1e9)



class Simulation:
    def __init__(self,worker,fault=None,arm_mode=0,mpc_mode=3,arm_topic=False):
        self.w=worker;self.now=1.;self.callbacks={};self.state=None;self.tracker=None
        self.q=[0.]*18;self.want=list(self.q);self.v=[0.]*18
        self.claw=[20.,20.];self.claw_want=list(self.claw)
        self.pose=[0.,0.,0.];self.base_want=[0.,0.,0.]
        self.arm_mode=arm_mode;self.mpc_mode=mpc_mode;self.arm_topic=arm_topic;self.quick_mode=0;self.transport=1;self.enabled=True;self.claw_last_command=-1e9;self.arm_mode_at=-1e9
        self.cartesian_commanded=False;self.lower_joint_commanded=False;self.base_goal=None;self.arm_feedback=None;self.next_mode_poll=0.;self.service_calls=[];self.getter_calls=[];self.seed_at_quick=None
        self.published=[];self.outputs=[];self.fault=fault;self.first_joint_motion_base_error=None
        self.closure=None;self.posture_started=False;self.last_frame=None
        origins=[(.093,.40635),(-.46521,.18325),(.48296,.12941),(0.,.25)]
        parents=['base_link','body_0','body_1','body_2'];children=['body_0','body_1','body_2','waist_yaw_link']
        self.urdf='<robot>'+''.join(
            '<joint name="%s" type="revolute"><parent link="%s"/><child link="%s"/><origin xyz="%s 0 %s"/><axis xyz="%s"/><limit lower="-3" upper="3"/></joint>'%
            (n,parents[i],children[i],origins[i][0],origins[i][1],'0 0 1' if i==3 else '0 1 0')
            if i<4 else '<joint name="%s"><limit lower="-3" upper="3"/></joint>'%n
            for i,n in enumerate(contract.NAMES))+'</robot>'
        self.body_model=self.w.TorsoModel(self.urdf)
        self.target=contract.target(dict(initial=dict(q=[.08]*18,joint_names=contract.NAMES,
                       base_pose=[.03,-.02,.02],source_s=3.,grippers=[dict(open_fraction=.7)]*2)))

    def tick(self,dt):
        dt=max(dt,.0001);self.now+=dt
        old=list(self.q)
        if self.fault!='stuck':
            if self.quick_mode in (1,3) or (self.quick_mode==0 and self.mpc_mode in (1,3) and (self.cartesian_commanded or self.lower_joint_commanded)):
                self.q[:4]=[a+(b-a)*min(1,dt/.08) for a,b in zip(self.q[:4],self.want[:4])]
            if self.transport==2 and self.arm_mode==2 and (self.quick_mode==3 or self.now-self.arm_mode_at>=.2):
                self.q[4:]=[a+(b-a)*min(1,dt/.08) for a,b in zip(self.q[4:],self.want[4:])]
        self.v=[(a-b)/dt for a,b in zip(self.q,old)]
        self.claw=[a+(b-a)*min(1,dt/.08) for a,b in zip(self.claw,self.claw_want)]
        if self.base_goal is not None:
            self.base_want=self.w.command(self.w.relative(self.pose,self.base_goal),self.base_want,min(dt,.25))
        x,y,a=self.pose;vx,vy,wz=self.base_want;c,s=math.cos(a),math.sin(a)
        if self.fault!='base_stuck':
            self.pose=[x+(c*vx-s*vy)*dt,y+(s*vx+c*vy)*dt,a+wz*dt]
        for topic,callback in list(self.callbacks.items()):
            if topic=='/mobile_manipulator/lb_mpc_control_mode':callback(NS(data=self.mpc_mode))
            elif topic=='/humanoid/mpc/arm_control_mode' and self.arm_topic:callback(NS(data=[self.arm_mode]))
            elif topic=='/robot_disabled_flag':callback(NS(data=False))
            elif topic=='/enable_control_state':callback(NS(data=self.enabled))
            elif topic=='/ik_debug/arm_traj_receive/transport':
                if self.fault!='transport_missing':callback(NS(data=1 if self.fault=='transport_ack_missing' else self.transport))
            elif topic=='/odom':
                if self.fault=='odom_loss' and self.posture_started:continue
                x,y,yaw=self.pose
                callback(NS(header=NS(stamp=NS(to_nsec=lambda:int(self.now*1e9)),frame_id='odom'),
                    pose=NS(pose=NS(position=NS(x=x,y=y,z=0.),orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))),
                    twist=NS(twist=NS(linear=NS(x=self.base_want[0],y=self.base_want[1]),angular=NS(z=self.base_want[2])))))
            elif topic=='/sensors_data_raw':
                if self.fault=='feedback_loss' and self.posture_started:continue
                callback(NS(joint_data=NS(joint_q=self.q+[2.,-2.],joint_v=self.v+[5.,-5.]),
                            sensor_time=NS(to_nsec=lambda:int(self.now*1e9)),header=NS(stamp=NS(to_nsec=lambda:0))))
            elif topic=='/leju_claw_state':
                callback(NS(header=NS(stamp=Stamp(self.now)),data=NS(name=['left_claw','right_claw'],position=self.claw,velocity=[.01,.01],effort=[1,1]),state=([1,1] if self.fault=='claw_streaming' and self.now-self.claw_last_command<.2 else [2,2])))
        if self.arm_feedback is not None and self.now>=self.next_mode_poll:
            self.arm_feedback.read(self.getter,self.now);self.next_mode_poll=self.now+.5
        if self.tracker is not None:
            if not(self.fault=='lease_loss' and self.posture_started):
                frame=str(int(self.now/.15))
                try:
                    self.tracker.observation(dict(error=self.w.relative(self.pose,self.target['base_pose']),
                                                  age_s=0,frame_id=frame,pose=list(self.pose)),self.now)
                except ValueError as exc:
                    # The real stdin receiver reports malformed vision and stops.
                    self.state['failure']=str(exc);self.closure['done'].set()
        if self.fault=='base_stuck' and self.now>15 and self.closure:
            self.closure['done'].set()
        if self.fault=='stdin_failure' and self.posture_started:
            self.state['failure']='랙 위치 추정이 갑자기 바뀌어 정지합니다'
            self.closure['done'].set()
        if self.fault=='stop' and self.posture_started:
            self.closure['done'].set()
        if self.fault=='foreign' and self.posture_started:
            self.callbacks['/lb_leg_traj'](NS(_connection_header={'callerid':'/other'}))

    def getter(self,control_mode):
        self.getter_calls.append((self.now,control_mode))
        if self.fault=='getter_missing' or (self.fault=='getter_loss' and self.posture_started):
            raise RuntimeError('read-only getter unavailable')
        return NS(mode=self.arm_mode)

    def fake_modules(self):
        sim=self
        class Publisher:
            def __init__(self,topic,*args,**kwargs):self.topic=topic
            def get_num_connections(self):return 1
            def publish(self,msg):
                sim.published.append((sim.now,self.topic,msg))
                if self.topic=='/cmd_vel':
                    sim.base_want=[msg.linear.x,msg.linear.y,msg.angular.z];sim.base_goal=None
                elif self.topic=='/cmd_pose':
                    x,y,a=sim.pose;c,si=math.cos(a),math.sin(a)
                    sim.base_goal=[x+c*msg.linear.x-si*msg.linear.y,y+si*msg.linear.x+c*msg.linear.y,a+msg.angular.z]
                    if sim.fault!='pose_ack_missing' and '/lb_cmd_pose_reach_time' in sim.callbacks:
                        sim.callbacks['/lb_cmd_pose_reach_time'](NS(data=1.))
                elif self.topic=='/cmd_lb_torso_pose':
                    sim.cartesian_commanded=True
                    wanted=np.array([msg.linear.x,msg.linear.z,msg.angular.y])
                    q=np.array(sim.want[:4]);q[3]=msg.angular.z
                    for _ in range(12):
                        actual=np.array(sim.body_model.position(q)[:3]);error=wanted-actual
                        if np.max(np.abs(error))<1e-11:break
                        jac=np.column_stack([(np.array(sim.body_model.position(q+np.eye(4)[i]*1e-6)[:3])-actual)/1e-6 for i in range(3)])
                        q[:3]+=np.linalg.solve(jac,error)
                    if sim.first_joint_motion_base_error is None and max(map(abs,q))>1e-4:
                        sim.first_joint_motion_base_error=sim.w.relative(sim.pose,sim.target['base_pose'])
                    sim.posture_started=True;sim.want[:4]=q.tolist()
                elif self.topic=='/lb_leg_traj':
                    # The native MPC reference manager also subscribes to this topic.
                    sim.lower_joint_commanded=True
                    if sim.first_joint_motion_base_error is None and any(abs(x)>1e-4 for x in msg.position):
                        sim.first_joint_motion_base_error=sim.w.relative(sim.pose,sim.target['base_pose'])
                    sim.posture_started=True;sim.want[:4]=[math.radians(x) for x in msg.position]
                elif self.topic=='/kuavo_arm_traj' and sim.transport==2:sim.want[4:]=[math.radians(x) for x in msg.position]
                elif self.topic=='/leju_claw_command':
                    sim.claw_want=list(msg.data.position);sim.claw_last_command=sim.now
        def subscriber(topic,typ,callback,**kwargs):sim.callbacks[topic]=callback
        def service(name,typ):
            def call(**payload):
                if name=='/humanoid_get_arm_ctrl_mode':return sim.getter(**payload)
                sim.service_calls.append((name,payload))
                if name=='/mobile_manipulator_mpc_control':
                    if sim.fault=='mpc_reject':return NS(result=False)
                    sim.mpc_mode=payload['control_mode'];return NS(result=True)
                if name==sim.w.ARM_LINK_SERVICE:
                    if sim.fault=='transport_reject':return NS(success=False)
                    sim.transport=payload['transport'];return NS(success=True)
                if name=='/enable_lb_arm_quick_mode':
                    if sim.fault=='quick_reject':return NS(success=False)
                    sim.seed_at_quick=(list(sim.want),list(sim.q))
                    sim.quick_mode=payload['quickMode'];return NS(success=True)
                if sim.fault!='arm_ack_missing':sim.arm_mode=payload['control_mode'];sim.arm_mode_at=sim.now
                return NS(result=sim.fault!='arm_reject')
            return call
        ros=NS(init_node=lambda *a,**kw:None,Subscriber=subscriber,Publisher=Publisher,
               get_name=lambda:'/ours',AnyMsg=Message,ROSException=RuntimeError,
               wait_for_service=lambda *a,**kw:None,ServiceProxy=service,is_shutdown=lambda:False,
               get_param=lambda *a:sim.urdf,
               Time=NS(now=lambda:Stamp(self.now)))
        return {'rospy':ros,'geometry_msgs.msg':NS(Twist=Message),'nav_msgs.msg':NS(Odometry=Message),'sensor_msgs.msg':NS(JointState=Message),
                'std_msgs.msg':NS(Bool=Message,Int8=Message,Float32=Message,Float64=Message,Float64MultiArray=Message),
                'kuavo_msgs.msg':NS(sensorsData=Message,lejuClawState=Message,lejuClawCommand=Message),
                'kuavo_msgs.srv':NS(changeArmCtrlMode=Message,changeTorsoCtrlMode=Message,SetIncrementalArmTrajLink=type('LinkService',(Message,),dict(_md5sum=sim.w.ARM_LINK_MD5)),changeLbQuickModeSrv=type("QuickService",(Message,),dict(_md5sum=sim.w.QUICK_SERVICE_MD5)))}

    def run(self):
        sim=self
        class Event:
            def __init__(self):self.value=False
            def set(self):self.value=True
            def is_set(self):return self.value
            def wait(self,seconds):sim.tick(seconds);return self.value
        class Thread:
            def __init__(self,target,**kw):self.target=target
            def start(self):
                # Replace only the stdin transport, not the real driver loop.
                scope=dict(zip(self.target.__code__.co_freevars,[x.cell_contents for x in self.target.__closure__]))
                if self.target.__name__=='read_arm_mode':
                    sim.arm_feedback=scope['arm_feedback']
                else:
                    sim.closure=scope;sim.state=scope['state'];sim.tracker=scope['tracker']
                    sim.state['target']=sim.target
                sim.tick(.001)
        def monotonic():
            sim.now+=.00001
            return sim.now
        fake_time=NS(monotonic=monotonic,sleep=lambda dt:sim.tick(dt))
        with patch.dict(sys.modules,self.fake_modules()),patch.object(self.w,'time',fake_time), \
             patch.object(self.w.threading,'Event',Event),patch.object(self.w.threading,'Thread',Thread), \
             patch.object(self.w.signal,'signal'),patch.object(self.w,'emit',side_effect=lambda **kw:sim.outputs.append(kw)):
            return self.w.main()
