#!/usr/bin/env python3
"""Warp KUAVO execution: RWH topic preparation and current-clock joint replay.

Importing this module is ROS-free for deterministic simulation tests. The live
entry point starts only after an explicit dashboard Start POST. EOF, stale
vision, a foreign motion command, disable, stop or SIGTERM stops it.
"""
import bisect
import collections
import hashlib
import json
import math
import signal
import sys
import threading
import time

LINEAR_MAX=.05
ANGULAR_MAX=math.radians(3)
LINEAR_ACCEL=.04
ANGULAR_ACCEL=math.radians(3)
POSITION_TOL=.02
YAW_TOL=math.radians(2)
MAX_DISTANCE=.75
MAX_YAW=math.radians(45)
LEASE_S=1.0
VISION_S=1.5
TIMEOUT_S=120
NAMES=['knee_joint','leg_joint','waist_pitch_joint','waist_yaw_joint']+[
    'zarm_%s%d_joint'%(s,i) for s in 'lr' for i in range(1,8)]
# Direct torso+arm joint replay contract. quickMode selects which joint targets
# override MPC output; it does not set the preparation or replay speed.
JOINT_CONTROL_MODES=dict(mpc=3,arm=2,quick=3)
# Bootstrap with the normal arm mode-switch path. Only after ROS input has
# been selected and primed do preparation/replay select direct quickMode 3.
PREPARATION_CONTROL_MODES=dict(mpc=3,arm=2,quick=0)
EXECUTOR_VERSION='warp-rwh-topics-v1'
QUICK_SERVICE_MD5='8f6c979af573879a9cb6f5c0c50c2be4'
ARM_LINK_SERVICE='/humanoid_wheel/set_incremental_arm_traj_link'
ARM_LINK_MD5='c50718844f9433e81f666d6ef58a2c25'


def quick_service_type():
    """Use installed bindings or genpy's generator for the exact deployed .srv.

    The rwh overlay contains arm/torso services but omits this generated Python
    class, although the controller advertises the service. Do not change ROS
    workspaces or install packages; generate only this primitive-only binding.
    """
    try:
        from kuavo_msgs.srv import changeLbQuickModeSrv
        result=changeLbQuickModeSrv
    except ImportError:
        from genpy.dynamic import generate_dynamic
        request='kuavo_msgs/changeLbQuickModeSrvRequest'
        response='kuavo_msgs/changeLbQuickModeSrvResponse'
        req=generate_dynamic(request,'int8 quickMode\n')[request]
        resp=generate_dynamic(response,'bool success\nstring message\n')[response]
        result=type('changeLbQuickModeSrv',(object,),dict(
            _type='kuavo_msgs/changeLbQuickModeSrv',_md5sum=QUICK_SERVICE_MD5,
            _request_class=req,_response_class=resp))
    if getattr(result,'_md5sum',None)!=QUICK_SERVICE_MD5:
        raise ValueError('quickMode 서비스 정의가 컨트롤러 계약과 다릅니다.')
    return result


def arm_link_service_type():
    """Match the deployed receiver service without changing ROS workspaces."""
    try:
        from kuavo_msgs.srv import SetIncrementalArmTrajLink
        result=SetIncrementalArmTrajLink
    except ImportError:
        from genpy.dynamic import generate_dynamic
        request='kuavo_msgs/SetIncrementalArmTrajLinkRequest'
        response='kuavo_msgs/SetIncrementalArmTrajLinkResponse'
        req=generate_dynamic(request,'int8 TRANSPORT_NONE=0\nint8 TRANSPORT_SHM=1\nint8 TRANSPORT_KUAVO_ARM_TRAJ=2\nint8 transport\n')[request]
        resp=generate_dynamic(response,'bool success\nstring message\n')[response]
        result=type('SetIncrementalArmTrajLink',(object,),dict(
            _type='kuavo_msgs/SetIncrementalArmTrajLink',_md5sum=ARM_LINK_MD5,
            _request_class=req,_response_class=resp))
    if getattr(result,'_md5sum',None)!=ARM_LINK_MD5:
        raise ValueError('팔 입력 경로 서비스 정의가 컨트롤러 계약과 다릅니다.')
    return result


def check_runtime():
    """Import/serialize/type-handshake only. No ROS node, publisher or RPC call."""
    import io
    import rosservice
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool,Int8
    from kuavo_msgs.msg import sensorsData,lejuClawState,lejuClawCommand
    from kuavo_msgs.srv import changeArmCtrlMode,changeTorsoCtrlMode
    quick=quick_service_type();arm_link=arm_link_service_type()
    claw=lejuClawCommand();claw.data.name=['left_claw','right_claw'];claw.data.position=[20.,20.]
    claw.data.velocity=[50.,50.];claw.data.effort=[1.,1.]
    claw.serialize(io.BytesIO())
    data=io.BytesIO();quick._request_class(quickMode=3).serialize(data)
    if data.getvalue()!=b'\x03':raise ValueError('quickMode 요청 직렬화 오류')
    data=io.BytesIO();quick._response_class(success=True,message='binding-check').serialize(data)
    response=quick._response_class().deserialize(data.getvalue())
    if response.success is not True or response.message!='binding-check':raise ValueError('quickMode 응답 직렬화 오류')
    data=io.BytesIO();arm_link._request_class(transport=2).serialize(data)
    if data.getvalue()!=b'\x02':raise ValueError('팔 ROS 입력 경로 요청 직렬화 오류')
    data=io.BytesIO();arm_link._response_class(success=True,message='binding-check').serialize(data)
    response=arm_link._response_class().deserialize(data.getvalue())
    if response.success is not True:raise ValueError('팔 입력 경로 응답 직렬화 오류')
    headers={}
    for name,typ in [('/humanoid_get_arm_ctrl_mode',changeArmCtrlMode),
                     ('/wheel_arm_change_arm_ctrl_mode',changeArmCtrlMode),
                     ('/mobile_manipulator_mpc_control',changeTorsoCtrlMode),
                     ('/enable_lb_arm_quick_mode',quick),
                     (ARM_LINK_SERVICE,arm_link)]:
        header=rosservice.get_service_headers(name,rosservice.get_service_uri(name))
        if header.get('md5sum')!=typ._md5sum:raise ValueError('서비스 타입 불일치: '+name)
        headers[name]=dict(type=header['type'],md5sum=header['md5sum'])
    TorsoModel(rospy.get_param('/robot_description'))
    pose=Twist();pose.linear.x=.01;pose.linear.z=1.2;pose.angular.y=.1;pose.angular.z=.2
    pose.serialize(io.BytesIO())
    emit(runtime_check=True,ok=True,services=headers,torso_model_ok=True,publishers_created=0,service_calls=0)
    return 0


class ArmModeFeedback:
    """Read-only service feedback; the arm topic is change-only, not a heartbeat."""
    def __init__(self):
        self.sample=None
        self.error='팔 모드 조회 서비스 응답 없음'

    def read(self,getter,now):
        try:
            mode=getter(control_mode=0).mode  # GET service ignores request value.
            # The deployed .srv also defines 3 = ik_ultra_fast_mode. It is a
            # valid prior mode, but direct JointState execution must select 2.
            if isinstance(mode,bool) or mode not in (0,1,2,3):
                raise ValueError('지원하지 않는 팔 제어 모드: %r'%mode)
            self.sample=(int(mode),now);self.error=''
            return True
        except Exception as exc:
            self.error=str(exc)
            return False

    def current(self,now):
        sample=self.sample
        if sample is None or not 0<=now-sample[1]<2.:
            return None
        return sample[0]


def finite_vector(values,n):
    if (not isinstance(values,list) or len(values)!=n or
        any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values)):
        raise ValueError('잘못된 첫 자세/실측 관절 입력')
    return list(map(float,values))



class TorsoModel:
    """S63 planar FK from the current ROS URDF; inputs and outputs use radians."""
    def __init__(self,xml):
        import xml.etree.ElementTree as ET
        root=ET.fromstring(xml)
        by_child={j.find('child').get('link'):j for j in root.findall('joint') if j.find('child') is not None}
        chain=[];child='waist_yaw_link';seen=set()
        while child!='base_link':
            if child in seen or child not in by_child:raise ValueError('몸통 URDF 연결 오류')
            seen.add(child);joint=by_child[child];chain.append(joint);child=joint.find('parent').get('link')
        chain.reverse();self.steps=[];names=[]
        for joint in chain:
            origin=joint.find('origin')
            xyz=[float(x) for x in (origin.get('xyz','0 0 0') if origin is not None else '0 0 0').split()]
            rpy=[float(x) for x in (origin.get('rpy','0 0 0') if origin is not None else '0 0 0').split()]
            if len(xyz)!=3 or len(rpy)!=3 or not all(math.isfinite(x) for x in xyz+rpy) or abs(xyz[1])>1e-9 or max(map(abs,rpy))>1e-9:
                raise ValueError('RWH 평면 몸통과 다른 URDF입니다.')
            index=None
            if joint.get('type')!='fixed':
                index=len(names);names.append(joint.get('name'))
                axis=joint.find('axis')
                values=[float(x) for x in (axis.get('xyz') if axis is not None else '').split()]
                if values!=([0.,0.,1.] if index==3 else [0.,1.,0.]):raise ValueError('몸통 관절 축 오류')
            self.steps.append((xyz[0],xyz[2],index))
        if names!=NAMES[:4]:raise ValueError('몸통 관절 순서 오류')

    def position(self,q):
        q=finite_vector(list(q),4);x=z=angle=0.
        for dx,dz,index in self.steps:
            c,si=math.cos(angle),math.sin(angle)
            x+=c*dx+si*dz;z+=-si*dx+c*dz
            if index is not None and index<3:angle+=q[index]
        return [x,z,angle,q[3]]


def claw_feedback(position,velocity,effort,states,names,stamp,ros_now):
    """One fresh percent-valued hardware sample, never the empty radian family."""
    p=list(position);v=list(velocity);e=list(effort)
    if list(names)!=['left_claw','right_claw'] or len(p)!=2 or len(v)!=2 or len(e)!=2:return None
    if not all(math.isfinite(float(x)) for x in p+v+e):return None
    if all(x==0 for x in v+e) and (any(x<-.01 for x in p) or all(x==0 for x in p)):return None
    if not math.isfinite(stamp) or stamp<=0 or not -.1<=ros_now-stamp<=.5:return None
    p=finite_vector(p,2)
    if any(not 0<=x<=100 for x in p):raise ValueError('실측 그리퍼 단위/범위 오류')
    if len(states)!=2 or any(x not in (-1,0,1,2,3) for x in states):raise ValueError('실측 그리퍼 상태 오류')
    if any(x<0 for x in states):raise ValueError('그리퍼 하드웨어 오류')
    return p,list(states)


def next_replay_wait(began,now,period=.01):
    """Wait to a future clock boundary; never flush a backlog of old samples."""
    slot=math.floor(max(0.,now-began)/period+1e-9)+1
    return max(.0001,began+slot*period-now)


class Stationarity:
    """Arrival-only feedback statistics; never filters or changes the target."""
    def __init__(self):
        self.rows=collections.deque();self.span_deg=None;self.drift_deg_s=None;self.rms_deg_s=None

    def update(self,q,v,now):
        q=finite_vector(q,18);v=finite_vector(v,18)
        if self.rows and now<self.rows[-1][0]:self.rows.clear()
        if self.rows and now==self.rows[-1][0]:self.rows.pop()
        self.rows.append((now,q,v))
        while len(self.rows)>2 and self.rows[1][0]<now-.6:self.rows.popleft()
        duration=now-self.rows[0][0]
        if len(self.rows)<6 or duration<.5:return False
        self.span_deg=max(math.degrees(max(row[1][i] for row in self.rows)-min(row[1][i] for row in self.rows)) for i in range(18))
        self.drift_deg_s=max(math.degrees(abs(q[i]-self.rows[0][1][i]))/duration for i in range(18))
        self.rms_deg_s=max(math.degrees(math.sqrt(sum(row[2][i]**2 for row in self.rows)/len(self.rows))) for i in range(18))
        return self.span_deg<=.3 and self.drift_deg_s<=.3 and self.rms_deg_s<=2.

CLAW_POSITION_TOL=8.  # Same bound as first_state.feedback_matches, plus hardware done state.


class JointBridge:
    """Slow torso quintic and init_pose-style three-second arm smoothstep.

    Only the 18 torso/arm joints are interpolated. Actual feedback, not elapsed
    command time, decides arrival. Head is excluded from commands and arrival.
    """
    def __init__(self,start,target):
        self.start=finite_vector(start,18)
        if target.get('joint_names')!=NAMES:raise ValueError('첫 자세 관절 순서 불일치')
        self.goal=finite_vector(target['q'],18)
        self.claw=finite_vector(target['claw'],2)
        if any(not 0<=x<=100 for x in self.claw):raise ValueError('그리퍼 목표 범위 오류')
        speed=finite_vector(target['limits']['velocity'],18)
        accel=finite_vector(target['limits']['acceleration'],18)
        if any(x<=0 for x in speed+accel):raise ValueError('속도·가속도 제한 오류')
        speed[:4]=[min(x,math.radians(2)) for x in speed[:4]]
        accel[:4]=[min(x,math.radians(4)) for x in accel[:4]]
        delta=[abs(b-a) for a,b in zip(self.start,self.goal)]
        self.arm_duration=3.0
        self.body_duration=max([2.]+[1.875*delta[i]/speed[i] for i in range(4)]+
                              [math.sqrt(10/math.sqrt(3)*delta[i]/accel[i]) for i in range(4)])
        for i in range(4,18):
            if 1.5*delta[i]/self.arm_duration>speed[i] or 6*delta[i]/self.arm_duration**2>accel[i]:
                raise ValueError('팔 3초 이동이 속도·가속도 한계를 넘습니다: '+NAMES[i])
        self.duration=max(self.body_duration,self.arm_duration)
        if self.duration>120:raise ValueError('첫 자세 연결 시간이 120초를 넘습니다.')
        self.settled=None;self.still=Stationarity()

    def position(self,elapsed):
        u=max(0.,min(1.,elapsed/self.body_duration));body=u*u*u*(10+u*(-15+6*u))
        u=max(0.,min(1.,elapsed/self.arm_duration));arm=u*u*(3-2*u)
        return [a+(b-a)*(body if i<4 else arm) for i,(a,b) in enumerate(zip(self.start,self.goal))]

    def arrived(self,q,velocity,claw,claw_state,now):
        q=finite_vector(q,18);velocity=finite_vector(velocity,18);claw=finite_vector(claw,2)
        close=(max(abs(a-b) for a,b in zip(q,self.goal))<=math.radians(1.5)
               and self.still.update(q,velocity,now)
               and max(abs(a-b) for a,b in zip(claw,self.claw))<=CLAW_POSITION_TOL
               and len(claw_state)==2 and all(s in (2,3) for s in claw_state))
        if not close:self.settled=None;return False
        if self.settled is None:self.settled=now
        return now-self.settled>=1.


def joint_commands(q):
    """The wheel SDK publishes BOTH /lb_leg_traj and /kuavo_arm_traj in degrees.

    SDK control.py::control_wheel_lower_joint, pub_wheel_lower_joint_cmd and
    case_wheel_test_torso_joint.py are the contract, rather than JointState's
    conventional SI unit assumption. Never send sensor radians unchanged.
    """
    values=[math.degrees(x) for x in finite_vector(q,18)]
    return values[:4],values[4:]


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


def relative(p,goal):
    c,s=math.cos(p[2]),math.sin(p[2]);dx,dy=goal[0]-p[0],goal[1]-p[1]
    return [c*dx+s*dy,-s*dx+c*dy,wrap(goal[2]-p[2])]


def norm_limit(x,y,limit):
    scale=min(1,limit/max(math.hypot(x,y),1e-12))
    return x*scale,y*scale


def command(error,last,dt):
    """Proportional, braking-distance and vector acceleration limits in SI units."""
    if not all(math.isfinite(v) for v in [*error,*last,dt]) or not 0<dt<=.25:
        raise ValueError("잘못된 제어 입력/주기")
    x,y,a=error;distance=math.hypot(x,y)
    vmax=min(LINEAR_MAX,math.sqrt(2*LINEAR_ACCEL*max(0,distance-.006)))
    vx,vy=norm_limit(.6*x,.6*y,vmax)
    if distance<.008:vx=vy=0
    angular=min(ANGULAR_MAX,math.sqrt(2*ANGULAR_ACCEL*max(0,abs(a)-math.radians(.3))))
    wz=max(-angular,min(angular,.65*a))
    if abs(a)<math.radians(.4):wz=0
    dx,dy=norm_limit(vx-last[0],vy-last[1],LINEAR_ACCEL*dt)
    dw=max(-ANGULAR_ACCEL*dt,min(ANGULAR_ACCEL*dt,wz-last[2]))
    return [last[0]+dx,last[1]+dy,last[2]+dw]


class ReplayPath:
    """ROS-free 1x sampler of the immutable final execution P/V/A columns.

    No preparation limits, rescaling or source-video clock is applied here.
    JointState has no acceleration field: acceleration is retained for checking
    and telemetry; positions and velocities are sent at their execution times.
    """
    def __init__(self, data):
        self.data=data
        if data.get('names')!=['base_x','base_y','base_yaw']+NAMES:
            raise ValueError('replay 좌표 순서 불일치')
        self.t=finite_vector(data.get('execution_s'),len(data.get('execution_s',[])))
        if (len(self.t)<2 or len(self.t)>100000 or any(not 0<b-a<=.021 for a,b in zip(self.t,self.t[1:]))):
            raise ValueError('replay 실행 시각/100Hz 표본 오류')
        self.p=[];self.v=[];self.a=[];self.claw=[]
        self.vmax=finite_vector(data['velocity_limits'],21)
        self.amax=finite_vector(data['acceleration_limits'],21)
        if min(self.vmax+self.amax)<=0:raise ValueError('replay 제한값 오류')
        for key,out,n in [('position',self.p,21),('velocity',self.v,21),('acceleration',self.a,21),('claw',self.claw,2)]:
            rows=data.get(key)
            if not isinstance(rows,list) or len(rows)!=len(self.t):raise ValueError('replay 표본 수 불일치: '+key)
            out.extend(finite_vector(row,n) for row in rows)
        for rows,limits in [(self.v,self.vmax),(self.a,self.amax)]:
            if any(abs(x)>lim+1e-6 for row in rows for x,lim in zip(row,limits)):
                raise ValueError('replay 속도·가속도 제한 초과')
        if any(not 0<=v<=100 for row in self.claw for v in row):raise ValueError('replay 그리퍼 범위 오류')
        self.begin,self.end=finite_vector(data.get('interval'),2)
        if not self.t[0]<=self.begin<self.end<=self.t[-1]:raise ValueError('replay 선택 구간 오류')
        if any(abs(v)>1e-5 for edge in (self.begin,self.end) for v in self.sample(edge)[1]):
            raise ValueError('선택 경계가 이동 중입니다. 1배속을 유지하려면 정지 지점으로 시작·끝을 선택하세요.')
        self.duration=self.end-self.begin
        source=finite_vector(data.get('source_s'),len(self.t))
        if any(b<a for a,b in zip(source,source[1:])):raise ValueError('replay 원본 시각 순서 오류')
        body={k:v for k,v in data.items() if k!='payload_sha256'}
        actual=hashlib.sha256(json.dumps(body,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
        if data.get('payload_sha256')!=actual:raise ValueError('replay 전송 궤적 해시 불일치')

    def sample(self, execution_s):
        t=min(self.t[-1],max(self.t[0],execution_s))
        j=max(0,min(len(self.t)-2,bisect.bisect_right(self.t,t)-1))
        u=(t-self.t[j])/(self.t[j+1]-self.t[j])
        return tuple([a+(b-a)*u for a,b in zip(rows[j],rows[j+1])] for rows in (self.p,self.v,self.a,self.claw))

    def elapsed(self, elapsed):
        return self.sample(self.begin+min(self.duration,max(0.,elapsed)))


def predict_pose(pose, stamp, now, history):
    """Advance the camera-time rack pose with commands since that exposure.

    This is a short command-based prediction, never substituted for fresh rack
    observations. The independent vision lease still expires normally.
    """
    p=list(pose);prev=stamp;velocity=[0.,0.,0.]
    for t,v in history:
        if t<=stamp:velocity=v;continue
        if t>now:break
        dt=t-prev;p=[x+y*dt for x,y in zip(p,velocity)];prev=t;velocity=v
    return [x+y*max(0.,now-prev) for x,y in zip(p,velocity)]


def replay_base(p,v,pose,last,dt,vmax,amax):
    error=[p[0]-pose[0],p[1]-pose[1],wrap(p[2]-pose[2])]
    if math.hypot(*error[:2])>.10 or abs(error[2])>math.radians(8):
        raise ValueError('replay 베이스 추종 오차가 10cm·8°를 넘었습니다.')
    world=[]
    for i in range(3):
        want=max(-vmax[i],min(vmax[i],v[i]+.6*error[i]))
        world.append(last[i]+max(-amax[i]*dt,min(amax[i]*dt,want-last[i])))
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return [c*world[0]+s*world[1],-s*world[0]+c*world[1],world[2]],world,error


class Tracker:
    def __init__(self):
        self.samples=collections.deque()
        self.replaying=False;self.pose=None
        self.ready=False;self.frame=None;self.vision_at=None;self.input_at=None
        self.visual=None;self.good_frames=collections.deque(maxlen=6);self.first_input_at=None
        self.last=[0.,0.,0.];self.settled_at=None;self.travel=0.

    def observation(self,data,now):
        error=data.get("error");age=data.get("age_s")
        if not isinstance(error,list) or len(error)!=3:
            raise ValueError("base_link 기준 정렬 오차 없음")
        if not isinstance(age,(int,float)) or not all(math.isfinite(v) for v in error+[age]) or not 0<=age<VISION_S:
            raise ValueError("정렬 영상 입력이 오래됐습니다")
        if not data.get('frame_id'):raise ValueError('영상 표본 ID 없음')
        self.input_at=now
        if self.first_input_at is None:self.first_input_at=now
        if data['frame_id']==self.frame:return
        if self.ready and self.visual is not None and not self.replaying:
            if math.hypot(error[0]-self.visual[0],error[1]-self.visual[1])>.08 or abs(wrap(error[2]-self.visual[2]))>math.radians(8):
                raise ValueError("랙 위치 추정이 갑자기 바뀌어 정지합니다")
        self.frame=data['frame_id'];self.vision_at=now-age;self.visual=error
        if self.replaying:self.pose=finite_vector(data.get('pose'),3)
        self.samples.append((now,error))
        # Keep at least four distinct frames: at the live ~2 Hz camera rate,
        # four frames span slightly more than 1.5 s. Dropping the oldest frame
        # at that boundary made startup stability impossible for some phases.
        while len(self.samples)>4 and now-self.samples[0][0]>1.5:self.samples.popleft()
        if not self.ready and len(self.samples)>=4 and self.samples[-1][0]-self.samples[0][0]>=1.1:
            values=[v for _,v in self.samples]
            mean=[sum(v[i] for v in values)/len(values) for i in (0,1)]
            yaw=math.atan2(sum(math.sin(v[2]) for v in values),sum(math.cos(v[2]) for v in values))
            if any(math.hypot(v[0]-mean[0],v[1]-mean[1])>.035 or abs(wrap(v[2]-yaw))>math.radians(3) for v in values):return
            if math.hypot(*error[:2])>MAX_DISTANCE or abs(error[2])>MAX_YAW:
                raise ValueError("자동 정렬 범위 75cm·45°를 넘습니다")
            self.ready=True
        if math.hypot(*error[:2])<=POSITION_TOL and abs(error[2])<=YAW_TOL:
            self.good_frames.append(now-age)
        else:self.good_frames.clear()

    def step(self,now,dt):
        if not all(math.isfinite(v) for v in [now,dt]) or not 0<dt<=.25:
            raise ValueError('유효하지 않은 제어 주기')
        if self.input_at is None or now-self.input_at>LEASE_S:
            raise ValueError("대시보드 제어 연결이 끊겨 정지합니다")
        if self.vision_at is None or now-self.vision_at>VISION_S:
            raise ValueError("랙 영상이 끊겨 정지합니다")
        if self.first_input_at is not None and now-self.first_input_at>TIMEOUT_S:
            raise ValueError('정렬 제한 시간 초과')
        if not self.ready:return [0.,0.,0.],False,"랙 위치 안정화 중"
        e=self.visual
        if self.travel>MAX_DISTANCE+.15 or math.hypot(*e[:2])>MAX_DISTANCE+.15:
            raise ValueError("예상 정렬 이동 범위를 벗어나 정지합니다")
        close=math.hypot(*e[:2])<=POSITION_TOL and abs(e[2])<=YAW_TOL
        visual_good=sum(now-t<VISION_S for t in self.good_frames)>=3
        if close:
            self.last=[0.,0.,0.]
            if self.settled_at is None:self.settled_at=now
            return self.last,visual_good and now-self.settled_at>=1.,"도착·정지 확인 중"
        self.settled_at=None
        self.last=command(e,self.last,dt)
        self.travel+=math.hypot(*self.last[:2])*dt
        return self.last,False,"천천히 정렬 중"


class SnapshotTracker(Tracker):
    """Freeze rack goal before any motion, then measure travel in local odom."""
    def __init__(self):
        super().__init__()
        self.frozen=False;self.goal=None;self.odom_pose=None;self.odom_v=None
        self.odom_at=None;self.odom_stamp=None;self.odom_frame=None

    def observation(self,data,now):
        if self.frozen:self.input_at=now;return
        super().observation(data,now)

    def heartbeat(self,now):
        if self.frozen:self.input_at=now

    def odometry(self,pose,velocity,stamp,frame,now):
        pose=finite_vector(pose,3);velocity=finite_vector(velocity,3)
        if not isinstance(stamp,int) or stamp<=0:return
        if self.odom_stamp is not None and stamp<=self.odom_stamp:return
        if self.frozen and self.odom_pose is not None:
            dt=now-self.odom_at
            if frame!=self.odom_frame:raise ValueError('이동 중 odom 좌표계가 바뀌었습니다.')
            if (math.hypot(pose[0]-self.odom_pose[0],pose[1]-self.odom_pose[1])>.03+2*LINEAR_MAX*dt
                or abs(wrap(pose[2]-self.odom_pose[2]))>math.radians(3)+2*ANGULAR_MAX*dt):
                raise ValueError('이동 중 odom 불연속/리셋을 감지했습니다.')
        self.odom_pose=pose;self.odom_v=velocity;self.odom_at=now
        self.odom_stamp=stamp;self.odom_frame=frame

    def freeze(self,now):
        if not self.ready or self.vision_at is None or now-self.vision_at>VISION_S:
            raise ValueError('초기 랙 위치를 안정적으로 확인하지 못했습니다.')
        if self.odom_at is None or now-self.odom_at>.3:raise ValueError('초기 상대 이동용 odom 피드백 없음')
        if math.hypot(*self.odom_v[:2])>.02 or abs(self.odom_v[2])>math.radians(1):
            raise ValueError('베이스가 정지한 뒤 초기 위치를 확정하세요.')
        samples=[v for _,v in self.samples]
        ex=sum(v[0] for v in samples)/len(samples);ey=sum(v[1] for v in samples)/len(samples)
        ea=math.atan2(sum(math.sin(v[2]) for v in samples),sum(math.cos(v[2]) for v in samples))
        x,y,a=self.odom_pose;c,si=math.cos(a),math.sin(a)
        self.goal=[x+c*ex-si*ey,y+si*ex+c*ey,wrap(a+ea)]
        self.initial_error=[ex,ey,ea];self.frozen=True;self.last=[0.,0.,0.]
        return list(self.goal)

    def step(self,now,dt):
        if self.replaying or not self.frozen:return super().step(now,dt)
        if not 0<dt<=.25:raise ValueError('유효하지 않은 제어 주기')
        if self.input_at is None or now-self.input_at>LEASE_S:raise ValueError('대시보드 제어 연결이 끊겨 정지합니다')
        if self.odom_at is None or now-self.odom_at>.3:raise ValueError('상대 이동 odom 피드백이 끊겼습니다.')
        if now-self.first_input_at>TIMEOUT_S:raise ValueError('정렬 제한 시간 초과')
        e=relative(self.odom_pose,self.goal);self.visual=e
        if self.travel>MAX_DISTANCE+.15 or math.hypot(*e[:2])>MAX_DISTANCE+.15:
            raise ValueError('고정한 상대 이동 범위를 벗어났습니다.')
        self.last=command(e,self.last,dt);self.travel+=math.hypot(*self.last[:2])*dt
        close=(math.hypot(*e[:2])<=POSITION_TOL and abs(e[2])<=YAW_TOL
               and math.hypot(*self.odom_v[:2])<=.005 and abs(self.odom_v[2])<=math.radians(.5)
               and math.hypot(*self.last[:2])<=.002 and abs(self.last[2])<=math.radians(.2))
        if not close:self.settled_at=None
        elif self.settled_at is None:self.settled_at=now
        return self.last,close and now-self.settled_at>=1.,'초기 고정 목표로 이동 · odom 도착 확인'


    def pose_arrival(self,now):
        """Read-only arrival test for the single /cmd_pose already sent."""
        if not self.frozen:raise ValueError('고정된 베이스 목표가 없습니다.')
        if self.input_at is None or now-self.input_at>LEASE_S:raise ValueError('대시보드 제어 연결 소실')
        if self.odom_at is None or now-self.odom_at>.3:raise ValueError('상대 이동 odom 피드백 소실')
        if now-self.first_input_at>TIMEOUT_S:raise ValueError('정렬 제한 시간 초과')
        self.visual=relative(self.odom_pose,self.goal)
        if math.hypot(*self.visual[:2])>MAX_DISTANCE+.15 or abs(self.visual[2])>MAX_YAW+math.radians(10):
            raise ValueError('고정한 상대 이동 범위를 벗어났습니다.')
        close=(math.hypot(*self.visual[:2])<=POSITION_TOL and abs(self.visual[2])<=YAW_TOL
               and math.hypot(*self.odom_v[:2])<=.005 and abs(self.odom_v[2])<=math.radians(.5))
        if not close:self.settled_at=None
        elif self.settled_at is None:self.settled_at=now
        return close and now-self.settled_at>=1.



def emit(**data):
    sys.stdout.write(json.dumps(data,ensure_ascii=False,separators=(",",":"))+"\n");sys.stdout.flush()


def main():
    import xml.etree.ElementTree as ET
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool,Int8,Float64,Float32
    from kuavo_msgs.msg import sensorsData,lejuClawState,lejuClawCommand
    from kuavo_msgs.srv import changeArmCtrlMode,changeTorsoCtrlMode
    changeLbQuickModeSrv=quick_service_type();SetIncrementalArmTrajLink=arm_link_service_type()
    rospy.init_node('kuavo_dashboard_retargeting_align',anonymous=True,disable_signals=True)
    done=threading.Event();lock=threading.RLock();tracker=SnapshotTracker();arm_feedback=ArmModeFeedback()
    state=dict(mode=None,mode_at=0,disabled=None,enabled=None,transport=None,transport_at=0,failure=None,target=None,
               replay=None,q=None,v=None,q_at=0,claw=None,claw_state=None,claw_at=0,claw_stamp=0,reach=None,stop_reason='정렬 시작/이동 취소')
    def stop_signal(number,*_):
        with lock:state['stop_reason']='실행기 종료 신호: %s'%number
        done.set()
    signal.signal(signal.SIGTERM,stop_signal);signal.signal(signal.SIGINT,stop_signal)
    def on_mode(msg):
        with lock:state.update(mode=int(msg.data),mode_at=time.monotonic())
    def on_enabled(msg):
        with lock:state['enabled']=bool(msg.data)
    def on_transport(msg):
        with lock:state.update(transport=msg.data,transport_at=time.monotonic())
    def on_disable(msg):
        with lock:state['disabled']=bool(msg.data)
    def on_stop(msg):
        if msg.data:
            with lock:state['stop_reason']='로봇 정지 신호(/stop_robot) 수신'
            done.set()
    def on_odom(msg):
        try:
            p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist
            values=[float(x) for x in (q.x,q.y,q.z,q.w)]
            if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values)-1)>.02:
                raise ValueError('odom 회전 값 오류')
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            with lock:tracker.odometry([float(p.x),float(p.y),yaw],
                [float(v.linear.x),float(v.linear.y),float(v.angular.z)],
                msg.header.stamp.to_nsec(),msg.header.frame_id,time.monotonic())
        except Exception as exc:
            with lock:state['failure']='상대 이동 피드백 오류: '+str(exc)
    def on_sensor(msg):
        try:
            q=finite_vector(list(msg.joint_data.joint_q[:18]),18)
            v=finite_vector(list(msg.joint_data.joint_v[:18]),18)
            now=time.monotonic()
            # ROS source stamps must progress too; repeated messages are not fresh feedback.
            stamp=msg.sensor_time.to_nsec() or msg.header.stamp.to_nsec()
            with lock:
                if not stamp or stamp<=state.get('q_stamp',0):return
                state.update(q=q,v=v,q_at=now,q_stamp=stamp)
        except Exception as exc:
            with lock:state['failure']='관절 피드백 오류: '+str(exc)
    def on_claw(msg):
        try:
            stamp=msg.header.stamp.to_sec()
            sample=claw_feedback(msg.data.position,msg.data.velocity,msg.data.effort,
                                 msg.state,msg.data.name,stamp,rospy.Time.now().to_sec())
            if sample is None:return
            with lock:
                if stamp<=state['claw_stamp']:return
                state.update(claw=sample[0],claw_state=sample[1],claw_at=time.monotonic(),claw_stamp=stamp)
        except Exception as exc:
            with lock:state['failure']=str(exc)
    def on_reach(msg):
        value=float(msg.data)
        if not math.isfinite(value) or value<0:return
        with lock:state['reach']=(value,time.monotonic())
    def foreign(topic):
        def callback(msg):
            caller=(msg._connection_header or {}).get('callerid','?')
            if caller!=rospy.get_name():
                with lock:state['failure']='다른 제어 명령 감지: %s (%s)'%(topic,caller)
        return callback
    rospy.Subscriber('/mobile_manipulator/lb_mpc_control_mode',Int8,on_mode,queue_size=1)
    rospy.Subscriber('/robot_disabled_flag',Bool,on_disable,queue_size=1)
    rospy.Subscriber('/enable_control_state',Bool,on_enabled,queue_size=1)
    rospy.Subscriber('/ik_debug/arm_traj_receive/transport',Float64,on_transport,queue_size=1)
    rospy.Subscriber('/stop_robot',Bool,on_stop,queue_size=1)
    rospy.Subscriber('/lb_cmd_pose_reach_time',Float32,on_reach,queue_size=5)
    rospy.Subscriber('/odom',Odometry,on_odom,queue_size=1)
    rospy.Subscriber('/sensors_data_raw',sensorsData,on_sensor,queue_size=1)
    rospy.Subscriber('/leju_claw_state',lejuClawState,on_claw,queue_size=1)
    for topic in ['/cmd_vel','/cmd_vel_world','/cmd_pose','/cmd_pose_world','/kuavo_arm_traj',
                  '/mm/two_arm_hand_pose_cmd','/cmd_lb_torso_pose','/lb_leg_traj','/leju_claw_command']:
        rospy.Subscriber(topic,rospy.AnyMsg,foreign(topic),queue_size=1)
    def stdin():
        try:
            for line in sys.stdin:
                data=json.loads(line)
                if data.get('op')=='stop':
                    with lock:state['stop_reason']='대시보드 정지 요청'
                    break
                with lock:
                    if data.get('op')=='configure':
                        if state['target'] is not None:raise ValueError('정렬 목표는 실행 중 바꿀 수 없습니다')
                        # Validate before creating any command publisher or acquiring control.
                        JointBridge([0.]*18,data['target'])
                        state['target']=data['target']
                        if data.get('replay') is not None:
                            state['replay']=ReplayPath(data['replay']);tracker.replaying=True
                    elif data.get('op')=='vision':tracker.observation(data,time.monotonic())
                    elif data.get('op')=='beat':tracker.heartbeat(time.monotonic())
        except Exception as exc:
            with lock:state['failure']=str(exc)
        finally:
            with lock:
                if not state['failure'] and state['stop_reason']=='정렬 시작/이동 취소':
                    state['stop_reason']='대시보드 제어 파이프 종료'
            done.set()
    threading.Thread(target=stdin,daemon=True).start()
    def read_arm_mode():
        getter=None
        while not done.is_set() and not rospy.is_shutdown():
            try:
                if getter is None:
                    rospy.wait_for_service('/humanoid_get_arm_ctrl_mode',timeout=.5)
                    getter=rospy.ServiceProxy('/humanoid_get_arm_ctrl_mode',changeArmCtrlMode)
                if not arm_feedback.read(getter,time.monotonic()):getter=None
            except Exception as exc:
                arm_feedback.error=str(exc);getter=None
            done.wait(.5)
    threading.Thread(target=read_arm_mode,daemon=True).start()
    base_pub=base_pose_pub=arm_pub=torso_pub=claw_pub=None;torso_model=None;pose_sent_at=None;plan=None
    success=False;activated=False;entered=False;arm_service=None;prev_arm=None
    mpc_service=None;quick_service=None;prev_mpc=None;mpc_changed=False
    commanded=None;last_claw=None;link_ready=False

    def guard(now,require_mpc=True):
        if state['failure']:raise ValueError(state['failure'])
        if done.is_set() or rospy.is_shutdown():raise ValueError(state['stop_reason'])
        if state['enabled'] is not True:raise ValueError('로봇 제어 enable 피드백이 꺼져 있거나 없습니다.')
        if link_ready and (state['transport']!=2 or now-state['transport_at']>2.5):
            state['failure']='팔 입력 경로가 ROS 궤적에서 변경되었거나 피드백이 끊겼습니다.'
            raise ValueError(state['failure'])
        if (state['disabled'] is not False or state['mode'] not in (0,1,2,3)
                or (require_mpc and state['mode']!=JOINT_CONTROL_MODES['mpc']) or now-state['mode_at']>2):
            raise ValueError('몸통·팔·베이스 동시 제어 가능한 MPC 모드 3을 확인하세요.')
        arm_mode=arm_feedback.current(now)
        if arm_mode is None:
            raise ValueError('팔 제어 모드 조회 실패: '+arm_feedback.error)
        if state['q'] is None or now-state['q_at']>.5:raise ValueError('실측 관절 피드백이 끊겼습니다.')
        if state['claw'] is None or now-state['claw_at']>1.:raise ValueError('실측 그리퍼 피드백이 끊겼습니다.')
        if len(state['claw_state'])!=2 or any(x<0 for x in state['claw_state']):
            raise ValueError('그리퍼 오류 상태입니다.')
        if state['replay'] is None and (tracker.odom_at is None or now-tracker.odom_at>.3):
            raise ValueError('최신 상대 이동용 odom 피드백이 없습니다.')
        if entered and arm_mode!=2:
            raise ValueError('팔 제어권이 바뀌었습니다.')

    def send_joints(q,v=None):
        torso,arm=joint_commands(q)
        rates=joint_commands([0.]*18 if v is None else v)
        if plan:
            m=JointState();m.header.stamp=rospy.Time.now();m.name=NAMES[:4];m.position=torso;m.velocity=rates[0]
        else:
            pose=torso_model.position(q[:4])
            m=Twist();m.linear.x,m.linear.z,m.angular.y,m.angular.z=pose
        torso_pub.publish(m)
        m=JointState();m.header.stamp=rospy.Time.now();m.name=NAMES[4:];m.position=arm
        if plan:m.velocity=rates[1]
        arm_pub.publish(m)

    def send_claw(values):
        m=lejuClawCommand();m.header.stamp=rospy.Time.now();m.data.name=['left_claw','right_claw'];m.data.position=list(values)
        m.data.velocity=[50.,50.];m.data.effort=[1.,1.];claw_pub.publish(m)

    try:
        start=time.monotonic()
        while time.monotonic()-start<4 and not done.is_set():
            with lock:
                if state['failure']:raise ValueError(state['failure'])
                ready=(state['target'] is not None and state['q'] is not None and state['claw'] is not None
                       and arm_feedback.current(time.monotonic()) is not None and tracker.input_at is not None)
            if ready and time.monotonic()-start>=.8:break
            done.wait(.05)
        with lock:
            guard(time.monotonic(),require_mpc=False)
            prev_mpc=state['mode']
            if state['target'] is None:raise ValueError('선택 구간 시작 목표가 없습니다.')
            target=state['target'];plan=state['replay']
            if plan:
                first=plan.elapsed(0)[0]
                if max(abs(a-b) for a,b in zip(first[3:],target['q']))>1e-8:
                    raise ValueError('replay 첫 자세와 정렬 목표 불일치')
                if max(abs(a-b) for a,b in zip(state['q'],first[3:]))>math.radians(1.5):
                    raise ValueError('replay 첫 자세로 다시 정렬하세요.')
            # Reject out-of-limit goals rather than clamping to a different state.
            model_xml=rospy.get_param('/robot_description');torso_model=TorsoModel(model_xml)
            root=ET.fromstring(model_xml)
            limits={j.get('name'):j.find('limit') for j in root.findall('joint')}
            for name,value,current in zip(NAMES,target['q'],state['q']):
                limit=limits.get(name)
                if (limit is None or not float(limit.get('lower'))<=value<=float(limit.get('upper'))
                        or not float(limit.get('lower'))<=current<=float(limit.get('upper'))):
                    raise ValueError('시작 관절 한계 오류: '+name)
                if not plan and NAMES.index(name)>=4:
                    peak=1.5*abs(value-current)/3.
                    if limit.get('velocity') is not None and peak>float(limit.get('velocity'))+1e-8:
                        raise ValueError('팔 3초 이동이 URDF 속도 한계를 넘습니다: '+name)
                if plan and any(not float(limit.get('lower'))<=row[3+NAMES.index(name)]<=float(limit.get('upper')) for row in plan.p):
                    raise ValueError('replay 관절 한계 오류: '+name)
        if not plan:
            emit(stage='초기 랙 위치 확정 중 · 움직이지 않고 관측')
            deadline=time.monotonic()+6.
            while not tracker.ready:
                with lock:
                    guard(time.monotonic(),require_mpc=False)
                    if tracker.input_at is None or time.monotonic()-tracker.input_at>LEASE_S:
                        raise ValueError('초기 관측 중 페이지 연결이 끊겼습니다.')
                if time.monotonic()>deadline:raise ValueError('초기 랙 위치 안정화 시간 초과')
                done.wait(.02)
            with lock:frozen_goal=tracker.freeze(time.monotonic())
            emit(stage='초기 위치 확정 · 상대 이동 목표 고정',vision_locked=True,
                 base_goal_odom=frozen_goal,base_odom_frame=tracker.odom_frame,
                 initial_error=tracker.initial_error)
        # Resolve every required mode service before the base can move.
        for name in ['/wheel_arm_change_arm_ctrl_mode','/change_arm_ctrl_mode']:
            try:rospy.wait_for_service(name,timeout=1);arm_service=rospy.ServiceProxy(name,changeArmCtrlMode);break
            except rospy.ROSException:continue
        if arm_service is None:raise ValueError('팔 제어 모드 서비스가 없습니다.')
        rospy.wait_for_service('/mobile_manipulator_mpc_control',timeout=1)
        mpc_service=rospy.ServiceProxy('/mobile_manipulator_mpc_control',changeTorsoCtrlMode)
        rospy.wait_for_service(ARM_LINK_SERVICE,timeout=1)
        link_service=rospy.ServiceProxy(ARM_LINK_SERVICE,SetIncrementalArmTrajLink)
        with lock:
            guard(time.monotonic(),require_mpc=False)
            if state['transport'] not in (0,1,2) or time.monotonic()-state['transport_at']>2.5:
                raise ValueError('팔 입력 경로 피드백을 확인할 수 없습니다.')
        rospy.wait_for_service('/enable_lb_arm_quick_mode',timeout=1)
        quick_service=rospy.ServiceProxy('/enable_lb_arm_quick_mode',changeLbQuickModeSrv)
        with lock:guard(time.monotonic(),require_mpc=False)
        if prev_mpc!=JOINT_CONTROL_MODES['mpc']:
            emit(stage='베이스·몸통·팔 동시 제어 모드 3 전환 확인 중')
            response=mpc_service(control_mode=JOINT_CONTROL_MODES['mpc'])
            if not response.result:raise ValueError('MPC 모드 3 전환 실패')
            mpc_changed=True
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                with lock:
                    guard(time.monotonic(),require_mpc=False)
                    if state['mode']==JOINT_CONTROL_MODES['mpc']:break
                done.wait(.02)
        with lock:guard(time.monotonic())
        base_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1,latch=False,tcp_nodelay=True)
        base_pose_pub=None if plan else rospy.Publisher('/cmd_pose',Twist,queue_size=1,latch=False,tcp_nodelay=True)
        torso_pub=(rospy.Publisher('/lb_leg_traj',JointState,queue_size=1,latch=False,tcp_nodelay=True) if plan
                   else rospy.Publisher('/cmd_lb_torso_pose',Twist,queue_size=1,latch=False,tcp_nodelay=True))
        arm_pub=rospy.Publisher('/kuavo_arm_traj',JointState,queue_size=1,latch=False,tcp_nodelay=True)
        claw_pub=rospy.Publisher('/leju_claw_command',lejuClawCommand,queue_size=1,latch=False,tcp_nodelay=True)
        deadline=time.monotonic()+3
        pubs=[base_pub,torso_pub,arm_pub,claw_pub]+([] if plan else [base_pose_pub])
        while any(p.get_num_connections()==0 for p in pubs) and time.monotonic()<deadline and not done.is_set():
            done.wait(.02)
        if any(p.get_num_connections()==0 for p in pubs):raise ValueError('베이스·몸통·팔·그리퍼 수신 제어기를 확인하세요.')
        with lock:
            guard(time.monotonic())
            prev_arm=arm_feedback.current(time.monotonic())
            if target.get('stationary_confirmed') is not True and max(map(abs,state['v']))>math.radians(1):raise ValueError('현재 몸통·팔이 정지한 뒤 다시 시작하세요.')
            bridge=JointBridge(state['q'],target)
            start_claw=list(state['claw'])
        # Use the existing arm authority service; never automatically reacquire
        # control if the operator or another controller takes it away.
        # Prime the direct-joint caches at the measured pose before selecting
        # quickMode=3. Otherwise a cached/default torso goal could be applied.
        emit(stage='몸통·팔 관절 명령 경로 준비 중',velocity=[0,0,0],stationary_confirmed=target.get('stationary_confirmed') is True)
        for _ in range(5):
            with lock:
                guard(time.monotonic())
                if tracker.input_at is None or time.monotonic()-tracker.input_at>LEASE_S:
                    raise ValueError('관절 제어 경로 준비 중 제어 연결이 끊겼습니다.')
                commanded=list(state['q'])
            base_pub.publish(Twist());send_joints(commanded);done.wait(.02)
        # Remove the arm quick override BEFORE mode 2 is selected. Otherwise
        # the controller's 200ms hold/transition result is overwritten later.
        response=quick_service(quickMode=PREPARATION_CONTROL_MODES['quick'])
        if not response.success:raise ValueError('첫 자세 일반 팔 제어 경로 전환 실패')
        emit(stage='팔 외부 제어 모드 2 전환 중',velocity=[0,0,0],previous_arm_mode=prev_arm)
        # Require a fresh query after SET; a stale cached 2 cannot acknowledge it.
        arm_feedback.sample=None
        response=arm_service(control_mode=2)
        if not response.result:raise ValueError('팔 외부 제어 모드 전환 실패')
        entered=True
        # Arm mode and receiver input are separate controls. Mode 2 alone cannot
        # make /kuavo_arm_traj reach WBC while the receiver selects SHM. The
        # normal mode-switch hold remains active while the measured pose primes
        # the ROS input. This is only reached after the explicit Start admission.
        with lock:
            if state['failure'] or done.is_set():raise ValueError(state['failure'] or state['stop_reason'])
            link_switch=state['transport']!=2
        if link_switch:
            response=link_service(transport=2)
            if not response.success:raise ValueError('팔 ROS 궤적 입력 경로 전환 실패')
        link_requested=time.monotonic()
        # Wait for fresh receiver feedback, never infer it from an RPC success.
        deadline=time.monotonic()+2.5
        while time.monotonic()<deadline:
            with lock:
                if state['failure'] or done.is_set() or rospy.is_shutdown():raise ValueError(state['failure'] or state['stop_reason'])
                if (state['transport']==2 and (not link_switch or state['transport_at']>=link_requested)):
                    link_ready=True;break
                commanded=list(state['q'])
            base_pub.publish(Twist());send_joints(commanded,[0.]*18);done.wait(.02)
        if not link_ready:raise ValueError('팔 ROS 궤적 입력 경로 전환 피드백 시간 초과')
        emit(stage='팔 ROS 궤적 입력 경로 확인',arm_transport=2)
        # Wait for a read-only service query to acknowledge the new arm mode.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            with lock:
                if done.is_set() or state['failure']:raise ValueError('정렬 제어권 확인 취소')
                if arm_feedback.current(time.monotonic())==2:break
                commanded=list(state['q'])
            base_pub.publish(Twist());send_joints(commanded,[0.]*18)
            done.wait(.02)
        with lock:
            guard(time.monotonic())
            if target.get('stationary_confirmed') is not True and max(map(abs,state['v']))>math.radians(1):raise ValueError('몸통·팔 정지 피드백을 확인하세요.')
            bridge=JointBridge(state['q'],target)
            start_claw=list(state['claw'])
        with lock:guard(time.monotonic())
        control_modes=JOINT_CONTROL_MODES if plan else PREPARATION_CONTROL_MODES
        # ROS publications made before selecting transport 2 cannot prime WBC's
        # SHM-selected cache. Prime AFTER selection and acknowledgement, leaving
        # the normal arm route active until ten measured-pose updates have flowed.
        for _ in range(10):
            with lock:
                guard(time.monotonic())
                if tracker.input_at is None or time.monotonic()-tracker.input_at>LEASE_S:
                    raise ValueError('직접 관절 입력 준비 중 제어 연결이 끊겼습니다.')
                commanded=list(state['q'])
            base_pub.publish(Twist());send_joints(commanded,[0.]*18);done.wait(.02)
        with lock:guard(time.monotonic())
        if plan:
            response=quick_service(quickMode=JOINT_CONTROL_MODES['quick'])
            if not response.success:raise ValueError('replay 직접 관절 제어 경로 전환 실패')
        control_modes=JOINT_CONTROL_MODES if plan else PREPARATION_CONTROL_MODES
        emit(stage=('replay 시각별 관절 목표 전송 준비' if plan else 'RWH 베이스 상대 목표 · 몸통 자세 · 팔 3초 준비'),
             velocity=[0,0,0],control_modes=control_modes,executor=EXECUTOR_VERSION,controller_pva_exact=False)
        with lock:
            guard(time.monotonic())
            bridge=JointBridge(state['q'],target);start_claw=list(state['claw'])
        if plan:
            # The robot must already be at rest at the exact selected beginning.
            # Waiting for stable observations sends only the measured holding pose.
            deadline=time.monotonic()+5
            while not tracker.ready:
                now=time.monotonic()
                with lock:
                    guard(now)
                    if tracker.input_at is None or now-tracker.input_at>LEASE_S or now-tracker.vision_at>VISION_S:
                        raise ValueError('replay 시작 전 영상·제어 연결 소실')
                if now>deadline:raise ValueError('replay 시작 전 랙 관측 안정화 실패')
                base_pub.publish(Twist());send_joints(commanded,[0.]*18);done.wait(.01)
            with lock:
                guard(time.monotonic())
                first=plan.elapsed(0);base_error=relative(tracker.pose,first[0][:3])
                if (math.hypot(*base_error[:2])>POSITION_TOL or abs(base_error[2])>YAW_TOL
                    or max(abs(a-b) for a,b in zip(state['q'],first[0][3:]))>math.radians(1.5)
                    or (target.get('stationary_confirmed') is not True and max(map(abs,state['v']))>math.radians(1))
                    or max(abs(a-b) for a,b in zip(state['claw'],first[3]))>CLAW_POSITION_TOL):
                    raise ValueError('replay 첫 state 도착·정지를 다시 확인하세요.')
            began=time.monotonic();last=began-.01;last_world=[0.,0.,0.]
            history=collections.deque([(began-2.,last_world)]);settled=None;report=0;replay_still=Stationarity();claw_final_sent=False
            while True:
                now=time.monotonic();dt=now-last;last=now
                if not 0<dt<=.05:raise ValueError('replay 100Hz 실행이 50ms 이상 지연되어 정지합니다.')
                elapsed=now-began
                with lock:
                    guard(now)
                    if tracker.input_at is None or now-tracker.input_at>LEASE_S or now-tracker.vision_at>VISION_S:
                        raise ValueError('replay 중 영상·제어 연결이 끊겼습니다.')
                    actual=list(state['q']);velocity=list(state['v']);grip=list(state['claw']);grip_state=list(state['claw_state'])
                    pose=predict_pose(tracker.pose,tracker.vision_at,now,history)
                p,v,a,claw=plan.elapsed(elapsed)
                commanded=p[3:]
                if max(abs(x-y) for x,y in zip(actual,commanded))>math.radians(10):
                    raise ValueError('replay 관절 추종 오차가 10°를 넘었습니다.')
                values,last_world,error=replay_base(p,v,pose,last_world,dt,plan.vmax,plan.amax)
                cmd=Twist();cmd.linear.x,cmd.linear.y,cmd.angular.z=values
                base_pub.publish(cmd);activated=True;send_joints(commanded,v[3:])
                if elapsed<plan.duration or not claw_final_sent:
                    send_claw(claw)
                    if elapsed>=plan.duration:claw_final_sent=True
                history.append((now,list(last_world)))
                while len(history)>2 and history[1][0]<now-2.:history.popleft()
                # Completion requires fresh, distinct vision frames AFTER the
                # endpoint and measured joint stopping, not just elapsed time.
                end_reached=elapsed>=plan.duration
                stopped=replay_still.update(actual,velocity,now)
                close=(end_reached and math.hypot(*error[:2])<=POSITION_TOL and abs(error[2])<=YAW_TOL
                       and max(abs(x-y) for x,y in zip(actual,commanded))<=math.radians(1.5)
                       and stopped
                       and max(abs(x-y) for x,y in zip(grip,claw))<=CLAW_POSITION_TOL
                       and len(grip_state)==2 and all(x in (2,3) for x in grip_state)
                       and max(map(abs,last_world))<.005)
                if not close:settled=None
                elif settled is None:settled=now
                emit(sample=True,execution_s=min(plan.end,plan.begin+elapsed),elapsed_s=min(elapsed,plan.duration),
                     position=p,velocity_target=v,acceleration_target=a,claw_target=claw,
                     actual_q=actual,actual_v=velocity,actual_claw=grip,base_pose=pose,base_command=values)
                if settled is not None and now-settled>=1. and tracker.vision_at>=settled:
                    success=True;break
                if elapsed>plan.duration+10:raise ValueError('replay 끝 자세 도착·정지 확인 시간 초과')
                if now-report>.2:
                    emit(stage='replay 1배속 · %.2f / %.2f초'%(min(elapsed,plan.duration),plan.duration),
                         elapsed_s=min(elapsed,plan.duration),duration_s=plan.duration,velocity=values)
                    report=now
                done.wait(next_replay_wait(began,time.monotonic()))
        else:
            with lock:
                guard(time.monotonic())
                displacement=relative(tracker.odom_pose,tracker.goal)
            request=Twist();request.linear.x,request.linear.y,request.angular.z=displacement
            pose_sent_at=time.monotonic()
            base_pose_pub.publish(request);activated=True
            deadline=pose_sent_at+2.
            while True:
                with lock:
                    now=time.monotonic();guard(now)
                    if tracker.input_at is None or now-tracker.input_at>LEASE_S:raise ValueError('상대 이동 확인 중 페이지 연결 소실')
                    reached=state['reach']
                    if reached is not None and reached[1]>=pose_sent_at:
                        if reached[0]>TIMEOUT_S:raise ValueError('베이스 도착 예정 시간이 실행 한도를 넘었습니다.')
                        break
                    commanded=list(state['q'])
                if now>deadline:raise ValueError('베이스 상대 목표 수신 확인이 없습니다.')
                send_joints(commanded);done.wait(.02)
            # RWH's base controller owns the one-shot displacement profile.
            # Re-sample measured joints only after command acceptance.
            with lock:
                bridge=JointBridge(state['q'],target);start_claw=list(state['claw'])
            began=last=time.monotonic();claw_began=None;report=0;posture_done=False;base_done=False;claw_final_sent=False
            while True:
                now=time.monotonic();dt=now-last;last=now
                if not 0<dt<=.25:raise ValueError('첫 자세 제어 주기가 지연됐습니다.')
                with lock:
                    guard(now)
                    # Keep vision/page leases active through the posture connection.
                    if tracker.input_at is None or now-tracker.input_at>LEASE_S:
                        raise ValueError('첫 자세 이동 중 제어 연결이 끊겼습니다.')
                    base_done=tracker.pose_arrival(now);values=list(tracker.odom_v)
                    base_message='RWH 상대 목표 이동 · odom 도착 확인'
                    actual=list(state['q']);velocity=list(state['v']);grip=list(state['claw']);grip_state=list(state['claw_state'])
                elapsed=now-began
                commanded=bridge.position(elapsed)
                errors=[abs(a-b) for a,b in zip(actual,commanded)]
                if max(errors)>math.radians(10):
                    j=max(range(18),key=errors.__getitem__)
                    raise ValueError('첫 자세 관절 추종 오차 10° 초과: %s · %.2f초 · 실제 %.2f° / 명령 %.2f° / 목표 %.2f°'%(
                        NAMES[j],elapsed,math.degrees(actual[j]),math.degrees(commanded[j]),math.degrees(bridge.goal[j])))
                send_joints(commanded)
                # Move claws only after the body and arms reach their first pose.
                stopped=bridge.still.update(actual,velocity,now)
                joints_close=(elapsed>=bridge.duration and max(abs(a-b) for a,b in zip(actual,bridge.goal))<=math.radians(1.5)
                              and stopped)
                if joints_close and claw_began is None:claw_began=now
                if claw_began is not None:
                    # Smooth, low-speed claw preparation (max 20 percentage points/s).
                    seconds=max(2.,1.875*max(abs(a-b) for a,b in zip(start_claw,bridge.claw))/20)
                    u=min(1.,(now-claw_began)/seconds);fraction=u*u*u*(10+u*(-15+6*u))
                    last_claw=[a+(b-a)*fraction for a,b in zip(start_claw,bridge.claw)]
                    # As in GraspSystem, cease command streaming before waiting
                    # for the driver's reached/grabbed state at the endpoint.
                    if u<1 or not claw_final_sent:
                        send_claw(last_claw)
                        if u>=1:claw_final_sent=True
                    posture_done=u>=1 and bridge.arrived(actual,velocity,grip,grip_state,now)
                    if posture_done and base_done:success=True;break
                if not posture_done and elapsed>bridge.duration+25:
                    raise ValueError('첫 자세 완료 시간 초과: 관절 %.2f도, 정지 RMS %s도/s, 그리퍼 오차 %.1f%%, 상태 %s'%(
                        math.degrees(max(abs(a-b) for a,b in zip(actual,bridge.goal))),
                        ('%.2f'%bridge.still.rms_deg_s if bridge.still.rms_deg_s is not None else '?'),
                        max(abs(a-b) for a,b in zip(grip,bridge.claw)),grip_state))
                if now-report>.2:
                    stage=('팔·몸통·그리퍼 첫 state 도착 · '+base_message if posture_done else
                           '베이스 정렬 + '+('첫 자세 도착·그리퍼 확인 중' if claw_began is not None else '첫 자세로 천천히 이동 중'))
                    error_deg=math.degrees(max(abs(a-b) for a,b in zip(actual,bridge.goal)))
                    if not posture_done:stage+=' · 팔 %.1f/3.0초 · 몸통 %.1f/%.1f초 · 관절 오차 %.1f°'%(min(elapsed,3.),min(elapsed,bridge.body_duration),bridge.body_duration,error_deg)
                    stage+=' · 정지 RMS %s°/s · 그리퍼 오차 %.1f%%'%(
                        ('%.2f'%bridge.still.rms_deg_s if bridge.still.rms_deg_s is not None else '?'),
                        max(abs(a-b) for a,b in zip(grip,bridge.claw)))
                    emit(stage=stage,velocity=values,base_error=tracker.visual,base_arrived=base_done,posture_arrived=posture_done,
                         joint_error_deg=math.degrees(max(abs(a-b) for a,b in zip(actual,bridge.goal))))
                    report=now
                done.wait(max(.001,.02-(time.monotonic()-now)))
    except Exception as exc:
        success=False;emit(error=str(exc))
    finally:
        if done.is_set():success=False
        if state.get('replay') is not None and arm_pub is not None and entered and not state['failure']:
            # Also clear the last target velocity after normal completion.
            with lock:hold=list(state['q']) if not success and state['q'] is not None else commanded
            if hold is not None:send_joints(hold,[0.]*18)
        if base_pose_pub is not None and pose_sent_at is not None and not success and not state['failure']:
            base_pose_pub.publish(Twist())  # Cancel our own relative goal, as RWH does.
        if base_pub is not None and activated:
            for _ in range(8):
                base_pub.publish(Twist());time.sleep(.05)
        # Successful preparation holds external arm mode 2 for replay. A failed
        # move releases only authority we still own; never fight a takeover.
        if not success and entered and arm_service is not None:
            with lock:owned=arm_feedback.current(time.monotonic())==2 and not state['failure']
            if owned:
                try:
                    if commanded is not None:
                        # Cancel velocity feed-forward at the measured pose.
                        with lock:hold=list(state['q']) if state['q'] is not None else commanded
                        send_joints(hold,[0.]*18)
                    arm_service(control_mode=prev_arm)
                except Exception as exc:emit(error='팔 제어권 복원 실패: '+str(exc))
        if not success and mpc_changed and mpc_service is not None:
            with lock:owned=state['mode']==JOINT_CONTROL_MODES['mpc'] and not state['failure']
            if owned:
                try:mpc_service(control_mode=prev_mpc)
                except Exception as exc:emit(error='MPC 제어 모드 복원 실패: '+str(exc))
        emit(done=True,success=success,head_excluded=True,executor=EXECUTOR_VERSION,controller_pva_exact=False,
             base_goal_odom=tracker.goal if tracker.frozen else None,base_odom_frame=tracker.odom_frame,
             stage=('replay 종료·정지 확인 완료' if state.get('replay') else '첫 자세 도착·정지 확인 완료') if success else '실행 정지')
    return 0 if success else 1


def build_package(key):
    """Read and validate final artifacts on the host; no ROS or motion IO."""
    import io
    from pathlib import Path
    import numpy as np
    fingerprint,stats,interval,allow_unverified=key
    trajectory,source,report_path=[Path(x[0]) for x in stats]
    raw={str(path):path.read_bytes() for path in (trajectory,source,report_path)}
    if any((path.stat().st_mtime_ns,path.stat().st_size)!=(stamp,size) for (name,stamp,size),path in zip(stats,(trajectory,source,report_path))):
        raise ValueError('replay 파일을 읽는 동안 변경됐습니다. 다시 준비하세요.')
    report=json.loads(raw[str(report_path)])
    # Keep the existing standard gate. A replay driver never promotes a result.
    if report.get('status')!='verified' and allow_unverified is not True:raise ValueError('선택한 궤적의 standard 검증이 통과되지 않았습니다.')
    peaks=report.get('validation',{}).get('summary',{}).get('joint_peaks')
    names=['base_x','base_y','base_yaw']+NAMES
    if not isinstance(peaks,list) or [x.get('name') for x in peaks]!=names:
        raise ValueError('검증 보고서의 좌표별 속도·가속도 제한이 없습니다.')
    with np.load(io.BytesIO(raw[str(trajectory)]),allow_pickle=False) as z:
        data={k:z[k].tolist() for k in ('execution_s','source_s','position','velocity','acceleration','names')}
    phases=data['source_s']
    if (len(phases)!=len(data['execution_s']) or not np.isfinite(phases).all()
            or np.any(np.diff(phases)<0)):raise ValueError('원본 시각 매핑 오류')
    times=[];grips=[]
    for line in raw[str(source)].decode().splitlines():
        row=json.loads(line)
        g=row.get('grippers')
        if not row.get('valid') or not isinstance(g,list) or len(g)!=2:continue
        values=finite_vector([x.get('open_fraction') if isinstance(x,dict) else None for x in g],2)
        if any(not 0<=x<=1 for x in values):raise ValueError('그리퍼 열림 비율 오류')
        times.append(float(row['elapsed_s']));grips.append([100*(1-x) for x in values])
    if (len(times)<2 or not np.isfinite(times).all() or np.any(np.diff(times)<=0)):
        raise ValueError('원본 양쪽 그리퍼 궤적이 없거나 시각 순서가 다릅니다.')
    selected=np.array(data['execution_s'])
    used=np.interp([interval[0],interval[1]],selected,phases)
    if used[0]<times[0]-1e-6 or used[1]>times[-1]+1e-6:
        raise ValueError('선택 구간의 그리퍼 원본이 없습니다.')
    if any(b-a>.25 for a,b in zip(times,times[1:]) if a<used[1] and b>used[0]):
        raise ValueError('선택 구간의 그리퍼 피드백 공백이 0.25초를 넘습니다.')
    data['claw']=np.array([np.interp(phases,times,np.array(grips)[:,i]) for i in range(2)]).T.tolist()
    data.update(interval=list(interval),fingerprint=fingerprint,
        velocity_limits=[float(x['velocity_limit']) for x in peaks],
        acceleration_limits=[float(x['acceleration_limit']) for x in peaks],
        artifact_sha256={name:hashlib.sha256(raw[str(path)]).hexdigest() for name,path in [('trajectory',trajectory),('source',source),('validation',report_path)]},
        rate=1.,kalman_applied=False,standard_status=report.get('status'),manual_standard_approval=allow_unverified is True)
    data['payload_sha256']=hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    ReplayPath(data)  # Exact same validation runs again on the robot.
    return data


def cli(argv=None):
    args=sys.argv[1:] if argv is None else argv
    try:
        if args==['--check-runtime']:return check_runtime()
        if args:raise ValueError('지원하지 않는 실행기 인자')
        return main()
    except Exception as exc:
        emit(done=True,success=False,error='실행기 초기화 실패: %s: %s'%(type(exc).__name__,exc),stage='정렬 시작 실패')
        return 1


if __name__=='__main__':sys.exit(cli())
