#!/usr/bin/env python3
"""Read-only full-data recorder for replay, reusing robot_databag's rosbag backend.

Imports are ROS-free. Only main() subscribes/records; no actuator publisher or
controller service is created. Team rosbag_data.sh is read with --list, not edited.
"""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

EVENT_PREFIX = '/warp/replay/events/'
CAMERAS = ('head_camera', 'left_wrist_camera', 'right_wrist_camera')
IMAGE_TOPICS = [f'/{c}/color/image_raw/compressed' for c in CAMERAS] + [
    '/head_camera/depth/image_raw/compressedDepth',
    '/left_wrist_camera/depth/image_rect_raw/compressedDepth',
    '/right_wrist_camera/depth/image_rect_raw/compressedDepth']
INFO_TOPICS = [f'/{c}/{kind}/camera_info' for c in CAMERAS for kind in ('color', 'depth')]
STATE_TOPICS = ['/sensors_data_raw', '/leju_claw_state', '/odom', '/tf']
COMMAND_TOPICS = ['/cmd_vel', '/kuavo_arm_traj', '/lb_leg_traj', '/leju_claw_command']
EXTRA_TOPICS = ['/cmd_pose', '/cmd_lb_torso_pose', '/cmd_torso_vel', '/torso_open_loop_state',
    '/tf_static', '/mobile_manipulator/lb_mpc_control_mode', '/ik_debug/arm_traj_receive/transport']
REQUIRED_LIVE = IMAGE_TOPICS + INFO_TOPICS + STATE_TOPICS + ['/tf_static']
LATCHED = set(INFO_TOPICS + ['/tf_static'])
MIN_FREE_BYTES = 10*1024**3


def emit(**d):
    print(json.dumps(d, separators=(',', ':'), allow_nan=False), flush=True)


def topics_for_run(run_id, inherited=()):
    if not re.fullmatch(r'[a-f0-9]{32}', run_id):
        raise ValueError('Invalid run ID')
    return list(dict.fromkeys(list(inherited) + REQUIRED_LIVE + COMMAND_TOPICS + EXTRA_TOPICS + [EVENT_PREFIX+run_id]))


class Stats:
    def __init__(self):
        self.count=0; self.first=None; self.last=None; self.last_received=None
        self.max_gap=0.; self.bad_stamps=0; self.source_first=None; self.source_last=None
        self.distinct_source=0

    def add(self, received_s, stamp_ns=None):
        if self.last is not None:self.max_gap=max(self.max_gap, received_s-self.last)
        if self.first is None:self.first=received_s
        self.last=received_s; self.last_received=time.monotonic(); self.count+=1
        if stamp_ns is not None:
            if stamp_ns<=0 or (self.source_last is not None and stamp_ns<=self.source_last):self.bad_stamps+=1
            else:
                self.distinct_source+=1
                if self.source_first is None:self.source_first=stamp_ns
            self.source_last=stamp_ns

    def summary(self):
        span=(self.last-self.first) if self.count>1 else 0.
        return dict(count=self.count, hz=(self.count-1)/span if span>0 else 0.,
            first_s=self.first,last_s=self.last,max_gap_s=self.max_gap,
            bad_source_stamps=self.bad_stamps,source_first_ns=self.source_first,source_last_ns=self.source_last)


def live_problem(stats, now):
    for topic in REQUIRED_LIVE:
        s=stats.get(topic)
        if s is None or not s.count:return '수신 대기: '+topic
        if topic not in LATCHED:
            if s.count<2 or now-s.last_received>1.5:return '새 데이터 대기: '+topic
            if topic in IMAGE_TOPICS and s.distinct_source<2:return '영상 원본 시각 대기: '+topic
            if topic in IMAGE_TOPICS and (s.source_last is None or abs(s.source_last/1e9-s.last)>1.5):
                return '영상·ROS 시계가 맞지 않습니다: '+topic
    return ''


def quality(topic_stats, all_counts, begin_ns, end_ns, finalized=True, command_end_ns=None):
    errors=[]
    if not finalized:errors.append('bag 정상 종료 실패')
    if not isinstance(begin_ns,int) or not isinstance(end_ns,int) or not 0<begin_ns<end_ns:
        return dict(valid=False,errors=errors+['replay 시작·종료 ROS 시각 없음'],topics=topic_stats)
    begin,end=begin_ns/1e9,end_ns/1e9
    for topic in REQUIRED_LIVE+COMMAND_TOPICS:
        row=topic_stats.get(topic,{})
        if topic in LATCHED:
            if not all_counts.get(topic,0):errors.append('필수 설정 누락: '+topic)
            continue
        if row.get('count',0)<2:
            errors.append('replay 구간 데이터 누락: '+topic);continue
        expected_end=(min(end_ns,command_end_ns)/1e9) if topic in COMMAND_TOPICS and command_end_ns else end
        if row['first_s']>begin+1.5 or row['last_s']<expected_end-1.5 or row['max_gap_s']>1.5:
            errors.append('데이터 시간 공백: '+topic)
        minimum_hz=2. if topic in IMAGE_TOPICS else 5. if topic in STATE_TOPICS else 20.
        if row['hz']<minimum_hz:errors.append('기록 주기 부족: '+topic)
        if topic in IMAGE_TOPICS:
            if row['bad_source_stamps']:errors.append('영상 원본 시각 중복·역행: '+topic)
            a,b=row.get('source_first_ns'),row.get('source_last_ns')
            if a is None or b is None or abs(a/1e9-row['first_s'])>1.5 or abs(b/1e9-row['last_s'])>1.5:
                errors.append('영상 시계와 ROS 시계 불일치: '+topic)
    return dict(valid=not errors,errors=errors,topics=topic_stats,replay_begin_ros_ns=begin_ns,replay_end_ros_ns=end_ns,
        required_topics=REQUIRED_LIVE+COMMAND_TOPICS,source_stamps_preserved=True)


def inspect_bag(path, event_topic, run_id, rosbag, rospy):
    with rosbag.Bag(str(path),'r') as bag:
        counts={t:v.message_count for t,v in bag.get_type_and_topic_info().topics.items()}
        events=[]
        for _,msg,_ in bag.read_messages(topics=[event_topic]):
            d=json.loads(msg.data)
            if d.get('run_id')==run_id:events.append(d)
        starts=[d for d in events if d.get('event')=='replay_start']
        ends=[d for d in events if d.get('event')=='replay_end']
        if len(starts)!=1 or len(ends)!=1:
            return dict(valid=False,errors=['replay 시작·종료 이벤트 개수 오류'],all_counts=counts,events=events)
        begin,end=starts[0]['ros_time_ns'],ends[0]['ros_time_ns']
        if not isinstance(begin,int) or not isinstance(end,int) or not 0<begin<end:
            return dict(valid=False,errors=['replay 이벤트 ROS 시각 오류'],all_counts=counts)
        stats={}
        selected=[t for t in REQUIRED_LIVE+COMMAND_TOPICS if t not in LATCHED]
        for topic,msg,t in bag.read_messages(topics=selected,start_time=rospy.Time(begin//1000000000,begin%1000000000),end_time=rospy.Time(end//1000000000,end%1000000000)):
            s=stats.setdefault(topic,Stats())
            header=getattr(msg,'header',None)
            stamp=header.stamp.to_nsec() if header is not None else None
            s.add(t.to_sec(),stamp)
        duration=starts[0].get('planned_duration_s')
        command_end=begin+round(duration*1e9) if isinstance(duration,(int,float)) and math.isfinite(duration) and duration>0 else end
        report=quality({t:s.summary() for t,s in stats.items()},counts,begin,end,command_end_ns=command_end)
        report.update(all_counts=counts,events=events,replay_completed=ends[0].get('success') is True)
        return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id',required=True)
    args=ap.parse_args()
    inherited=[]
    team=Path.home()/'rosbag_data.sh'
    if team.is_file():
        probe=subprocess.run(['bash',str(team),'--list'],capture_output=True,text=True,timeout=10)
        if probe.returncode==0:inherited=[t for t in probe.stdout.split() if t.startswith('/') and not t.startswith('//')]
    topics=topics_for_run(args.run_id,inherited)
    root=Path.home()/'kuavo_dataset/rosbags'/time.strftime('%Y-%m-%d')
    root.mkdir(parents=True,exist_ok=True)
    if shutil.disk_usage(root).free<MIN_FREE_BYTES:raise ValueError('녹화 디스크 여유가 10 GiB 미만입니다')
    path=root/('warp_replay_'+args.run_id+'.bag')
    if path.exists() or Path(str(path)+'.active').exists():raise ValueError('이미 있는 녹화 파일')
    import rospy
    import rosbag
    import roslib.message
    from std_msgs.msg import String
    rospy.init_node('warp_replay_capture_'+args.run_id[:12],anonymous=False,disable_signals=True)
    description=rospy.get_param('/robot_description','')
    if not description:raise ValueError('robot_description이 없어 수집 준비를 중단합니다')
    description_path=path.with_suffix('.urdf');description_path.write_text(description)
    description_sha=hashlib.sha256(description.encode()).hexdigest()
    event_topic=EVENT_PREFIX+args.run_id
    publisher=rospy.Publisher(event_topic,String,queue_size=100)
    stop=threading.Event();guard=threading.RLock();events=queue.Queue(maxsize=2048)
    state=dict(last_input=time.monotonic(),problem='',announced=False)
    stats={};type_cache={}
    def stopped(*_):stop.set()
    signal.signal(signal.SIGINT,stopped);signal.signal(signal.SIGTERM,stopped)
    def receive(topic):
        def cb(raw):
            try:
                kind=(raw._connection_header or {}).get('type')
                if kind not in type_cache:type_cache[kind]=roslib.message.get_message_class(kind)
                cls=type_cache[kind];stamp=None
                # Deserialize headers without image decoding; raw image payload remains unchanged in rosbag.
                if cls is not None and cls._slot_types and cls._slot_types[0]=='std_msgs/Header':
                    import struct
                    _,sec,nsec=struct.unpack_from('<III',raw._buff)
                    stamp=sec*1000000000+nsec
                with guard:stats.setdefault(topic,Stats()).add(rospy.Time.now().to_sec(),stamp)
            except Exception as exc:
                with guard:state['problem']='녹화 수신 오류: '+topic+': '+str(exc)
        return cb
    subscribers=[rospy.Subscriber(t,rospy.AnyMsg,receive(t),queue_size=1,buff_size=8*1024*1024) for t in REQUIRED_LIVE]
    def stdin():
        try:
            for line in sys.stdin:
                d=json.loads(line)
                with guard:state['last_input']=time.monotonic()
                if d.get('op')=='stop':break
                if d.get('op')=='event':
                    value=dict(d['data']);value['run_id']=args.run_id
                    events.put_nowait(value)
        except Exception as exc:
            with guard:state['problem']='녹화 연결 오류: '+str(exc)
        finally:stop.set()
    threading.Thread(target=stdin,daemon=True).start()
    node='/warp_replay_bag_'+args.run_id[:12]
    command=['rosbag','record','--buffsize=1024','-O',str(path)]+topics+['__name:='+node.lstrip('/'),'__ns:=/']
    errors=collections.deque(maxlen=20)
    proc=subprocess.Popen(command,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True,start_new_session=True)
    def stderr():
        for line in proc.stderr:
            errors.append(line.strip())
    threading.Thread(target=stderr,daemon=True).start()
    started=time.monotonic();last_report=0.;problem=''
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            now=time.monotonic()
            with guard:
                problem=state['problem'] or ('녹화 heartbeat가 끊겼습니다' if now-state['last_input']>3. else '')
                missing=live_problem(stats,now)
            if proc.poll() is not None:problem='rosbag 녹화가 예기치 않게 종료됐습니다'
            if state['announced'] and missing:problem=missing
            if problem:emit(error=problem);break
            while not events.empty():
                event=events.get_nowait()
                publisher.publish(String(data=json.dumps(event,separators=(',',':'),allow_nan=False)))
            if not state['announced']:
                opened=path.exists() or Path(str(path)+'.active').exists()
                connected=False
                if opened and not missing and publisher.get_num_connections():
                    code,_,system=rospy.get_master().getSystemState()
                    receivers=dict(system[1]) if code==1 else {}
                    connected=all(node in receivers.get(t,[]) for t in REQUIRED_LIVE+COMMAND_TOPICS+[event_topic])
                if connected:
                    state['announced']=True
                    emit(ready=True,path=str(path),topics=topics,ros_time_ns=rospy.Time.now().to_nsec(),monotonic_s=now,run_id=args.run_id)
                elif now-started>25.:raise ValueError(missing or 'bag 구독 준비 확인 시간 초과')
            if now-last_report>1.:
                emit(progress=True,waiting=(missing or 'bag 구독 준비 확인 중') if not state['announced'] else '',
                     bytes=(path.stat().st_size if path.exists() else Path(str(path)+'.active').stat().st_size if Path(str(path)+'.active').exists() else 0))
                last_report=now
            stop.wait(.03)
    finally:
        # Drain queued end/clock events before closing only our own bag process.
        while not events.empty():
            publisher.publish(String(data=json.dumps(events.get_nowait(),separators=(',',':'),allow_nan=False)))
        time.sleep(.15)
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=120)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10);problem=problem or 'bag 종료 시간 초과'
        active=Path(str(path)+'.active').exists()
        report=dict(valid=False,errors=[problem or 'bag 정상 종료 실패'])
        if path.is_file() and not active and path.stat().st_size and proc.returncode in (0,-signal.SIGINT):
            try:report=inspect_bag(path,event_topic,args.run_id,rosbag,rospy)
            except Exception as exc:report=dict(valid=False,errors=['bag 검사 실패: '+str(exc)])
        if problem:
            report['valid']=False;report.setdefault('errors',[]).append(problem)
        digest=None
        if path.is_file():
            h=hashlib.sha256()
            with path.open('rb') as f:
                for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
            digest=h.hexdigest()
        report.update(path=str(path),bytes=path.stat().st_size if path.exists() else 0,sha256=digest,
            finalized=bool(path.is_file() and not active),active_left=active,run_id=args.run_id,recorder_errors=list(errors),
            robot_description_path=str(description_path),robot_description_sha256=description_sha)
        path.with_suffix('.qc.json').write_text(json.dumps(report,indent=2)+'\n')
        emit(done=True,recording=report)
    return 0 if report['valid'] else 1


if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as exc:emit(error=str(exc));raise SystemExit(1)
