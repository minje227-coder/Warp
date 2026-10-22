"""Read existing validation artifacts; reproduce their gates without new IK."""
import json
import math
from pathlib import Path
import threading
import xml.etree.ElementTree as ET

CACHE={}
LOCK=threading.Lock()


def read(p):return json.loads(Path(p).read_text())


def summarize(result):
    result=Path(result);report=read(result/'REPORT.json')
    cfgpath=result/'data/source-config.json'
    cfg=read(cfgpath).get('ik',{}) if cfgpath.exists() else {}
    validation=report.get('validation',{});moveit=report.get('moveit',{})
    out=dict(status=report.get('status','unknown'),result=str(result),reasons=[],
             invalid_samples=validation.get('sampled_state_invalid'),sampled_states=validation.get('sampled_states'),
             source_ik_accepted=report.get('search',{}).get('metrics',{}).get('accepted'),
             checks=dict(collision_failures=validation.get('clearance_below10mm_minus1um_states'),
                 source_knot_failures=validation.get('source_knot_invalid'),
                 moveit_invalid=moveit.get('accepted_invalid'),moveit_interframe_invalid=moveit.get('interframe_invalid'),
                 velocity_ratio=validation.get('max_sampled_velocity_ratio'),acceleration_ratio=validation.get('max_sampled_acceleration_ratio')))
    groups={};covered=set()
    def add(code,label,sample,value=None,limit=None,excess=None,unit='',subject=None,operator='≤'):
        g=groups.setdefault(code,dict(code=code,label=label,_indices=set(),_times=[],subjects=set(),
            measured=None,limit=None,excess=None,unit=unit,operator=operator,worst_execution_s=None))
        index=sample['index'];g['_indices'].add(index);covered.add(index)
        if sample.get('execution_s') is not None:g['_times'].append(sample['execution_s'])
        if subject:g['subjects'].add(subject)
        if excess is not None and (g['excess'] is None or excess>g['excess']):
            g.update(measured=value,limit=limit,excess=excess,worst_execution_s=sample.get('execution_s'),worst_subject=subject,operator=operator)
    path=result/'results/drake-validation.json'
    dense=read(path) if path.exists() else {}
    bad=[s for s in dense.get('samples',[]) if not s.get('check',{}).get('valid',True)]
    for sample in bad:
        check=sample['check'];allow=sample.get('position_allowance_m')
        for i,e in enumerate(check.get('tcp_errors',[])):
            side='왼손' if i==0 else '오른손'
            pos=e.get('position_m')
            if allow is not None and pos is not None and pos>allow+1e-5:
                add('tcp_position','손끝 목표 위치 오차',sample,pos*1000,allow*1000,(pos-allow)*1000,'mm',side)
            if e.get('orientation_mode')=='body_downward':
                angle=e.get('body_down_angle_rad');limit=math.radians(cfg.get('body_down_max_deg',60))
                if angle is not None and angle>limit+1e-5:
                    add('downward','그리퍼 하향 각도',sample,math.degrees(angle),math.degrees(limit),math.degrees(angle-limit),'°',side)
                up=e.get('camera_mount_up_projection')
                if cfg.get('wrist_camera_flip_only'):
                    if up is not None and up < -1e-5:
                        angle=math.degrees(math.asin(min(1.,max(-1.,-up))))
                        add('camera_flip','손목 카메라 뒤집힘 경계',sample,angle,0.,angle,'°',side)
                else:
                    angle=e.get('closing_axis_error_rad');limit=cfg.get('tcp_orientation_tolerance_rad')
                    if angle is not None and limit is not None and angle>limit+1e-5:
                        add('roll','그리퍼 닫힘축 방향',sample,math.degrees(angle),math.degrees(limit),math.degrees(angle-limit),'°',side)
                forward=e.get('body_forward_projection')
                if forward is not None and forward < -1e-5:
                    angle=math.degrees(math.asin(min(1.,max(-1.,-forward))))
                    add('forward','그리퍼 전방 방향 경계',sample,angle,0.,angle,'°',side)
            elif e.get('orientation_mode')!='box_inward':
                angle=e.get('orientation_rad');limit=cfg.get('tcp_orientation_tolerance_rad')
                if angle is not None and limit is not None and angle>limit+1e-5:
                    add('orientation','그리퍼 목표 방향 오차',sample,math.degrees(angle),math.degrees(limit),math.degrees(angle-limit),'°',side)
        angle=check.get('mutual_tcp_orientation_error_rad');limit=cfg.get('mutual_tcp_orientation_tolerance_rad')
        if angle is not None and limit is not None and angle>limit+1e-5:
            add('parallel','양손 평행 각도',sample,math.degrees(angle),math.degrees(limit),math.degrees(angle-limit),'°')
        distance=sample.get('minimum_distance_m');margin=cfg.get('collision_margin_m',.01)
        if distance is not None and distance<margin-1e-6:
            add('collision','충돌 안전거리 부족',sample,distance*1000,margin*1000,(margin-distance)*1000,'mm',operator='≥')
    # Dense samples omit q. The exact matching MoveIt audit supplies q and its
    # explicit index. Read its URDF limits and apply the same joint safety margin.
    jointpath=result/'moveit_verify/data/checks.json';urdf=result/'validation-model/kuavo_s63_retarg.urdf'
    if bad and jointpath.exists() and urdf.exists():
        joints=read(jointpath);limits={}
        for j in ET.parse(urdf).getroot().findall('joint'):
            limit=j.find('limit')
            if limit is not None and 'lower' in limit.attrib and 'upper' in limit.attrib:
                margin=cfg.get('joint_margin_rad',0.)
                limits[j.attrib['name']]=(float(limit.attrib['lower'])+margin,float(limit.attrib['upper'])-margin)
        wanted={s['index']:s for s in bad}
        for row in joints.get('samples',[]):
            sample=wanted.get(row['index'])
            if sample is None:continue
            for name,q in zip(joints['joint_names'],row['q']):
                if name not in limits:continue
                lo,hi=limits[name]
                if q<lo-1e-7:add('joint_guard','관절 허용 범위 (안전 여유 포함)',sample,math.degrees(q),math.degrees(lo),math.degrees(lo-q),'°',name,'≥')
                if q>hi+1e-7:add('joint_guard','관절 허용 범위 (안전 여유 포함)',sample,math.degrees(q),math.degrees(hi),math.degrees(q-hi),'°',name,'≤')
    for sample in bad:
        if sample['index'] not in covered:add('other','기타 자세 제약 위반 · 세부 보고서 확인',sample,subject=sample['check'].get('reason'))
    for group in groups.values():
        group['samples']=len(group.pop('_indices'));times=group.pop('_times')
        group['first_execution_s']=min(times) if times else None;group['last_execution_s']=max(times) if times else None
        group['subjects']=sorted(group['subjects'],key=lambda x:(0 if x=='왼손' else 1 if x=='오른손' else 2,x));out['reasons'].append(group)
    for code,label,count in [
        ('source_knots','원본 IK 자세 검증 실패',validation.get('source_knot_invalid',0)),
        ('moveit','MoveIt 충돌·관절·FK 검증 실패',moveit.get('accepted_invalid',0)),
        ('moveit_between','MoveIt 중간 자세 검증 실패',moveit.get('interframe_invalid',0))]:
        if count:out['reasons'].append(dict(code=code,label=label,samples=count))
    for name,label in [('max_sampled_velocity_ratio','속도 제한 초과'),('max_sampled_acceleration_ratio','가속도 제한 초과')]:
        ratio=validation.get(name)
        if ratio is not None and ratio>1.00001:out['reasons'].append(dict(code=name,label=label,measured=ratio,limit=1.,excess=ratio-1.,unit='배',operator='≤'))
    failures=report.get('search',{}).get('failures',[])
    if failures:out['reasons'].append(dict(code='ik',label='IK 해를 구하지 못한 입력 프레임',samples=len(failures)))
    if out['status']!='verified' and not out['reasons']:
        out['reasons'].append(dict(code='report',label='세부 실패 수치가 없는 결과: '+out['status']))
    return out


def detail(key):
    from . import compute
    entries={e['id']:e for e in read(compute.CATALOG)['episodes']}
    if key not in entries:raise ValueError('episode를 찾을 수 없습니다.')
    e=entries[key];result=Path(e['result']) if e.get('result') else None
    if result and (result/'REPORT.json').is_file():
        paths=[result/p for p in ('REPORT.json','data/source-config.json','results/drake-validation.json','moveit_verify/data/checks.json','validation-model/kuavo_s63_retarg.urdf')]
        stamp=tuple((str(p),p.stat().st_mtime_ns,p.stat().st_size) for p in paths if p.exists())
        with LOCK:
            cached=CACHE.get(key)
            if not cached or cached[0]!=stamp:CACHE[key]=(stamp,summarize(result))
            out=dict(CACHE[key][1])
    else:out=dict(status='not_computed',reasons=[])
    job=compute.jobs().get(key,{})
    if job.get('status')=='failed':out['compute_error']=job.get('error') or job.get('log_tail') or job.get('stage') or '계산 오류 · 계산 로그 확인'
    return out
