"""Episode catalogue and read-only rack-anchored head-camera projection.

The adopted solver/results remain read-only. Physical replay is unavailable
unless standard, selected boundary and live first-state gates pass;
the legacy arm-only recording player cannot reproduce this standard trajectory.
"""
import hashlib
import copy
import itertools
import json
import math
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

from .head_stream import Agent, pose_to_T
from . import standard_rack, first_state, package as playback
from .history import DataDirectory

common = None
datadir = DataDirectory()
ros_setup = []
OBSERVER = Path(__file__).resolve().parents[1]/'observe_replay.py'


def configure(*, control, paths, setup=()):
    global common, datadir, ros_setup, CATALOG
    common = control
    datadir = paths
    ros_setup = list(setup)
    CATALOG = Path(paths.path('retargeting','catalog.json'))

CATALOG = None
LOCK = threading.RLock()
SESSIONS = {}
SELECTIONS = {}
RANGE_DETAILS = {}
DETAILS = {}
STALE_S = 1.5
DRIVER_REASON = "저장된 최종 궤적을 1배속으로 재생합니다. 검증 통과 또는 직접 승인·정지 경계·첫 state·배치 확인이 필요합니다."


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b""):
            h.update(chunk)
    return h.hexdigest()


class HeadAgent(Agent):
    def ensure(self):
        with LOCK:
            self.polled = time.time()
            if self.running():
                return
            link = common.LINK
            if link is None or getattr(link,"host",None)=="offline" or not hasattr(link, "popen_script"):
                return
            if time.time()<getattr(self,"retry_at",0):
                return
            self.retry_at=time.time()+5
            try:
                proc = link.popen_script(str(OBSERVER), [], setup_extra=ros_setup)
            except Exception as exc:
                self._log("head view 연결 실패: %s" % exc)
                return
            self.proc, self.started_at = proc, time.time()
            threading.Thread(target=self._read, args=(proc,), daemon=True).start()
            threading.Thread(target=self._idle, args=(proc,), daemon=True).start()


HEAD = HeadAgent()


def entries():
    if not CATALOG.exists():
        return []
    rows = read(CATALOG).get("episodes", [])
    ids = [r["id"] for r in rows]
    if len(set(ids)) != len(ids):
        raise ValueError("catalogue episode IDs must be unique")
    return rows


def entry(key):
    for row in entries():
        if row["id"] == key:
            return row
    raise ValueError("episode를 찾을 수 없습니다")


def catalogue():
    result = []
    for row in entries():
        r = Path(row["result"]) if row.get("result") else None
        report = read(r/"REPORT.json") if r and (r/"REPORT.json").exists() else {}
        video = Path(row["video"]) if row.get("video") else None
        result.append(dict(id=row["id"], label=row.get("label", row["id"]),
                           status=report.get("status", "not_computed"), has_video=bool(video and video.is_file()),
                           review_duration_s=row.get("review_duration_s"),
                           duration_s=(report.get("search",{}).get("timing",{}) or {}).get("duration_s"),
                           trimmed_source=row.get("trimmed_source", False),
                           coordinate_frame=row.get('coordinate_frame','recording'),calibration_at=row.get('calibration_at')))
    return {"episodes": result, "driver_ready": True, "driver_reason": DRIVER_REASON}


def _first_valid(path):
    with Path(path).open() as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("valid") and row.get("q") is not None:
                return row
    raise ValueError("유효한 시작 자세 없음")


def _scene_at(path, source_s):
    best = None
    with Path(path).open() as stream:
        for line in stream:
            row = json.loads(line)
            if best is None or abs(row["elapsed_s"]-source_s) < abs(best["elapsed_s"]-source_s):
                best = row
            if row["elapsed_s"] > source_s:
                break
    if best is None or abs(best["elapsed_s"]-source_s) > .1:
        raise ValueError("시작 시점의 환경 배치 없음")
    return best


def scene_boxes(scene):
    world = {o["id"]:o for o in scene.get("world",[])}
    # The review provider stores whole boxes; the collision world stores shells.
    # Prefer the whole recorded geometry, preserving its actual dimensions.
    boxes = [{k:o[k] for k in ("id","position","quaternion_xyzw","dimensions")}
             for o in world.values() if o["id"].startswith("box_") and o["id"].endswith("_A13")]
    present={b["id"] for b in boxes}
    for box_id in sorted({k.rsplit("_",1)[0] for k in world if k.startswith("box_")}):
        left,right = world.get(box_id+"_left"),world.get(box_id+"_right")
        if left and right and box_id not in present:
            width=float(np.linalg.norm(np.array(left["position"])-right["position"]))+left["dimensions"][0]
            boxes.append(dict(id=box_id,position=((np.array(left["position"])+right["position"])/2).tolist(),
                              quaternion_xyzw=left["quaternion_xyzw"],dimensions=[width,*left["dimensions"][1:]]))
    return sorted(boxes,key=lambda b:b["id"])


def rotation_quaternion(M):
    # Stable largest-component conversion, XYZW convention; NumPy only.
    d=np.array([1+M[0,0]-M[1,1]-M[2,2],1-M[0,0]+M[1,1]-M[2,2],
                1-M[0,0]-M[1,1]+M[2,2],1+np.trace(M)])
    i=int(np.argmax(d));q=np.zeros(4);q[i]=np.sqrt(max(0.,d[i]))/2
    scale=4*q[i]
    if i==3:q[:3]=[M[2,1]-M[1,2],M[0,2]-M[2,0],M[1,0]-M[0,1]]
    else:
        j,k=(i+1)%3,(i+2)%3
        q[j]=M[j,i]+M[i,j];q[k]=M[k,i]+M[i,k];q[3]=M[k,j]-M[j,k]
    for j in range(4):
        if j!=i:q[j]/=scale
    return (q/np.linalg.norm(q)).tolist()


def transform_boxes(boxes,transform):
    T=np.array(transform)
    result=copy.deepcopy(boxes)
    for box in result:
        box['position']=(T[:3,:3]@np.array(box['position'])+T[:3,3]).tolist()
        M=T[:3,:3]@pose_to_T([0,0,0]+box['quaternion_xyzw'])[:3,:3]
        box['quaternion_xyzw']=rotation_quaternion(M)
    return result


def load_detail(key):
    e = entry(key)
    prepared = read(e["prepared"])
    provider = Path(prepared["display_source"])
    result = Path(e["result"]) if e.get("result") else None
    reference = result/"data/source-trajectory.jsonl" if result else Path(prepared["reference"])
    paths = [Path(e['prepared']), reference, provider/"data/frame-scenes.jsonl", provider/"data/rack-preview-anchor.json", provider/"tag_policy.json", provider/"object_overlay_config.json", standard_rack.SOURCE]
    if result:
        paths.append(result/"REPORT.json")
        if (result/"results/execution-trajectory-100hz.npz").is_file():
            paths.append(result/"results/execution-trajectory-100hz.npz")
    from . import timeline
    try:
        timeline_stat = timeline.signature(e)
    except (OSError, ValueError, KeyError):
        timeline_stat = ()
    stat = (json.dumps(e, sort_keys=True),) + tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in paths) + timeline_stat
    cached = DETAILS.get(key)
    if cached and cached[0] == stat:
        return cached[1]
    first = _first_valid(reference)
    names = ["knee_joint","leg_joint","waist_pitch_joint","waist_yaw_joint"] + ["zarm_%s%d_joint"%(s,i) for s in "lr" for i in range(1,8)]
    initial = {"source_s": first["elapsed_s"], "base_pose": first["base_pose"],
               "joint_names": names, "q": first["q"], "grippers": first.get("grippers")}
    # Use the final retimed initial position, not the IK seed/reference pose.
    if result and (result/"results/execution-trajectory-100hz.npz").is_file():
        with np.load(result/"results/execution-trajectory-100hz.npz", allow_pickle=False) as z:
            initial.update(source_s=float(z["source_s"][0]), base_pose=z["position"][0,:3].tolist(),
                           q=z["position"][0,3:].tolist(), joint_names=z["names"][3:].tolist())
    scene = _scene_at(provider/"data/frame-scenes.jsonl", initial["source_s"])
    boxes = scene_boxes(scene)
    coordinate_preparation=prepared.get('coordinate_preparation')
    if coordinate_preparation:
        boxes=transform_boxes(boxes,coordinate_preparation['T_base_recording'])
    anchor = read(provider/"data/rack-preview-anchor.json")
    policy = read(provider/"tag_policy.json")
    overlay_config=read(provider/"object_overlay_config.json")
    fit_settings={k:overlay_config[k] for k in ["pose_selection","multi_tag_pose"]}
    fit_settings["tag_size_m"]=overlay_config["rack"]["tag_size_m"]
    offsets = anchor.get("tag_offsets") or policy["rack"]["tag_offsets_in_geometry_tag_axes_m"]
    T = np.eye(4);T[:3,:3]=anchor["rotation"];T[:3,3]=anchor["position"]
    rack_tags = {}
    for tag,offset in offsets.items():
        M = T.copy();M[:3,3] = T[:3,3]+T[:3,:3]@offset
        rack_tags[tag] = M.tolist()
    if coordinate_preparation:
        T_base_recording=np.array(coordinate_preparation['T_base_recording'])
        rack_tags={k:(T_base_recording@np.array(v)).tolist() for k,v in rack_tags.items()}
    report = read(result/"REPORT.json") if result else {}
    fingerprint = hashlib.sha256(json.dumps({"stat":stat,"initial":initial,"boxes":boxes,"rack":rack_tags},sort_keys=True).encode()).hexdigest()
    out = dict(id=key,label=e.get("label",key),fingerprint=fingerprint,initial=initial,boxes=boxes,rack_tags=rack_tags,
               coordinate_preparation=coordinate_preparation,rack_fit_settings=fit_settings, status=report.get("status","not_computed"), has_video=bool(e.get("video") and Path(e["video"]).is_file()),
               review_duration_s=e.get("review_duration_s"),trimmed_source=e.get("trimmed_source",False),
               environment_source_s=scene["elapsed_s"],driver_ready=True,driver_reason=DRIVER_REASON)
    try:
        out['timeline'] = timeline.public(timeline.load(e))
    except (ValueError, OSError, KeyError, IndexError, TypeError, subprocess.SubprocessError) as exc:
        out['timeline'] = dict(available=False, reason='구간 선택 불가: '+str(exc))
    DETAILS[key] = (stat,out)
    return out


def selection_detail(key, sid=None):
    """Per-page target; the adopted trajectory and catalogue are never rewritten."""
    base = load_detail(key)
    if not base.get('timeline', {}).get('available'):
        return base
    from . import timeline
    data = timeline.load(entry(key))
    now = time.time()
    with LOCK:
        for k in [k for k, v in SELECTIONS.items() if now-v['at'] > 86400]:
            SELECTIONS.pop(k, None)
        saved = SELECTIONS.get((sid, key), {})
        selected = saved.get('selection') if saved.get('base_fingerprint') == base['fingerprint'] else None
    if selected is None:
        selected = timeline.interval(data, 0, data['points'][-1]['video_s'])
    fingerprint = hashlib.sha256(json.dumps([base['fingerprint'], data['fingerprint'],
        selected['start']['video_s'], selected['end']['video_s']]).encode()).hexdigest()
    with LOCK:
        if fingerprint in RANGE_DETAILS:
            return RANGE_DETAILS[fingerprint]
    out = copy.deepcopy(base)
    out['episode_fingerprint'] = base['fingerprint']
    out['selection'] = selected
    out['fingerprint'] = fingerprint
    out['initial'].update(timeline.initial(data, selected['start']['execution_s']))
    prepared = read(entry(key)['prepared'])
    provider = Path(prepared['display_source'])
    scene = _scene_at(provider/'data/frame-scenes.jsonl', out['initial']['source_s'])
    out['boxes'] = scene_boxes(scene)
    if prepared.get('coordinate_preparation'):
        out['boxes']=transform_boxes(out['boxes'],prepared['coordinate_preparation']['T_base_recording'])
    out['environment_source_s'] = scene['elapsed_s']
    reference = Path(entry(key)['result'])/'data/source-trajectory.jsonl'
    source = _scene_at(reference, out['initial']['source_s'])
    out['initial']['grippers'] = source.get('grippers')
    with LOCK:
        if len(RANGE_DETAILS) >= 16:
            RANGE_DETAILS.pop(next(iter(RANGE_DETAILS)))
        RANGE_DETAILS[fingerprint] = out
    return out


def select_range(key, sid, fingerprint, start, end):
    from . import session as alignment, timeline
    with alignment.LOCK:
        if alignment.status(sid)['running']:
            raise ValueError('자동 정렬을 정지한 뒤 구간을 변경하세요.')
        current = selection_detail(key, sid)
        if current['fingerprint'] != fingerprint:
            raise ValueError('선택 구간이나 episode가 바뀌었습니다. 다시 불러오세요.')
        selected = timeline.interval(timeline.load(entry(key)), start, end)
        with LOCK:
            SELECTIONS[(sid, key)] = dict(base_fingerprint=load_detail(key)['fingerprint'],
                                         selection=selected, at=time.time())
            SESSIONS.pop(sid, None)
        detail = selection_detail(key, sid)
    path = Path(datadir.path('retargeting', 'events.jsonl'))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as stream:
        stream.write(json.dumps(dict(t=time.time(), episode=key, op='select_range',
            fingerprint=detail['fingerprint'], selection=selected))+'\n')
    return dict(ok=True, detail=detail, text='선택 구간을 저장했습니다. 정렬·배치와 구간 출발·정지 검증이 필요합니다.')


def project(points,T,K,D):
    """OpenCV optical axes; plumb_bob/rational distortion on raw RGB pixels."""
    p = np.asarray(points)@T[:3,:3].T+T[:3,3]
    good = p[:,2] > .02
    xy = p[:,:2]/np.maximum(p[:,2,None],.0001)
    x,y = xy.T;r2=x*x+y*y
    d=np.zeros(8);d[:min(len(D),8)] = D[:8]
    k1,k2,p1,p2,k3,k4,k5,k6=d
    radial=(1+k1*r2+k2*r2**2+k3*r2**3)/(1+k4*r2+k5*r2**2+k6*r2**3)
    xd=x*radial+2*p1*x*y+p2*(r2+2*x*x)
    yd=y*radial+p1*(r2+2*y*y)+2*p2*x*y
    uv=np.column_stack([K[0,0]*xd+K[0,2],K[1,1]*yd+K[1,2]])
    return uv,good


def rack_transform(detail,sample):
    q=standard_rack.estimate(detail,sample)
    T=np.array(q['T_camera_recording'])
    # Pixel residual is unavailable in AprilTagDetectionArray. Do not invent corners.
    if q.get('detector')!='apt_apriltag_ros' or q['rms_m']>.05:
        raise ValueError('rwh 랙 관측을 신뢰할 수 없습니다')
    return T,q['tag_ids'],None


EDGES=[(a,b) for a in range(8) for b in range(a+1,8) if bin(a^b).count("1")==1]


def overlay(detail,sample,now=None):
    now=time.time() if now is None else now
    age=now-sample.get("received",0)+sample.get("image_age_s",0)
    out=dict(ok=False,age_s=age,frame_id=sample.get("frame_id"),width=sample.get("width",640),
             height=sample.get("height",480),jpeg=sample.get("jpeg"),boxes=[],robot_error=None,tf_error=sample.get("tf_error"))
    if age>=STALE_S or age<0:
        out["message"]="태그 관측이 오래됐습니다. 최신 head 영상은 계속 표시합니다."
        return out
    if sample.get('tf_ready') is False:
        out['message']=sample.get('tf_status','좌표 연결 준비 중')
        return out
    try:
        fit=standard_rack.estimate(dict(detail,allow_single_tag_projection=True),sample)
        single=bool(fit.get('projection_only'))
        if single:
            T,ids,rms=np.array(fit['T_camera_recording']),fit['tag_ids'],None
        else:
            T,ids,rms=rack_transform(detail,sample)
        K=np.array(sample["K"]).reshape(3,3);D=sample.get("D",[])
        for box in detail["boxes"]:
            signs=np.array(list(itertools.product([-1,1],repeat=3)))
            M=pose_to_T(box["position"]+box["quaternion_xyzw"])
            pts=(signs*np.array(box["dimensions"])/2)@M[:3,:3].T+M[:3,3]
            uv,good=project(pts,T,K,D)
            out["boxes"].append(dict(id=box["id"],corners=uv.tolist(),edges=[(a,b) for a,b in EDGES if good[a] and good[b]],
                                     visible=bool(np.any(good))))
        if sample.get("base_camera_pose") and not single:
            base_world=np.linalg.inv(T)@np.linalg.inv(pose_to_T(sample["base_camera_pose"]))
            target=detail["initial"]["base_pose"]
            yaw=math.atan2(base_world[1,0],base_world[0,0])
            dx,dy=target[0]-base_world[0,3],target[1]-base_world[1,3]
            relative=[math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy]
            dyaw=math.atan2(math.sin(target[2]-yaw),math.cos(target[2]-yaw))
            out["robot_error"]=dict(forward_m=float(relative[0]),left_m=float(relative[1]),yaw_deg=math.degrees(dyaw))
        label="RWH 랙 태그 %d개 정합" % len(ids)
        if fit["method"]=="single_tag_fallback":label+=" (다중 추정 실패)"
        message=("RWH 랙 태그 %s 단일 투영 · 정렬은 다중 태그 대기" % ids[0] if single else "%s %s · 피팅 %.1fmm"%(label,"·".join(map(str,ids)),fit["rms_m"]*1000))
        out.update(ok=not single,projection_ok=True,projection_only=single,rack_ids=ids,rack_fit={k:v for k,v in fit.items() if k not in ("T_camera_recording","raw_tag_corners")},rms_px=None,message=message,
                   tf_error=sample.get("tf_error"))
    except (ValueError,KeyError,np.linalg.LinAlgError) as exc:
        out["message"]=str(exc)
    return out


def stationary_status(snapshot):
    out=dict(threshold_deg_s=3.,available=False,maximum_deg_s=None,joint=None)
    try:
        sensor=snapshot['topics']['/sensors_data_raw']
        if not first_state.topic_fresh(sensor,snapshot,time.time(),.5):return out
        values=first_state.vector(sensor['data']['joint_data']['joint_v'][:18],18,'실측 속도')
        j=max(range(18),key=lambda i:abs(values[i]))
        out.update(available=True,maximum_deg_s=math.degrees(abs(values[j])),joint=first_state.NAMES[j])
    except (ValueError,KeyError,TypeError):pass
    return out


def head_display(now=None):
    """Metadata for the same head buffer served by /kuavo/cam/head."""
    now=time.time() if now is None else now
    link=common.LINK if common is not None else None
    entry=link.frame('head') if link is not None and hasattr(link,'frame') else None
    if not entry or not entry.get('jpeg'):
        return dict(ok=False,message='head 카메라 연결 중…')
    age=now-float(entry.get('t') or 0)+float(entry.get('age') or 0)
    return dict(ok=0<=age<STALE_S,age_s=age,frame_id='dashboard-head-'+str(entry['seq']),
                seq=entry['seq'],url='/kuavo/cam/head',source='dashboard_head',
                stamp_ns=entry.get('stamp_ns'))


def state(key,sid):
    from . import session as alignment
    detail=selection_detail(key,sid)
    HEAD.ensure()
    sample,_,_,logs,_=HEAD.snapshot()
    live=overlay(detail,sample) if sample else {"ok":False,"message":"랙 태그 관측 대기 중 · head 영상은 별도 갱신","boxes":[]}
    with LOCK:
        ss=SESSIONS.get(sid,{})
        valid=ss.get("id")==key and ss.get("fingerprint")==detail["fingerprint"]
        identity_valid=valid
        checks=dict(ss.get("checks",{})) if valid else {}
        manual_approval=identity_valid and checks.get("validation_override") is True
        error=live.get('robot_error')
        frozen_base=ss.get('base_goal_odom') is not None
        feedback_wait=False
        if checks.get('robot_manual') and frozen_base:
            snapshot=common.LINK.snapshot if common.LINK else None
            try:
                feedback_wait=not all(first_state.topic_fresh(snapshot['topics'][topic],snapshot,time.time(),limit)
                    for topic,limit in (('/odom',.5),('/sensors_data_raw',1.),('/leju_claw_state',1.)))
            except (KeyError,TypeError,ValueError):feedback_wait=True
        if checks.get('robot'):
            if frozen_base and not feedback_wait:
                if not first_state.base_feedback_matches(ss['base_goal_odom'],ss.get('base_odom_frame'),
                        common.LINK.snapshot if common.LINK else None,time.time()):valid=False
            elif not frozen_base and (not error or math.hypot(error['forward_m'],error['left_m'])>.04 or abs(error['yaw_deg'])>4):
                valid=False
        if checks.get('robot') and not feedback_wait and not first_state.feedback_matches(ss.get('first_state_target',{}),
                                                                    common.LINK.snapshot if common.LINK else None,time.time()):
            valid=False
        if not valid or (not live["ok"] and not frozen_base):
            checks={"validation_override":True} if manual_approval else {}
            if manual_approval:
                saved=dict(ss);saved["checks"]=checks;saved.pop("first_state_target",None)
                SESSIONS[sid]=saved
            else:SESSIONS.pop(sid,None)
    reasons=[]
    if feedback_wait and checks.get('robot'):
        reasons.append('로봇 피드백 수신 지연 · 도착 확인은 보관하며 최신 피드백까지 replay를 차단합니다.')
    if detail["status"]!="verified" and not checks.get("validation_override"): reasons.append("선택한 궤적의 standard 검증이 통과되지 않았습니다.")
    if not detail["trimmed_source"]: reasons.append("녹화 끝 2초 제외 후 실행 궤적 재검증이 필요합니다.")
    if detail.get('selection') and (detail['status']=='verified' or checks.get('validation_override')):
        why=playback.reason(entry(key),detail,allow_unverified=checks.get('validation_override') is True)
        if why:reasons.append(why)
    elif not detail.get('selection'):reasons.append('영상에서 실행 구간을 선택하세요.')
    if not checks.get('robot'):reasons.append('첫 state 도착·정지를 먼저 확인하세요.')
    if not checks.get('boxes'):reasons.append('시작 박스 배치를 확인하세요.')
    if not live['ok']:reasons.append('실행 전 최신 랙 태그·TF 관측을 확인하세요.')
    align=alignment.status(sid)
    busy_reason=alignment.unavailable() if not align['running'] else 'replay 진행 중' if align.get('kind')=='replay' else '자동 정렬 진행 중'
    if busy_reason:reasons.append(busy_reason)
    align_reason=busy_reason
    try:first_state.target(detail)
    except (ValueError,KeyError,TypeError) as exc:align_reason=str(exc)
    if not live.get('robot_error'):
        align_reason='새 랙 영상과 head→base_link TF를 기다리는 중입니다.'
    if not live['ok']:align_reason=live.get('message','랙 영상 없음')
    return {"id":key,"fingerprint":detail["fingerprint"],"selection":detail.get('selection'),"live":live,"camera":head_display(),"checks":checks,"ready":not reasons,"blockers":reasons,"logs":logs[-3:],
            "stationary":stationary_status(common.LINK.snapshot if common.LINK else None),"validation_status":detail["status"],"alignment":align,"align_available":not bool(align_reason),"align_reason":align_reason}


def confirm(key,sid,fingerprint,step,value):
    if step not in ("robot","boxes","validation_override") or not isinstance(value,bool):
        raise ValueError("정렬 확인 항목이 올바르지 않습니다")
    if step=='validation_override':
        from . import session as alignment
        if alignment.status(sid)['running']:raise ValueError('실행 중에는 검증 승인을 변경할 수 없습니다.')
    from . import session as alignment
    active=alignment.status(sid)
    if step=='robot' and not value and active['running'] and active['mine']:
        alignment.stop()
    st=state(key,sid)
    if fingerprint!=st["fingerprint"]:
        raise ValueError("episode 결과가 바뀌었습니다. 다시 선택하세요.")
    if value and not st["live"]["ok"]:
        raise ValueError("새 head 영상과 랙 태그를 먼저 확인하세요.")
    if value and step=="boxes" and not st["checks"].get("robot"):
        raise ValueError("로봇 정렬을 먼저 확인하세요.")
    verified_target=None;verified_base=None;verified_frame=None
    if value and step=='robot':
        if active['running']:raise ValueError('정렬·replay 실행이 끝난 뒤 도착을 확인하세요.')
        detail=selection_detail(key,sid);verified_target=first_state.target(detail)
        snapshot=common.LINK.snapshot if common.LINK else None;now=time.time()
        topics=snapshot['topics']
        if not all(k in topics for k in ('/sensors_data_raw','/leju_claw_state','/odom')):
            raise ValueError('최신 베이스·관절·그리퍼 피드백을 확인하세요.')
        sensor=topics['/sensors_data_raw'];claw=topics['/leju_claw_state']
        if not all(first_state.topic_fresh(t,snapshot,now,1.) for t in (sensor,claw)):
            raise ValueError('최신 몸통·팔·그리퍼 피드백을 확인하세요.')
        q=sensor['data']['joint_data']
        # Manual approval accepts the displayed error. Anchor actual feedback
        # solely for drift revocation; never modify the selected replay target.
        verified_target['q']=first_state.vector(q['joint_q'][:18],18,'수동 확인 관절')
        verified_target['claw']=first_state.vector(claw['data']['data']['position'][:2],2,'수동 확인 그리퍼')
        odom=snapshot['topics'].get('/odom',{})
        if not first_state.topic_fresh(odom,snapshot,now,.5):
            raise ValueError('최신 베이스 odom을 확인하세요.')
        data=odom['data'];position=data['pose']['pose']['position'];orientation=data['pose']['pose']['orientation']
        yaw=math.atan2(2*(orientation['w']*orientation['z']+orientation['x']*orientation['y']),
                      1-2*(orientation['y']**2+orientation['z']**2))
        twist=data['twist']['twist']
        if math.hypot(twist['linear']['x'],twist['linear']['y'])>.005 or abs(twist['angular']['z'])>math.radians(.5):
            raise ValueError('베이스가 정지한 뒤 확인하세요.')
        # The live rack verifies placement; odom anchors this measured,
        # accepted pose for subsequent drift detection. This sends no command.
        verified_base=[position['x'],position['y'],yaw];verified_frame=data['header']['frame_id']
    with LOCK:
        ss=SESSIONS.setdefault(sid,{"id":key,"fingerprint":fingerprint,"checks":{}})
        if ss["id"]!=key or ss["fingerprint"]!=fingerprint:
            ss={"id":key,"fingerprint":fingerprint,"checks":{}};SESSIONS[sid]=ss
        ss["at"]=time.time();ss["checks"][step]=value
        if verified_target is not None:
            ss.update(first_state_target=verified_target,base_goal_odom=verified_base,base_odom_frame=verified_frame)
            ss['checks']['robot_manual']=True
        if step=="robot" and not value:
            ss["checks"].pop("boxes",None);ss["checks"].pop("robot_manual",None)
        path=Path(datadir.path("retargeting","events.jsonl"));path.parent.mkdir(parents=True,exist_ok=True)
        with path.open("a") as stream:
            stream.write(json.dumps(dict(t=time.time(),episode=key,fingerprint=fingerprint,step=step,checked=value,manual=(step=="robot" and value)))+"\n")
    return {"ok":True}


def align_start(key,sid,fingerprint,stationary_confirmed=False):
    from . import session as alignment
    detail=selection_detail(key,sid)
    if detail['fingerprint']!=fingerprint:raise ValueError('episode 결과가 바뀌었습니다. 다시 선택하세요.')
    HEAD.ensure();sample,*_=HEAD.snapshot()
    if not sample:raise ValueError('랙 영상을 기다리는 중입니다.')
    live=overlay(detail,sample)
    with LOCK:
        saved=SESSIONS.get(sid,{})
        approval=(saved.get('id')==key and saved.get('fingerprint')==fingerprint
                  and saved.get('checks',{}).get('validation_override') is True)
        SESSIONS.pop(sid,None)
        if approval:SESSIONS[sid]=dict(id=key,fingerprint=fingerprint,at=saved['at'],checks={'validation_override':True})
    return {'ok':True,'alignment':alignment.start(detail,sid,sample,live,stationary_confirmed=stationary_confirmed is True),'text':'베이스 정렬과 선택 구간의 첫 자세 이동을 함께 진행합니다. head 관절은 제외합니다.'}


def replay(key,sid,fingerprint,stationary_confirmed=False):
    from . import session as alignment
    with alignment.LOCK:
        # Check readiness once inside the same global motion lock.
        again=state(key,sid)
        if again.get('fingerprint')!=fingerprint:raise ValueError('선택 구간이 바뀌었습니다.')
        if not again.get('ready'):raise ValueError(' / '.join(again['blockers']))
        detail=selection_detail(key,sid)
        if detail['fingerprint']!=fingerprint:raise ValueError('선택 구간이 바뀌었습니다.')
        plan=playback.prepare(entry(key),detail,fresh=True,allow_unverified=again.get('checks',{}).get('validation_override') is True)
        HEAD.ensure();sample,*_=HEAD.snapshot()
        if not sample:raise ValueError('새 랙 영상이 없습니다.')
        live=overlay(detail,sample)
        result=alignment.start(detail,sid,sample,live,replay_plan=plan,stationary_confirmed=stationary_confirmed is True)
        with LOCK:SESSIONS.pop(sid,None)
    return dict(ok=True,alignment=result,text='선택한 최종 궤적을 1배속으로 replay합니다. head 관절은 제외합니다.')


def video_path(key):
    e=entry(key)
    if not e.get("video") or not Path(e["video"]).is_file():
        raise ValueError("4-window 영상이 아직 없습니다")
    return Path(e["video"])
