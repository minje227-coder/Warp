"""Source-time box-relative grasp latch, shared by preview and replay preparation.

Human thumb/index tip midpoint is measured against the tracked box's local +Z
rim. XY only associates a nearby box; a fingertip need not lie inside its walls.
No ROS or motion IO. Missing geometry never releases an already closed hand.
"""
import bisect
import hashlib
import json
from functools import lru_cache
from pathlib import Path

import numpy as np

POLICY = dict(kind='box_relative_grasp_v1', close_at_open_fraction=.70,
              release_above_open_fraction=.50, release_aperture_increase=.12, close_depth_m=.003,
              association_margin_m=.06, close_confirm_s=.08,
              release_confirm_s=.04, release_displacement_m=.005,
              release_relative_speed_m_s=.015,
              release_near_rim_depth_m=.025, history_s=.20,
              regrasp_close_fraction=.35, rearm_outside_depth_m=-.005,
              max_gap_s=.25, target_open_fraction=0.,
              indicator='red_finger_tips', time_axis='source_s',
              fingertip='human thumb/index midpoint; unmodified recorded human_points',
              depth_axis='box local +Z; positive depth below top rim',
              calibration_verified=False)


def quaternion_matrix(quaternion):
    q=np.asarray(quaternion,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise ValueError('Invalid box quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def local_tip(tip, box):
    return quaternion_matrix(box['quaternion_xyzw']).T @ (np.asarray(tip)-box['position'])


def metrics(local, dimensions):
    dims=np.asarray(dimensions)
    lateral=float(np.linalg.norm(np.maximum(np.abs(local[:2])-dims[:2]/2,0)))
    return float(dims[2]/2-local[2]), lateral


class Latch:
    def __init__(self, policy=None):
        self.policy=dict(POLICY if policy is None else policy)
        self.closed=False; self.box_id=None; self.pending_box=None
        self.close_since=None; self.release_since=None; self.open_origin=None; self.release_origin=None
        self.last_t=None; self.history=[]
        self.released_box_id=None;self.withdrawn=False;self.minimum_aperture=1.

    def update(self, t, aperture, tip, boxes):
        cfg=self.policy
        if self.last_t is not None and (t<=self.last_t or t-self.last_t>cfg['max_gap_s']):
            self.close_since=self.release_since=self.open_origin=None; self.history=[]; self.pending_box=None
        self.last_t=t
        found={b['id']:b for b in boxes}
        box=found.get(self.box_id) if self.closed else None
        if self.released_box_id in found and tip is not None:
            rb=found[self.released_box_id]
            if metrics(local_tip(tip,rb),rb['dimensions'])[0]<=cfg['rearm_outside_depth_m']:
                self.withdrawn=True
        local=None; depth=lateral=None; reason='open'
        if not self.closed and tip is not None:
            candidates=[]
            for b in boxes:
                p=local_tip(tip,b);d,l=metrics(p,b['dimensions'])
                if l<=cfg['association_margin_m'] and cfg['close_depth_m']<=d<=b['dimensions'][2]+.03:
                    candidates.append((l,b['id'],b,p))
            if candidates:
                _,_,box,local=min(candidates,key=lambda v:(v[0],v[1]))
        if box is not None and tip is not None:
            if local is None:local=local_tip(tip,box)
            depth,lateral=metrics(local,box['dimensions'])
        if not self.closed:
            eligible=(local is not None and aperture<=cfg['close_at_open_fraction']
                      and (box['id']!=self.released_box_id or self.withdrawn
                           or aperture<=cfg['regrasp_close_fraction']))
            if eligible:
                if self.close_since is None or self.pending_box!=box['id']:
                    self.close_since=t;self.pending_box=box['id']
                if t-self.close_since+1e-9>=cfg['close_confirm_s']:
                    self.closed=True;self.box_id=box['id'];self.history=[];self.minimum_aperture=aperture
                    self.close_since=None;self.pending_box=None;reason='close_depth_and_aperture'
            else:self.close_since=None;self.pending_box=None
        else:
            self.minimum_aperture=min(self.minimum_aperture,aperture)
            reason='hold_closed'
            if local is None:
                self.release_since=self.open_origin=None;self.history=[];reason='hold_missing_geometry'
            elif (aperture<=cfg['release_above_open_fraction']
                  or aperture<self.minimum_aperture+cfg['release_aperture_increase']):
                self.release_since=self.open_origin=None
            else:
                if self.open_origin is None:
                    recent=self.history or [(t,local.copy(),lateral)]
                    self.open_origin=(min(float(p[2]) for _,p,_ in recent),min(v for _,_,v in recent))
                outward=max(float(local[2])-self.open_origin[0],lateral-self.open_origin[1])
                previous=self.history[0] if self.history else (t,local,lateral)
                dt=t-previous[0]
                relative_speed=0. if dt<=0 else max(float(local[2]-previous[1][2]),lateral-previous[2])/dt
                escaping=(depth<=cfg['release_near_rim_depth_m']
                          and outward>=cfg['release_displacement_m']
                          and relative_speed>=cfg['release_relative_speed_m_s'])
                if escaping:
                    if self.release_since is None:
                        self.release_since=t;self.release_origin=(float(local[2]),lateral)
                    continued=max(float(local[2])-self.release_origin[0],lateral-self.release_origin[1])
                    if (t-self.release_since+1e-9>=cfg['release_confirm_s']
                            and continued>=cfg['release_relative_speed_m_s']*cfg['release_confirm_s']):
                        self.released_box_id=self.box_id;self.withdrawn=False
                        self.closed=False;self.box_id=None;self.release_since=self.open_origin=None
                        self.history=[];reason='release_open_and_relative_withdrawal'
                else:self.release_since=None
        if self.closed and local is not None:
            self.history.append((t,local.copy(),lateral))
            self.history=[v for v in self.history if t-v[0]<=cfg['history_s']]
        return dict(closed=self.closed,box_id=self.box_id,depth_m=depth,lateral_distance_m=lateral,
                    raw_open_fraction=float(aperture),open_fraction=0. if self.closed else 1.,reason=reason)


def inputs(result):
    root=Path(result)
    policy=json.loads((root/'POLICY.json').read_text())
    config=json.loads((root/'data/source-config.json').read_text())
    return root/'data/source-trajectory.jsonl',Path(policy['source_candidate'])/'data/frame-scenes.jsonl',root/'data/source-config.json'


def signature(result):
    paths=inputs(result)+(Path(__file__),)
    return tuple((str(p),p.stat().st_mtime_ns,p.stat().st_size) for p in paths)


@lru_cache(maxsize=12)
def _load(result, stamp):
    source,scene_path,config_path=inputs(result)
    with source.open() as stream:
        rows=[r for r in map(json.loads,stream) if r.get('valid')]
    with scene_path.open() as stream:
        scenes={r['external_index']:r for r in map(json.loads,stream)}
    cfg=json.loads(config_path.read_text());T=np.array(cfg.get('coordinate_preparation',{}).get('T_base_recording',np.eye(4)))
    states=[Latch(),Latch()];trace=[]
    for row in rows:
        boxes=[]
        for b in scenes.get(row['external_index'],{}).get('world',[]):
            if not (b['id'].startswith('box_') and b['id'].endswith('_A13')):continue
            boxes.append(b)
        hands=[]
        for j,kind in enumerate(('XRHandLeft','XRHandRight')):
            hp=row.get('human_points',{}).get(kind,{})
            tip=(np.array(hp['5'])+hp['10'])/2 if '5' in hp and '10' in hp else None
            if tip is not None:tip=T[:3,:3].T@(tip-T[:3,3])
            hands.append(states[j].update(float(row['elapsed_s']),float(row['grippers'][j]['open_fraction']),tip,boxes))
        trace.append(dict(source_s=float(row['elapsed_s']),external_index=row['external_index'],hands=hands))
    if len(trace)<2:raise ValueError('Box grasp policy requires two valid source samples')
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    return dict(policy=dict(POLICY),trace=trace,source_s=[r['source_s'] for r in trace],
                input_sha256={str(p):sha(p) for p in (source,scene_path,config_path)},policy_sha256=sha(Path(__file__)))


def load(result):
    return _load(str(Path(result).resolve()),signature(result))


def sample(timeline, source_s):
    times=timeline['source_s'];trace=timeline['trace'];j=max(0,min(len(times)-1,bisect.bisect_right(times,source_s)-1))
    return [0. if trace[j]['hands'][i]['closed'] else 1. for i in range(2)]


def sample_many(timeline, phases):
    times=np.asarray(timeline['source_s']);trace=timeline['trace'];indices=np.clip(np.searchsorted(times,phases,side='right')-1,0,len(times)-1)
    closed=np.asarray([[h['closed'] for h in r['hands']] for r in trace])
    return np.where(closed[indices],0.,1.)


def at_execution(result, execution_s):
    with np.load(Path(result)/'results/execution-trajectory-100hz.npz',allow_pickle=False) as z:
        times=z['execution_s'];phases=z['source_s'];j=int(np.clip(np.searchsorted(times,execution_s,side='right')-1,0,len(times)-1))
        return sample(load(result),float(phases[j]))
