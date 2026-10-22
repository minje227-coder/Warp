"""Adapt already fused bilateral WiLoR hands to an isolated KUAVO solver snapshot."""
from pathlib import Path
import argparse, json, sys
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'source'))
from retarget import convert as converter
from retarget.input import read_episode, sha256, UNIX_TICKS

IDS = [1,2,3,4,5,7,8,9,10,12,13,14,15,17,18,19,20,22,23,24,25]
def readl(p): return [json.loads(l) for l in p.open() if l.strip()]
def save(p, v): p.write_text(json.dumps(v, indent=2)+'\n')

def run(take, maximum=None):
    allocation='geometry'
    review = ROOT.parent
    fused_path = review/'wilor-both-hands-20261016-newtake'/take/'fused_both.jsonl'
    refs_path = review/'external-skeleton-20261016-newtake'/take/'meta_frame_reference.jsonl'
    raw_root=Path('/home/lab6/Warp_data')
    candidates=sorted(p for p in raw_root.glob(f'????-??-??/{take}') if p.is_dir())
    if len(candidates)!=1:
        raise ValueError(f'Expected one processed episode for {take}, found {candidates}')
    raw=candidates[0]
    manifest, _, discontinuities = read_episode(raw)  # Check every raw manifest hash.
    refs = {r['external_index']:r['reference'] for r in readl(refs_path)}
    streams = {k:[] for k in ['body','XRHandLeft','XRHandRight']}
    provenance=[]
    for row in readl(fused_path):
        index = row['external_index']; rr = refs[index]; br = rr['body']
        # Original reference delta is sample tick minus external host-arrival tick.
        ticks = br['sample']['timestamp_ticks'] - round(br['delta_ms']*10000)
        detail={'external_index':index,'timestamp_ticks':ticks,'source_external_pts_us':row['source_external_pts_us'],
                'reference':{k:{'delta_ms':v['delta_ms'],'sample_index':v['sample']['original_sample_index'],
                               'timestamp_ticks':v['sample']['timestamp_ticks']} for k,v in rr.items() if v},
                'hand_status':{s:h['status'] for s,h in row['hands'].items()}}
        provenance.append(detail)
        for kind in streams:
            if kind=='body':
                points={int(b['id']):np.array([b['position'][k] for k in 'xyz']) for b in br['sample']['bones']}
                valid=br['usable'] and abs(br['delta_ms'])<=20
                original=br['sample']
            else:
                side='left' if kind=='XRHandLeft' else 'right'; h=row['hands'][side]
                points={} if h['joints'] is None else dict(zip(IDS,map(np.asarray,h['joints'])))
                valid=h['status']=='hand_wrist_candidate' and bool(points)
                original=rr[kind]['sample']
            streams[kind].append(dict(ticks=ticks,utc_ns=(ticks-UNIX_TICKS)*100,line=index+1,points=points,
                valid=bool(valid),frame='derived_WiLoR_on_Meta_wrist' if kind!='body' else original['coordinate_frame'],
                time_source='external_host_arrival_UTC; exposure_offset_unverified',
                origin=original['tracking_origin_pose'],provider=original['provider_transform']))
    assert all(all(a['ticks']<b['ticks'] for a,b in zip(s,s[1:])) for s in streams.values())
    from input_gap_fill import fill_missing
    gap_records,gap_summary=fill_missing(streams,discontinuities)
    save(ROOT/'input-gap-summary.json',gap_summary)
    (ROOT/'interpolated-inputs.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in gap_records))
    gap_by_index={}
    for item in gap_records:gap_by_index.setdefault(item['external_index'],[]).append(item)
    for detail in provenance:
        detail['input_gap_interpolation']=gap_by_index.get(detail['external_index'],[])
    # Every generated observation retains its original source timestamp.

    converter.read_episode=lambda p:(manifest,streams,discontinuities)
    cfg=json.loads((ROOT/'standard_config.json').read_text())
    end_seconds=cfg.get('source_end_seconds')
    seed_trajectory=ROOT/'reference-seeds'/f'{take}.jsonl'
    if seed_trajectory.is_file():
        cfg['initial_seed_trajectory']=str(seed_trajectory)
    config=ROOT/'results'/f'{take}.input-config.json'
    config.parent.mkdir(parents=True,exist_ok=True)
    output=ROOT/'results'/take
    if output.exists() or output.with_name(output.name+'.partial').exists():
        raise FileExistsError('Result already exists; choose a new take or use render/verify')
    save(config,cfg)
    converter.convert(raw, output,
        '/home/lab6/humanoid/git/kuavo_ros_application/data_collection/custom_urdf/kuavo_s63_twofinger.urdf',
        config,maximum_frames=maximum,allocation=allocation,end_seconds=end_seconds)
    trajectory=readl(output/'trajectory.jsonl')
    provenance=provenance[:len(trajectory)]
    assert len(trajectory)==len(provenance)
    for r,p in zip(trajectory,provenance):
        assert r['timestamp_ticks']==p['timestamp_ticks']
        r.update(external_index=p['external_index'],source_external_pts_us=p['source_external_pts_us'])
        r['input_gap_interpolation']=p['input_gap_interpolation']
    (output/'trajectory.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in trajectory))
    (output/'input-provenance.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in provenance[:maximum]))
    m=json.loads((output/'manifest.json').read_text())
    m.update(input_adapter='run_retarget.py: original timestamps; bounded missing XYZ inputs linearly interpolated; measured inputs unchanged',
        input_source_hashes={str(p):sha256(p) for p in [fused_path,refs_path,Path(__file__)]},
        automatic_hand_selection=False,body_wrist_fallback_allowed_for_IK=False,
        source_line_semantics='derived external_index+1; raw sample indices in input-provenance.jsonl',
        grasp_axis_relaxation=True, body_down_range_deg=[0,60], body_down_reference_deg=cfg['ik']['body_down_reference_deg'],tcp_target_offset_world_m=cfg['tcp_target_offset_world_m'],initial_pose_recomputed=True)
    m['input_gap_interpolation']=gap_summary
    m['input_source_hashes'][str(ROOT/'interpolated-inputs.jsonl')]=sha256(ROOT/'interpolated-inputs.jsonl')
    m['output_checksums']['trajectory.jsonl']=sha256(output/'trajectory.jsonl')
    m['output_checksums']['input-provenance.jsonl']=sha256(output/'input-provenance.jsonl')
    save(output/'manifest.json',m)
    print(json.dumps(json.loads((output/'quality.json').read_text())),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('take');p.add_argument('--maximum',type=int)
    a=p.parse_args();run(a.take,a.maximum)
