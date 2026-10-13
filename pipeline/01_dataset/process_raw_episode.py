"""Offline raw-v3 QC/AprilTag processing. Never modifies the collection archive.

Clock offset is UTC nanoseconds minus Camera2 SENSOR_TIMESTAMP nanoseconds.
It must be independently validated; callback wall-clock is not exposure time.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import json
import math
import re
from pathlib import Path

UNIX_TICKS = 621355968000000000


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for part in iter(lambda: f.read(1024 * 1024), b''): h.update(part)
    return h.hexdigest()


def read_jsonl(path):
    with Path(path).open() as f:
        return [json.loads(line) for line in f if line.strip()]


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def reconcile(encoded, sensor, tolerance_ns=2000):
    """Match unique PTS to unique exposure timestamp; no FPS reconstruction."""
    times = sorted((int(f['sensor_timestamp_ns']), i) for i, f in enumerate(sensor))
    values = [t[0] for t in times]
    used, results = set(), []
    counts = Counter(values)
    pts_counts = Counter(int(f['pts_us']) for f in encoded)
    for frame in encoded:
        t = int(frame['pts_us']) * 1000
        j = bisect_left(values, t)
        candidates = [k for k in (j-1,j) if 0 <= k < len(times)]
        candidates.sort(key=lambda k: abs(values[k]-t))
        match = None
        if candidates:
            k = candidates[0]
            ambiguous = len(candidates)>1 and abs(values[candidates[1]]-t)==abs(values[k]-t)
            if abs(values[k]-t)<=tolerance_ns and not ambiguous and counts[values[k]]==1 and pts_counts[int(frame['pts_us'])]==1 and times[k][1] not in used:
                match = times[k][1]; used.add(match)
        results.append({'encoded_index':frame['index'], 'pts_us':int(frame['pts_us']),
                        'sensor_index':match, 'sensor_timestamp_ns':None if match is None else int(sensor[match]['sensor_timestamp_ns'])})
    return results


def verify_container_pts(decoded, encoded, tolerance_us=1000):
    """Verify one constant muxer shift across every frame, preserving order."""
    original=[int(f['pts_us']) for f in encoded]
    result={'verified':False,'container_to_encoder_shift_us':None,'maximum_residual_us':None,'tolerance_us':tolerance_us,'tolerance_reason':'MP4 container time-base quantization; sensor exposure reconciliation separately uses 2us'}
    if not decoded or len(decoded)!=len(original):
        return dict(result,reason='frame_count_mismatch')
    if any(t is None for t in decoded): return dict(result,reason='missing_pts')
    if any(b<=a for a,b in zip(decoded,decoded[1:])) or any(b<=a for a,b in zip(original,original[1:])):
        return dict(result,reason='nonmonotonic_or_duplicate_pts')
    shift=original[0]-decoded[0]
    residual=max(abs((a-b)-shift) for a,b in zip(original,decoded))
    result.update(container_to_encoder_shift_us=shift,maximum_residual_us=residual)
    if residual>tolerance_us: return dict(result,reason='nonconstant_pts_shift')
    return dict(result,verified=True,reason='verified_full_sequence')


def slerp(a, b, weight):
    def normalized(q):
        n = math.sqrt(sum(x*x for x in q))
        if n < 1e-12: raise ValueError('Zero quaternion')
        return [x/n for x in q]
    a,b=normalized(a),normalized(b)
    dot=sum(x*y for x,y in zip(a,b))
    if dot<0: b=[-x for x in b]; dot=-dot
    dot=min(1.0,max(-1.0,dot))
    if dot>0.9995: return normalized([x+(y-x)*weight for x,y in zip(a,b)])
    theta=math.acos(dot)
    return [(math.sin((1-weight)*theta)*x+math.sin(weight*theta)*y)/math.sin(theta) for x,y in zip(a,b)]


def vec(v, keys): return [float(v[k]) for k in keys]


def align_body(matches, tracking, events, offset_ns, max_bracket_ms=50):
    if offset_ns is None: return []
    if not 0 < max_bracket_ms <= 50: raise ValueError('Bracket limit must be in (0,50] ms')
    body=sorted((s for s in tracking if s.get('kind')=='body'),key=lambda s:s['timestamp_ticks'])
    ns=[(int(s['timestamp_ticks'])-UNIX_TICKS)*100 for s in body]
    discontinuities=[(int(e['unix_ticks'])-UNIX_TICKS)*100 for e in events
                     if 'unix_ticks' in e and any(x in e.get('event','') for x in ('recenter','pause','regressed','tracking_lost'))]
    output=[]
    for match in matches:
        sensor=match['sensor_timestamp_ns']
        row={'encoded_index':match['encoded_index'],'valid':False,'reason':'unmatched_camera'}
        if sensor is None: output.append(row); continue
        target=sensor+int(offset_ns); row['utc_ns']=target
        i=bisect_left(ns,target)
        if i<len(ns) and ns[i]==target: lo=hi=i
        elif i==0 or i==len(ns): row['reason']='outside_tracking';output.append(row);continue
        else: lo,hi=i-1,i
        if ns[hi]-ns[lo]>max_bracket_ms*1e6: row['reason']='tracking_gap';output.append(row);continue
        a,b=body[lo],body[hi]
        if not a.get('valid') or not b.get('valid'): row['reason']='invalid_tracking';output.append(row);continue
        if a.get('coordinate_frame')!=b.get('coordinate_frame') or a.get('tracking_origin_pose')!=b.get('tracking_origin_pose') or any(ns[lo]<d<=ns[hi] for d in discontinuities):
            row['reason']='tracking_discontinuity';output.append(row);continue
        aa={v['id']:v for v in a.get('bones',[])};bb={v['id']:v for v in b.get('bones',[])}
        weight=0 if lo==hi else (target-ns[lo])/(ns[hi]-ns[lo])
        bones=[]
        try:
            for ident in sorted(aa.keys() & bb.keys()):
                x,y=aa[ident],bb[ident]
                if x['name']!=y['name']: continue
                px,py=vec(x['position'],'xyz'),vec(y['position'],'xyz')
                position=[p+(q-p)*weight for p,q in zip(px,py)]
                rotation=slerp(vec(x['rotation'],'xyzw'),vec(y['rotation'],'xyzw'),weight)
                if not all(math.isfinite(v) for v in position+rotation): raise ValueError('Nonfinite bone')
                bones.append({'id':ident,'name':x['name'],'position':dict(zip('xyz',position)),'rotation':dict(zip('xyzw',rotation))})
        except (ValueError,KeyError,TypeError): row['reason']='invalid_bones';output.append(row);continue
        row.update(valid=bool(bones),reason='aligned' if bones else 'missing_bones',bones=bones,
                   coordinate_frame=a.get('coordinate_frame'),source_timestamp_ticks=[a['timestamp_ticks'],b['timestamp_ticks']],interpolation_weight=weight)
        output.append(row)
    return output


def coverage(tracking):
    result={}
    for kind in sorted({s.get('kind','unknown') for s in tracking}):
        samples=[s for s in tracking if s.get('kind','unknown')==kind]
        source_valid=[s for s in samples if s.get('valid')]
        is_hand=kind in ('HandLeft','HandRight','XRHandLeft','XRHandRight')
        def zero_geometry(sample):
            bones=sample.get('bones',[])
            return bool(is_hand and len(bones)>1 and all(all(abs(float(b.get('position',{}).get(axis,0)))<1e-9 for axis in 'xyz') for b in bones))
        zeros=[s for s in samples if zero_geometry(s)]
        valid=[s for s in source_valid if not zero_geometry(s)]
        names=Counter(b.get('name','unknown') for s in valid for b in s.get('bones',[]))
        result[kind]={'samples':len(samples),'source_valid_samples':len(source_valid),'all_zero_geometry_samples':len(zeros),'valid_samples':len(valid),'valid_fraction':len(valid)/len(samples),'joint_valid_sample_counts':dict(names)}
    result['missing_types']=[k for k,variants in {'body':['body'],'left_hand':['HandLeft','XRHandLeft'],'right_hand':['HandRight','XRHandRight']}.items() if not any(v in result for v in variants)]
    return result


def process(source, destination, offset_ns=None, max_bracket_ms=50, tag_size_m=None, calibration_override=None, detect_tags=True):
    import av
    if not 0 < max_bracket_ms <= 50: raise ValueError('Bracket limit must be in (0,50] ms')
    source,destination=Path(source).resolve(),Path(destination).resolve()
    if destination==source or source in destination.parents: raise ValueError('Derived output must be outside raw episode')
    manifest=json.loads((source/'manifest.json').read_text())
    if manifest.get('schema_version')!=3: raise ValueError('Expected raw schema v3')
    checks={}
    for name,spec in manifest['files'].items():
        if Path(name).name!=name: raise ValueError('Invalid manifest filename')
        path=source/name
        checks[name]=sha256(path)
        if checks[name]!=spec['sha256'] or path.stat().st_size!=spec['bytes']: raise ValueError('Raw checksum mismatch: '+name)
    sensor=read_jsonl(source/'camera_frames.jsonl');encoded=read_jsonl(source/'encoded_frames.jsonl')
    tracking=read_jsonl(source/'tracking.jsonl');events=read_jsonl(source/'events.jsonl')
    matches=reconcile(encoded,sensor)
    calibration=json.loads(Path(calibration_override).read_text()) if calibration_override else json.loads((source/'calibration.json').read_text())
    intrinsics=calibration.get('output_intrinsics')
    # Camera2 calibration applies directly only to an uncropped full active array.
    dims=[0,0,manifest['width'],manifest['height']]
    rect=lambda value: [int(v) for v in re.findall(r'-?\d+', str(value))]
    native=calibration.get('intrinsics')
    direct=bool(native and len(native)>=4 and rect(calibration.get('active_array'))==dims and sensor and all(rect(f.get('crop_region'))==dims for f in sensor))
    if intrinsics is None and direct and (len(native)<5 or abs(float(native[4]))<1e-8) and not any(abs(float(v))>1e-8 for v in (calibration.get('distortion') or [])):
        calibration['output_intrinsics_verified']=True
        intrinsics=native[:4]
        calibration['output_intrinsics']=intrinsics
    verified=bool(calibration.get('output_intrinsics_verified') and intrinsics and calibration.get('width')==manifest['width'] and calibration.get('height')==manifest['height'])
    detector=None
    if detect_tags:
        from pupil_apriltags import Detector
        detector=Detector(families='tag36h11',quad_decimate=1.0,nthreads=2)
    if tag_size_m is not None and tag_size_m<=0: raise ValueError('Tag size must be positive')
    destination.mkdir(parents=True,exist_ok=False)
    write_json(destination/'processing_manifest.json',{'state':'processing','source':str(source),'source_manifest_sha256':sha256(source/'manifest.json'),'source_checksums':checks,
        'processor_sha256':sha256(__file__),'settings':{'camera_to_utc_offset_ns':offset_ns,'max_bracket_ms':max_bracket_ms,'tag_size_m':tag_size_m,'detect_tags':detect_tags},
        'calibration_sha256':sha256(calibration_override or source/'calibration.json'),'pyav_version':av.__version__,'camera_pose_frame':'camera_optical','world_pose_verified':False})
    # MediaMuxer may normalize container timestamps. Verify a constant shift
    # over the complete sequence before assigning any exposure to decoded pixels.
    with av.open(str(source/'camera.mp4')) as video:
        container_pts=[None if f.pts is None else round(f.pts*f.time_base*1000000) for f in video.decode(video=0)]
    container_mapping=verify_container_pts(container_pts,encoded)
    decoded_pts=[];unmatched_decode=0;detections=0
    with (destination/'apriltag.jsonl').open('w') as tags, (destination/'decoded_frames.jsonl').open('w') as frames:
        with av.open(str(source/'camera.mp4')) as container:
            for index,frame in enumerate(container.decode(video=0)):
                if (frame.width,frame.height)!=(manifest['width'],manifest['height']): raise ValueError('Decoded dimensions differ from manifest')
                pts_us=None if frame.pts is None else round(frame.pts*frame.time_base*1000000)
                decoded_pts.append(pts_us)
                encoder_pts_us=int(encoded[index]['pts_us']) if container_mapping['verified'] else None
                mapping=matches[index] if container_mapping['verified'] else None
                unmatched_decode+=mapping is None or mapping['sensor_index'] is None
                frames.write(json.dumps({'decoded_index':index,'container_pts_us':pts_us,'encoder_pts_us':encoder_pts_us,'camera_match':mapping})+'\n')
                if detector:
                    pose=verified and tag_size_m is not None
                    found=detector.detect(frame.to_ndarray(format='gray'),estimate_tag_pose=pose,camera_params=intrinsics if pose else None,tag_size=tag_size_m if pose else None)
                    for d in found:
                        row={'decoded_index':index,'container_pts_us':pts_us,'encoder_pts_us':encoder_pts_us,'camera_match':mapping,'tag_id':int(d.tag_id),'hamming':int(d.hamming),'decision_margin':float(d.decision_margin),'corners':d.corners.tolist(),'center':d.center.tolist(),'pose_frame':'camera_optical','pose_verified':pose}
                        if pose: row.update(rotation=d.pose_R.tolist(),translation_m=d.pose_t.reshape(-1).tolist(),pose_error=float(d.pose_err))
                        tags.write(json.dumps(row,allow_nan=False)+'\n');detections+=1
    aligned=align_body(matches,tracking,events,offset_ns,max_bracket_ms) if container_mapping['verified'] else []
    with (destination/'aligned_body.jsonl').open('w') as f:
        for row in aligned: f.write(json.dumps(row,allow_nan=False)+'\n')
    gaps=[(b-a)/1e6 for a,b in zip([s['sensor_timestamp_ns'] for s in sensor],[s['sensor_timestamp_ns'] for s in sensor][1:])]
    qc={'state':'needs_frame_validation' if not container_mapping['verified'] else 'needs_clock_validation' if offset_ns is None else 'processed_with_explicit_clock_offset','container_pts_validation':container_mapping,'dataset_ready':False,'decoded_frames':len(decoded_pts),'encoded_frames':len(encoded),'sensor_frames':len(sensor),
        'unmatched_encoded_frames':sum(r['sensor_index'] is None for r in matches),'unmatched_decoded_frames':unmatched_decode,'duplicate_decoded_pts':len(decoded_pts)-len(set(decoded_pts)),'decoded_pts_regressions':sum(b<=a for a,b in zip(decoded_pts,decoded_pts[1:]) if a is not None and b is not None),'unencoded_sensor_frames':len(sensor)-sum(r['sensor_index'] is not None for r in matches),
        'sensor_clock_regressions':sum(g<=0 for g in gaps),'maximum_sensor_gap_ms':max(gaps,default=None),'observed_sensor_fps':None if len(sensor)<2 or sensor[-1]['sensor_timestamp_ns']<=sensor[0]['sensor_timestamp_ns'] else (len(sensor)-1)*1e9/(sensor[-1]['sensor_timestamp_ns']-sensor[0]['sensor_timestamp_ns']),
        'apriltag_detections':detections,'camera_pose_status':'output_intrinsics_verified' if verified else 'needs_camera_calibration','tracking_coverage':coverage(tracking),
        'alignment_valid_frames':sum(r['valid'] for r in aligned),'alignment_reasons':dict(Counter(r['reason'] for r in aligned)),
        'notes':['Container normalization tolerates 1ms residual; longer episodes may need quantization-aware validation, without fabricated corrections.','Hand coverage excludes multi-joint all-zero geometry even if SDK reports valid; raw validity remains preserved.','No repeated-frame padding or fixed-FPS reconstruction.','Static scenes cannot establish image duplication; PTS and exposure provenance are checked.','Explicit offset must be validated independently, including drift; camera-to-world extrinsics remain unverified.']}
    write_json(destination/'quality.json',qc)
    provenance=json.loads((destination/'processing_manifest.json').read_text());provenance['state']='complete';provenance['outputs']={p.name:sha256(p) for p in destination.iterdir() if p.name!='processing_manifest.json'}
    write_json(destination/'processing_manifest.json',provenance)
    return qc


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source');p.add_argument('destination')
    p.add_argument('--camera-to-utc-offset-ns',type=int);p.add_argument('--max-bracket-ms',type=float,default=50);p.add_argument('--tag-size-m',type=float)
    p.add_argument('--calibration',help='Verified native-output intrinsics JSON: output_intrinsics=[fx,fy,cx,cy], output_intrinsics_verified=true, width,height')
    p.add_argument('--skip-apriltag',action='store_true')
    a=p.parse_args();print(json.dumps(process(a.source,a.destination,a.camera_to_utc_offset_ns,a.max_bracket_ms,a.tag_size_m,a.calibration,not a.skip_apriltag),indent=2))

if __name__=='__main__': main()
