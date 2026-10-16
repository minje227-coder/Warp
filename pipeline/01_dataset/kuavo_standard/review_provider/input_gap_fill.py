"""Offline input-only gap filling; raw measurements and source clocks stay intact."""
import numpy as np
from collections import Counter

def fill_missing(streams,discontinuities=()):
    records=[];skipped=[];counts={}
    for kind,samples in streams.items():
        # Endpoints come only from original valid observations, never previous fills.
        valid=np.array([bool(s['valid']) for s in samples]);indices=np.flatnonzero(valid)
        for i in np.flatnonzero(~valid):
            k=int(np.searchsorted(indices,i))
            if k==0 or k==len(indices):
                skipped.append(dict(stream=kind,external_index=samples[i]['line']-1,reason='no_two_sided_valid_bracket'));continue
            lo,hi=int(indices[k-1]),int(indices[k]);a,z=samples[lo],samples[hi];r=samples[i]
            if any(a.get(key)!=z.get(key) for key in ['frame','origin','provider']) or any(a['ticks']<d<=z['ticks'] for d in discontinuities):
                skipped.append(dict(stream=kind,external_index=r['line']-1,reason='coordinate_or_clock_discontinuity'));continue
            if a['points'].keys()!=z['points'].keys():
                skipped.append(dict(stream=kind,external_index=r['line']-1,reason='joint_set_mismatch'));continue
            w=(r['ticks']-a['ticks'])/(z['ticks']-a['ticks']);assert 0<w<1
            points={j:(1-w)*a['points'][j]+w*z['points'][j] for j in a['points']}
            assert all(np.isfinite(x).all() for x in points.values())
            r['points']=points;r['valid']=True;r['origin']=a['origin'];r['provider']=a['provider'];r['frame']=a['frame']
            records.append(dict(stream=kind,external_index=r['line']-1,timestamp_ticks=r['ticks'],anchor_external_indices=[a['line']-1,z['line']-1],anchor_ticks=[a['ticks'],z['ticks']],weight=w,bracket_s=(z['ticks']-a['ticks'])*1e-7,method='linear XYZ',points={str(j):x.tolist() for j,x in points.items()}))
        counts[kind]=dict(original_invalid=int(sum(~valid)),filled=int(sum(not v and s['valid'] for v,s in zip(valid,samples))),remaining_invalid=int(sum(not s['valid'] for s in samples)))
    return records,dict(streams=counts,filled_source_frames=len({r['external_index'] for r in records}),maximum_bracket_s=max((r['bracket_s'] for r in records),default=0),records_scope='entire preprocessed input including recorded lookahead; IK output clipped to 0-20s',skipped=skipped,extrapolation=False,observed_samples_unchanged=True,source_timestamps_unchanged=True)
