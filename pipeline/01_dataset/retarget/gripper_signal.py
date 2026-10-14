"""Preview-only binary closing score and source-backed revolute linkage angles."""
import numpy as np

# KUAVO MobileManipulatorDummyVisualization getClawLinkageAngles (200062).
LINKAGE = np.array([[0,0,0,0,0],[-.109209,-.101740,-.092681,.121829,-.413562],
    [-.145684,-.128299,-.112237,.170645,-.521204],[-.174150,-.146919,-.125623,.209873,-.599041],
    [-.198813,-.161801,-.136211,.244441,-.663050]])


def linkage_angles(side, opened):
    # The custom S63 URDF has fixed bar-2/finger joints. Never rotate those.
    values = LINKAGE[4 if opened else 0]
    s = side[0]
    return {f'{s}_{branch}_bar_{index}_joint': sign*float(values[column])
        for branch,sign in (('f',1),('b',-1)) for index,column in ((1,0),(3,2),(4,4))}


def binary_signals(rows, low=.35, high=.65):
    if not 0 <= low < high <= 1:
        raise ValueError('Invalid closing-score thresholds')
    calibration = {}
    for side in ('left','right'):
        widths = [r['hands'][side]['candidates']['pinch_width_m'] for r in rows if r['hands'][side]['valid']]
        closed,opened=np.percentile(widths,[20,80])
        if opened-closed < .01:raise ValueError('Insufficient aperture range')
        calibration[side]={'closed_reference_m':float(closed),'open_reference_m':float(opened),
            'open_score_at_most':low,'close_score_at_least':high,'verified':False,'method':'episode P20/P80; preview-only'}
    states={side:None for side in calibration};filtered={side:None for side in calibration};last={side:None for side in calibration}
    output=[]
    for row in rows:
        signals={}
        for side,cfg in calibration.items():
            h=row['hands'][side];t=row['elapsed_s']
            if not h['valid']:
                states[side]=filtered[side]=last[side]=None
                signals[side]={'valid':False,'reason':h['reason']};continue
            dt=None if last[side] is None else t-last[side]
            if dt is None or dt>.25:states[side]=None;filtered[side]=h['candidates']['pinch_width_m']
            else:
                alpha=1-np.exp(-dt/.08);filtered[side]+=alpha*(h['candidates']['pinch_width_m']-filtered[side])
            last[side]=t
            score=float(np.clip((cfg['open_reference_m']-filtered[side])/(cfg['open_reference_m']-cfg['closed_reference_m']),0,1))
            if score<=low:states[side]='open'
            elif score>=high:states[side]='closed'
            state=states[side]
            signals[side]={'valid':state is not None,'state':state,'closing_score':score,
                'raw_pinch_m':h['candidates']['pinch_width_m'],'filtered_pinch_m':float(filtered[side]),
                'open_target':None if state is None else int(state=='open'),
                'joint_targets':{} if state is None else linkage_angles(side,state=='open'),
                'hardware_calibration_verified':False}
        output.append({'frame':row['index'],'elapsed_s':row['elapsed_s'],'hands':signals})
    return output,calibration
