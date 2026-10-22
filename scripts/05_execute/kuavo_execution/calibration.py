"""Surveyed rack heights for offline input preparation; no robot observation."""
import json
import time
from pathlib import Path
import numpy as np


def rack_reference(prepared):
    provider=Path(prepared['display_source'])
    anchor=json.loads((provider/'data/rack-preview-anchor.json').read_text())
    policy=json.loads((provider/'tag_policy.json').read_text())
    overlay=json.loads((provider/'object_overlay_config.json').read_text())
    settings={k:overlay[k] for k in ('pose_selection','multi_tag_pose')}
    settings['tag_size_m']=overlay['rack']['tag_size_m']
    offsets=anchor.get('tag_offsets') or policy['rack']['tag_offsets_in_geometry_tag_axes_m']
    T=np.eye(4);T[:3,:3]=anchor['rotation'];T[:3,3]=anchor['position']
    tags={}
    for tag,offset in offsets.items():
        M=T.copy();M[:3,3]+=M[:3,:3]@offset;tags[tag]=M.tolist()
    return dict(rack_tags=tags,rack_fit_settings=settings)


FIXED_TAG_HEIGHTS_M = {'0': 1.615, '4': 1.615, '1': 1.615,
                       '2': 1.000, '5': 1.000, '3': 1.000}


def capture(episodes, key, timeout=8.):
    """Use surveyed heights only; preserve recording XY, yaw and geometry."""
    entry = episodes.entry(key)
    source = Path(entry.get('input_prepared') or entry['prepared'])
    prepared = json.loads(source.read_text())
    if prepared.get('coordinate_preparation'):
        raise ValueError('원본 녹화 입력이 없습니다. 높이 변환을 중복 적용할 수 없습니다.')
    tags = rack_reference(prepared)['rack_tags']
    if not set(FIXED_TAG_HEIGHTS_M).issubset(tags):
        raise ValueError('고정 높이 기준 태그 0,4,1,2,5,3이 필요합니다.')
    dz = float(np.mean([height - tags[tag][2][3]
                       for tag, height in FIXED_TAG_HEIGHTS_M.items()]))
    T = np.eye(4)
    T[2, 3] = dz
    residual = {tag: float(tags[tag][2][3] + dz - height)
                for tag, height in FIXED_TAG_HEIGHTS_M.items()}
    if not np.isfinite(T).all() or max(map(abs, residual.values())) > .02:
        raise ValueError('녹화 랙 높이와 실측 높이의 잔차가 2cm를 넘었습니다.')
    return dict(episode_id=key, T_base_recording=T.tolist(),
                captured_at=time.time(), frame='fixed_rack_height',
                fixed_tag_heights_m=FIXED_TAG_HEIGHTS_M.copy(),
                height_residual_m=residual, height_offset_m=dz,
                method='surveyed_heights_translation_only',
                xy_yaw='original_recording', robot_observation_used=False,
                input_descriptor=str(source), robot_commands_sent=0)
