"""Selected first-state contract. Head is never a command target."""
import math

NAMES = ['knee_joint', 'leg_joint', 'waist_pitch_joint', 'waist_yaw_joint'] + [
    'zarm_%s%d_joint' % (s, i) for s in 'lr' for i in range(1, 8)]
HEAD = ['zhead_1_joint', 'zhead_2_joint']


def vector(values, size, label):
    if (not isinstance(values, list) or len(values) != size
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values)):
        raise ValueError(label + ' 값이 없거나 잘못되었습니다.')
    return list(map(float, values))


def target(detail):
    initial = detail['initial']
    names = initial.get('joint_names', [])
    q = vector(initial.get('q'), 18, '시작 관절')
    if len(names) != 18 or set(names) != set(NAMES):
        raise ValueError('시작 자세의 몸통 4 + 양팔 14 관절 이름이 다릅니다.')
    q = [q[names.index(n)] for n in NAMES]
    grips = initial.get('grippers')
    if not isinstance(grips, list) or len(grips) != 2:
        raise ValueError('선택 시작 시점의 양쪽 그리퍼 상태가 없습니다.')
    fractions = vector([g.get('open_fraction') if isinstance(g, dict) else None for g in grips], 2, '그리퍼')
    if any(not 0 <= v <= 1 for v in fractions):
        raise ValueError('그리퍼 열림 비율이 범위를 벗어났습니다.')
    if initial.get('gripper_policy', {}).get('kind') != 'box_relative_grasp_v1':
        fractions = [0. if v <= .40 else v for v in fractions]
    limits = dict(velocity=[3.14]*4+[5.14]*14,
                  acceleration=[5.14]*4+[25.]*14)
    return dict(q=q, joint_names=NAMES, limits=limits, base_pose=vector(initial.get('base_pose'), 3, '시작 베이스'),
                claw=[100*(1-v) for v in fractions], source_s=float(initial['source_s']), arm_duration_s=2., head_excluded=True)



def topic_fresh(topic, snapshot, now, limit):
    # Agent timestamps its snapshot before decoding topics. A callback during
    # decoding may therefore be newer than snapshot.t (observed -0.08 s).
    # Bound that race; never let a negative age extend stale snapshot validity.
    elapsed=now-snapshot['t']
    age=topic['age']
    return (isinstance(age,(int,float)) and math.isfinite(age)
            and 0<=elapsed<1.5 and -.25<=age
            and elapsed+max(0.,age)<limit)


def feedback_matches(target_state, snapshot, now):
    """A completed alignment expires if real non-head joints or claws move."""
    try:
        topics = snapshot['topics']
        sensor = topics['/sensors_data_raw']
        claw = topics['/leju_claw_state']
        # RobotLink topic stamps are wall-clock arrival times, not ROS stamps.
        if (not 0 <= now-snapshot['t'] < 1.5
                or any(not topic_fresh(x,snapshot,now,1.) for x in (sensor, claw))):
            return False
        q = sensor['data']['joint_data']['joint_q'][:18]
        grip = claw['data']['data']['position']
        q = vector(q, 18, '실측 관절')
        grip = vector(grip[:2], 2, '실측 그리퍼')
        return (max(abs(a-b) for a, b in zip(q, target_state['q'])) <= math.radians(3)
                and max(abs(a-b) for a, b in zip(grip, target_state['claw'])) <= 8)
    except (KeyError, TypeError, ValueError):
        return False



def base_feedback_matches(goal,frame,snapshot,now):
    try:
        goal=vector(goal,3,'고정 베이스 목표')
        odom=snapshot['topics']['/odom'];data=odom['data']
        if not topic_fresh(odom,snapshot,now,.5):return False
        if data['header']['frame_id']!=frame:return False
        p=data['pose']['pose']['position'];q=data['pose']['pose']['orientation']
        yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
        dyaw=math.atan2(math.sin(goal[2]-yaw),math.cos(goal[2]-yaw))
        return math.hypot(goal[0]-p['x'],goal[1]-p['y'])<=.04 and abs(dyaw)<=math.radians(4)
    except (ValueError,KeyError,TypeError):return False
