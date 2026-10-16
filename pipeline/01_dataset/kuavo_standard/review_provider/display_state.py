"""Display-only last accepted pose on failures. Never changes trajectory rows."""


def make_display_states(rows):
    last = None
    states = {}
    for row in rows:
        if row['valid']:
            last = row
        failed = not row['valid']
        states[row['external_index']] = dict(
            q=None if last is None else last['q'],
            base=None if last is None else last['base_pose'],
            grippers=[] if last is None else last.get('grippers', []),
            failed=failed,
            held=failed and last is not None,
            display_only=failed,
            pose_timestamp_ticks=None if last is None else last['timestamp_ticks'],
            pose_external_index=None if last is None else last['external_index'],
            mode='IK accepted' if not failed else 'FAILED | last accepted pose (display only)',
        )
    return states


def display_metadata(state):
    return dict(robot_pose_held=state['held'],
                pose_is_display_only=state['display_only'],
                display_pose_timestamp_ticks=state['pose_timestamp_ticks'],
                display_pose_external_index=state['pose_external_index'],
                robot_color='red' if state['failed'] else 'original')
