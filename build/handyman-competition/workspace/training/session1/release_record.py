"""Record observed open-hand acknowledgement using the sensor/session clock."""
def record_release(snapshot,*,session_id,run_id,operation_id,hand_is_open):
    stamp=snapshot['streams']['depth']['stamp_ns']
    if type(stamp) is not int or stamp<=0 or not hand_is_open(snapshot['joints']['hand_motor_joint']):
        raise ValueError('fresh_open_hand_acknowledgement_required')
    if not all(isinstance(v,str) and v for v in (session_id,run_id,operation_id)):
        raise ValueError('current_operation_identity_required')
    return dict(session_id=session_id,run_id=run_id,operation_id=operation_id,
                release_stamp_ns=stamp,clock='depth_sensor_clock',
                release_sent=True,placement_verified=False,
                timestamp_basis='first_fresh_depth_after_open_feedback')
