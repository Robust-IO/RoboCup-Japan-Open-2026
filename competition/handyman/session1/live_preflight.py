"""Explicit live read-only preflight. Never calls task handlers or protocol IO."""
import json
from pathlib import Path


def run(bundle):
    runtime=bundle['runtime'];motion=bundle['motion_backend']
    old_publisher=runtime.publisher;old_command=motion.command
    def forbidden(*a,**kw):raise RuntimeError('motion_disabled_during_preflight')
    runtime.publisher=forbidden;motion.command=forbidden
    output=Path(bundle['output'])
    report=dict(mode='live_read_only_preflight',status='checking',robot_commands_sent=False,
                competition_protocol_messages_sent=False,full_transfer_verified=False)
    try:
        runtime.sample(profile='state');state=runtime.state()
        room=motion.room_at(state['poses']['map'])
        if room is None:raise ValueError('initial_room_not_localized')
        from task_outcome_policy import hand_is_open
        report['initial_hand_open']=hand_is_open(state['joints'][5])
        report['task_start_ready']=report['initial_hand_open']
        report['preparation_needed']=[] if report['initial_hand_open'] else ['open_empty_gripper_before_task']
        report.update(room=room,map_pose=state['poses']['map'],joints=state['joints'])
        item=bundle['grasp_backend']._sample()
        runtime.fresh_measurement(item['snapshot'])
        capture=runtime.save(item,'preflight-rgbd')
        detections=bundle['worker'].infer(capture,pump=lambda:runtime.pump(0))
        if not isinstance(detections.get('detections'),list):raise ValueError('model_detection_response_invalid')
        report.update(status='passed',capture=str(capture),depth_stamp_ns=item['snapshot']['streams']['depth']['stamp_ns'],
                      rgb_shape=list(item['rgb'].shape),depth_shape=list(item['depth'].shape),
                      detection_count=len(detections['detections']),model_inference_responded=True,
                      target_visible=any(d.get('name')=='dog_doll' for d in detections['detections']))
        (output/'preflight-detections.json').write_text(json.dumps(detections))
    except Exception as exc:
        report.update(status='failed',reason=type(exc).__name__+':'+str(exc))
        raise
    finally:
        (output/'preflight-result.json').write_text(json.dumps(report,indent=2))
        runtime.publisher=old_publisher;motion.command=old_command
    return report
