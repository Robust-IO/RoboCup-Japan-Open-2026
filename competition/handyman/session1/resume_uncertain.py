"""Same-round lift recheck from actual preclose and rejected-registration artifacts.

No replayed lift/closure. Only head viewpoints change, then actual fresh tracking
must prove the original lift before the protocol can report Object_grasped.
"""
import json
from pathlib import Path
import numpy as np
from execution_plan import grasp_result

def restore(session,assembled,checkpoint,current):
    path=Path(checkpoint);saved=json.loads(path.read_text());root=path.parent
    if saved.get('state')!='stopped' or saved['task']!=session.snapshot()['task']:raise ValueError('matching_stopped_task_required')
    report_path=root/'grasp-result.json'
    unresolved=report_path.exists() and json.loads(report_path.read_text()).get('status')=='holding_unresolved_preserved'
    journal=(root/'journal.jsonl').read_text()
    if not unresolved and not (saved.get('reason')=='ValueError:invalid_table_plane' and '"holding": "uncertain"' in journal and 'generic-verification-lift' in journal):raise ValueError('uncertain_lift_required')
    if set(saved['evidence'])!={'navigate_room','navigate_source','observe_target'}:raise ValueError('unexpected_resume_stage')
    if current['joints'][5]>=.60:raise ValueError('gripper_not_retained')
    stages=assembled['stages'];b=stages.grasp_backend
    closure=json.loads((root/'closure-contact.json').read_text())
    before=Path(closure['before_capture']);pre=json.loads((before/'snapshot.json').read_text())
    measurement=json.loads((before/'object-measurement.json').read_text())
    rejected=sorted((root/'captures').glob('rejected-track-*'),key=lambda p:int(p.name.rsplit('-',1)[1]))[-1]
    rs=json.loads((rejected/'snapshot.json').read_text());arrays=np.load(rejected/'registration-inputs.npz')
    R0,t0=b._tf(pre);R,t=b._tf(rs)
    old_palm=np.asarray(pre['base_from_palm_at_depth']['translation'])@R0.T+t0
    observed_palm=np.asarray(rs['base_from_palm_at_depth']['translation'])@R.T+t
    delta=observed_palm-old_palm
    b.template=arrays['template']@R.T+t-delta;b.colours=arrays['template_rgb']
    b.anchor=np.asarray(measurement['center_m'])@R0.T+t0
    from can_destination_surface import measure
    from local_handle_geometry import plane_base
    identity=root/'captures/identity-1'
    identity_snapshot=json.loads((identity/'snapshot.json').read_text())
    b.plane=plane_base(pre,measure(np.load(identity/'depth_m.npy'),identity_snapshot,b.table)['plane_map'])
    b.source_snapshot=pre;b.target_name=session.task.target;b.identity_confirmed=True;b.pose_locked=False
    b.opening=float(pre['joints']['hand_motor_joint'])
    closure=json.loads((root/'closure-contact.json').read_text())
    b.opening=float(closure['closure_evidence']['observed_opening_rad'])
    b.baseline=dict(anchor=b.anchor.copy(),template=b.template.copy(),palm=old_palm)
    b.lift_done=True;b.evidence=None;b.holding='uncertain';b.pre_lift_snapshot=pre;b.session_motion=stages.motion
    from support_reference import make_reference
    from hand_camera_ray import rotation
    palm=pre['base_from_palm_at_depth']
    b.source_support_reference=make_reference((b.template-t0)@R0,b.plane,palm['translation'],rotation(palm['quaternion']),pre['streams']['depth']['stamp_ns'])
    evidence=b.verify_lift()
    if not evidence.verified:
        b.return_uncertain_to_source()
        session.environment=session.task.layout;session.evidence={k:v for k,v in saved['evidence'].items() if k in ('navigate_room','navigate_source')}
        session._request('navigate_source')
        return dict(resumed_stage='navigate_source',uncertain_object_returned_to_source=True,requires_fresh_grasp=True)
    report=dict(status='grasp_verified',lift_evidence=vars(evidence),basis='same_round_preclose_reference_and_fresh_multiview_recheck')
    stages.context=b.held_context
    (stages.output/'held-context.json').write_text(json.dumps(stages.context))
    (stages.output/'grasp-result.json').write_text(json.dumps(report))
    result=grasp_result(report,target=session.task.target);result['held_context_path']=str(stages.output/'held-context.json')
    session.environment=session.task.layout;session.evidence=dict(saved['evidence'],grasp=result)
    session.emit('protocol',message='Object_grasped',detail='');session._request('tuck')
    return dict(resumed_stage='tuck',grasp_replayed=False,lift_replayed=False,current_lift_verified=True)
