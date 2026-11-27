"""Same-round release continuation; never repeat a completed table approach."""
import json
import math
from pathlib import Path
from threading import Event
import numpy as np

def restore(session,assembled,checkpoint,current):
    path=Path(checkpoint);saved=json.loads(path.read_text());root=path.parent
    if saved.get('state')!='stopped' or saved['task']!=session.snapshot()['task'] or saved.get('reason') not in ('UnobservedSurface:only_tiny_surface_fragment_visible','ValueError:destination_plane_changed'):
        raise ValueError('matching_pre_release_occlusion_checkpoint_required')
    if (root/'release-record.json').exists():raise ValueError('release_already_recorded')
    endpoint=json.loads((root/'session1-place-approach-0.json').read_text())
    if endpoint.get('status')!='endpoint_verified' or math.dist(current['poses']['map'][:2],endpoint['final_map'][:2])>.10:
        raise ValueError('current_pose_must_match_completed_approach')
    last=json.loads((root/'captures/destination-004-pre-release-support/snapshot.json').read_text())
    names=['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']
    if any(abs(a-last['joints'][n])>.04 for a,n in zip(current['joints'],names)) or current['joints'][5]>=.60:
        raise ValueError('held_posture_changed_since_checkpoint')
    evidence=saved['evidence'];context=json.loads(Path(evidence['grasp']['held_context_path']).read_text())
    stages=assembled['stages'];stages.context=context
    session.environment=session.task.layout;session.evidence=evidence;session._request('place')
    transfer=assembled['transfer'];transfer.initialize(session.effects[-1],Event())
    from placement_planner import plan_placement
    from transfer_actions import measured_release_shape
    from can_destination_surface import measure
    cap=root/'captures/destination-001-plan';snapshot=json.loads((cap/'snapshot.json').read_text())
    depth=np.load(cap/'depth_m.npy');volume=json.loads((cap/'object-measurement.json').read_text())
    shape=measured_release_shape(context,transfer.release_rotation);shape['holding_opening_rad']=current['joints'][5]
    plan=plan_placement(depth,snapshot,transfer.destination,measure(depth,snapshot,transfer.destination),shape,stages.config,'restore_completed_approach',held_volume=volume,outcome_policy=transfer.outcome_policy,**({'predict':transfer.core.predict} if hasattr(transfer.core,'predict') else {}))
    transfer.prepared_plan=plan
    (stages.output/'restored-placement-plan.json').write_text(json.dumps(plan))
    return dict(resumed_stage='place_release_only',approach_replayed=False,grasp_replayed=False,checkpoint=str(path),requires_current_release_geometry=True)
