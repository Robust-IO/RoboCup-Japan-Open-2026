"""Select new pickup or a live-verified held continuation; never opens the hand."""
import json,math
from pathlib import Path
import cv2
from outcome_transfer_policy import hold_assessment,holding_feedback

def candidate_context(base,receipt):
    """A previous result is context only, never sufficient proof of current hold."""
    base=Path(base);root=Path(receipt['output']).resolve()
    if not root.is_relative_to(base.resolve()):raise ValueError('task_context_outside_runtime')
    mission=json.loads((root/'result.json').read_text())
    grasp=Path(mission.get('grasp',''))
    if not grasp.is_absolute():return None
    prior=json.loads((grasp/'result.json').read_text())
    if not prior.get('grasp_success_verified') or prior.get('release_commands_sent')!=0:return None
    held=json.loads((grasp/'held-context.json').read_text())
    proof=Path(prior['lift_evidence'][-1]['capture'])
    if not proof.resolve().is_relative_to(grasp.resolve()):raise ValueError('invalid_lift_reference_path')
    carried=None;destination_pose=None
    if mission.get('carry'):
        path=Path(mission['carry']);r=json.loads((path/'result.json').read_text())
        if r.get('carry_destination_verified') and r.get('release_commands_sent')==0:
            carried=str(path);destination_pose=r['final_state']['poses']['map']
    return dict(grasp=str(grasp),carry=carried,destination_pose=destination_pose,held=held,reference=str(proof/'hand.png'))

def choose_held_entry(context,assessments,current_opening,*,at_destination=False):
    if not math.isfinite(current_opening):raise ValueError('finite_grip_feedback_required')
    if current_opening>=.60 or current_opening<=.01:return dict(mode='new_pick',initial_pose_is_acceptance_gate=False)
    if context is None:return dict(mode='wait',reason='partially_closed_hand_needs_current_object_context')
    try:holding_feedback([0,0,0,0,0,current_opening],context['held']['holding_opening_rad'])
    except ValueError:return dict(mode='wait',reason='current_hold_differs_reobserve')
    if len(assessments)<2 or any(r['state']!='consistent' for r in assessments[-2:]):
        return dict(mode='wait',reason='held_identity_not_yet_confirmed')
    return dict(mode='continue_held',grasp=context['grasp'],carry=context['carry'] if at_destination else None,
        initial_pose_is_acceptance_gate=False,current_hand_verified=True)

def inspect_entry(runtime,base):
    base=Path(base)
    while True:
        state=runtime.state();opening=state['joints'][5]
        if opening>=.60 or opening<=.01:return choose_held_entry(None,[],opening)
        context=None
        try:context=candidate_context(base,json.loads((base/'cigarette-full-auto-latest.json').read_text()))
        except (FileNotFoundError,KeyError,ValueError) as exc:runtime.recovery_status().waiting('held_context:'+str(exc))
        assessments=[]
        if context:
            reference=cv2.imread(context['reference']);last=0
            while len(assessments)<2:
                sample=runtime.sample(profile='hand',force_new=True);stamp=sample['snapshot']['streams']['hand']['stamp_ns']
                if last and stamp-last<100_000_000:runtime.pump(.05);continue
                assessments.append(hold_assessment(reference,sample['hand'],context['held']['appearance']));last=stamp
            cfg=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text())
            import yaml
            from scripts.search_room_gate import inside
            env=yaml.safe_load(Path(cfg['environment']).read_text())
            current=runtime.state()['poses']['map'];prior=context.get('destination_pose')
            at_destination=bool(prior and math.dist(current[:2],prior[:2])<=.15 and inside(*current[:2],env['rooms'][cfg['destination_room']]['region']))
        else:at_destination=False
        result=choose_held_entry(context,assessments,runtime.state()['joints'][5],at_destination=at_destination)
        if result['mode']!='wait':runtime.recovery_status().recovered();return result
        runtime.recovery_status().waiting(result['reason']);runtime.pump(.1)
