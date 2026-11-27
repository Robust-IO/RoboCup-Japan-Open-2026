"""Build/test the paired-camera compatibility repair; no physical command."""
import ast,json,os,shutil,subprocess,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
prior=json.loads((b/'penguin-kitchen-live-latest.json').read_text());old=Path(prior['output'])
result=json.loads((old/'result.json').read_text())
if result.get('reason')!="'base_from_odom_at_hand'" or result.get('close_commands_sent')!=1 or result.get('release_commands_sent')!=0:
    raise ValueError('specific_unlifted_contact_failure_required')
root=b/('penguin-contact-resume-adapter-'+time.strftime('%Y%m%d-%H%M%S'))
shutil.copytree(prior['source_adapter'],root,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
for name in ('penguin_contact_resume_20261107.py','test_penguin_contact_resume_20261107.py'):
    shutil.copy2(b/name,root/name)
p=root/'profiled_capture.py';text=p.read_text()
a="        requests += [('base_from_hand','base_link',hand.header.frame_id,h),('palm_from_hand','hand_palm_link',hand.header.frame_id,h)]"
z=a+"\n        requests.append(('base_from_odom_at_hand','base_link','odom',h))"
if text.count(a)!=1:raise ValueError('capture_source_not_unique')
text=text.replace(a,z).replace("name.startswith(('odom_from_base','map_from_base'))", "name.startswith(('odom_from_base','map_from_base','base_from_odom'))")
p.write_text(text)
p=root/'pepper_transfer_automation.py';text=p.read_text()
def replace(a,z):
    global text
    if text.count(a)!=1:raise ValueError('source_not_unique:'+a[:100])
    text=text.replace(a,z,1)
replace("navigation_profile=None,source_imgsz=640):", "navigation_profile=None,source_imgsz=640,continue_closed_contact=None):")
replace("    report['continued_from_open_pregrasp']=continue_open_pregrasp is not None", "    report['continued_from_closed_contact']=continue_closed_contact is not None\n    report['continued_from_open_pregrasp']=continue_open_pregrasp is not None")
replace("                    if continue_open_pregrasp is not None:\n", """                    if continue_closed_contact is not None:
                        from penguin_contact_resume_20261107 import load_resume
                        restored=load_resume(continue_closed_contact,cfg,meta['boot_id'],PepperRuntime,root,record)
                        runtime=restored['runtime'];state=restored['state']
                        backend.start_room=backend.room_at(state['poses']['map']);backend.initial_opening=state['joints'][5]
                        if backend.start_room!=source_room:raise ValueError('closed_contact_must_remain_in_source_room')
                        report.update(historical_checkpoint_used=True,predecessor_run=str(continue_closed_contact),
                            previous_close_commands_sent=1,source_contact_reused_without_reclosing=True)
                        write_json(preflight,dict(ready=True,current_state=state,previous_close_commands=1,
                            actual_contact_state_checked=True,grasp_lift_not_yet_verified=True,actions_replayed=False))
                    elif continue_open_pregrasp is not None:
""")
replace("                    if not observe_only and continue_open_pregrasp is None:", "                    if not observe_only and continue_open_pregrasp is None and continue_closed_contact is None:")
replace("                    if not observe_only:\n", "                    if not observe_only and continue_closed_contact is None:\n")
start=text.index("                    folder('grasp');stage('new_'+object_name+'_rgbd')")
end=text.index("                    hand_contact=None",start)
block=text[start:end].replace("                    folder('grasp');", "                    ",1)
wrapped="                    folder('grasp')\n                    if continue_closed_contact is None:\n"+''.join('    '+line if line.strip() else line for line in block.splitlines(True))
wrapped+="""                    else:
                        model=restored['model'];before=restored['before'];before_sample=restored['before_sample']
                        close=restored['close'];holding=restored['holding']
                        report['close']=close;report['source_observations']=restored['source_observations']
                        write_json(runtime.root/'source-reference.json',model)
                        record(dict(event='previous_closed_contact_verified_without_another_close',
                            original_preclose_capture=close['before_capture'],current_grip=holding))
"""
text=text[:start]+wrapped+text[end:]
replace("uninterrupted_transfer_verified=(continue_open_pregrasp is None and continue_destination_hold is None", "uninterrupted_transfer_verified=(continue_closed_contact is None and continue_open_pregrasp is None and continue_destination_hold is None")
replace("    p.add_argument('--continue-open-pregrasp',type=Path)", "    p.add_argument('--continue-open-pregrasp',type=Path)\n    p.add_argument('--continue-closed-contact',type=Path)")
replace("    if sum(x is not None for x in (args.continue_open_pregrasp,", "    if sum(x is not None for x in (args.continue_closed_contact,args.continue_open_pregrasp,")
replace("    if args.run and args.observe_only:p.error('choose one run mode')", """    if args.run and args.observe_only:p.error('choose one run mode')
    if args.continue_closed_contact is not None and (object_name!='toy_penguin' or not args.run
            or not args.continue_closed_contact.resolve().is_relative_to(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime'))):
        p.error('closed contact resume requires the verified penguin D-drive checkpoint')""")
replace("navigation_profile=args.navigation_profile,source_imgsz=args.source_imgsz)", "navigation_profile=args.navigation_profile,source_imgsz=args.source_imgsz,continue_closed_contact=args.continue_closed_contact)")
ast.parse(text);p.write_text(text)
env=dict(os.environ,PYTHONPATH=str(root)+':'+str(b))
checks=subprocess.run(['/usr/bin/python3','-m','unittest','test_penguin_contact_resume_20261107','test_penguin_hand_hold','test_penguin_geometry','test_penguin_source_identity','test_pepper_transfer','-q'],env=env,cwd='/tmp',capture_output=True,text=True,timeout=120)
(root/'contact-resume-validation.log').write_text(checks.stdout+checks.stderr)
if checks.returncode:raise RuntimeError((checks.stdout+checks.stderr)[-5000:])
# Replay the ACTUAL preclose RGB/hand/3D, then fit its hand appearance before
# authorizing any live continuation. Original capture files remain untouched.
replay="""
import json,sys,cv2
from pathlib import Path
from penguin_contact_resume_20261107 import complete_recorded_hand_inverse
from penguin_hand_hold import create_reference
p=Path(sys.argv[1]);root=Path(sys.argv[2]);d=json.loads((p/'result.json').read_text());capture=Path(d['close']['before_capture'])
s=complete_recorded_hand_inverse(json.loads((capture/'snapshot.json').read_text()))
m=json.loads((capture/'object-measurement.json').read_text());model=json.loads((p/'grasp/source-reference.json').read_text())
proof=create_reference(cv2.imread(str(capture/'hand.png')),s,m,root/'recorded-preclose-face.png',model=model)
(root/'recorded-preclose-face-proof.json').write_text(json.dumps(proof,indent=2));print(json.dumps(proof))
"""
replay_result=subprocess.run(['/usr/bin/python3','-c',replay,str(old),str(root)],env=env,capture_output=True,text=True)
(root/'recorded-preclose-replay.log').write_text(replay_result.stdout+replay_result.stderr)
if replay_result.returncode:raise RuntimeError((replay_result.stdout+replay_result.stderr)[-5000:])
receipt=dict(status='contact_resume_adapter_offline_validated',adapter=str(root),predecessor=str(old),
    task=prior['task'],prior_receipt=prior,tests=(checks.stdout+checks.stderr)[-500:],recorded_preclose_hand_proof=True,
    canonical_sources_changed=False,robot_commands_sent=False)
(b/'penguin-contact-resume-adapter-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
