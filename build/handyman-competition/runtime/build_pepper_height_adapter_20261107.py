"""Back up the current sources and create an isolated dynamic-height adapter."""
import ast,fcntl,hashlib,json,shutil,sys,tarfile,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
lease=Path('/tmp/handyman-apple-transfer-automation.lock').open('a+');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
receipt=json.loads((b/'pepper-kitchen-live-latest.json').read_text());prior=Path(receipt['output']);result=json.loads((prior/'result.json').read_text())
if (result.get('stage')!='measure_destination_and_plan' or not result.get('grasp_success_verified') or result.get('release_commands_sent')!=0):raise ValueError('verified_unreleased_destination_hold_required')
stamp=time.strftime('%Y%m%d-%H%M%S');backup=b/'backups'/('before-dynamic-table-height-'+stamp);backup.mkdir()
shutil.copytree(f,backup/'frozen',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
for name in ('held-context.json','result.json','config.snapshot.json','run-meta.json','high-table-height-audit.json'):shutil.copy2(prior/name,backup/name)
archive=backup.with_suffix('.tar.gz')
with tarfile.open(archive,'w:gz') as t:t.add(backup,arcname='baseline')
adapter=b/('pepper-dynamic-height-adapter-'+stamp);shutil.copytree(backup/'frozen',adapter)
changes=[]
def edit(name,before,after):
    p=adapter/name;text=p.read_text()
    if text.count(before)!=1:raise ValueError('expected_source_not_unique:'+name+':'+before[:60])
    p.write_text(text.replace(before,after,1));changes.append(name)
edit('inverse_table_place_plan.py',
    "    if not all(math.isfinite(x) for x in (table_height,bottom_below_palm,gap)) or not .25<table_height<.75 or not 0<bottom_below_palm<.2 or not .005<=gap<=.03:raise ValueError('invalid_measured_release_geometry')\n    height=table_height+bottom_below_palm+gap\n    if not .35<=height<=.72:raise ValueError('unreachable_reviewed_release_height')\n    return height",
    "    from dynamic_table_height import release_height as measured_release_height\n    return measured_release_height(table_height,bottom_below_palm,gap)")
edit('pepper_runtime.py','not 0<=goal[0]<=.443','not 0<=goal[0]<=.69')
edit('pepper_runtime.py',
    "        lift=palm_height-OFFSET\n        if not .07<=lift<=.443:raise ValueError('pepper_destination_height_outside_reviewed_arm_range')\n        target[0]=.4428021848314039;self.move_joints(target,'raise_tucked_before_placement')",
    "        from dynamic_table_height import lift_targets\n        heights=lift_targets(palm_height,target[0]);lift=heights['release_lift_m']\n        self.record(dict(event='dynamic_table_height_plan',**heights))\n        target[0]=heights['extension_lift_m'];self.move_joints(target,'raise_tucked_before_placement')")
edit('pepper_runtime.py',
    "        target=self.state()['joints'];target[0]+=.08\n        return self.move_joints(target,'raise_open_hand')",
    "        from dynamic_table_height import LIFT_MAX_M,PREFERRED_CLEARANCE_M\n        target=self.state()['joints'];raise_by=min(PREFERRED_CLEARANCE_M,LIFT_MAX_M-target[0])\n        if raise_by<.001:\n            self.record(dict(event='no_extra_lift_headroom_withdraw_before_folding',actual_lift_m=target[0]))\n            return dict(status='already_at_physical_lift_limit',arm_commands_sent=False)\n        target[0]+=raise_by\n        return self.move_joints(target,'raise_open_hand')")
edit('reviewed_carry_adjustment.py','not .07<=arm[0]<=.443','not 0<=arm[0]<=.69')
shutil.copy2(b/'dynamic_table_height.py',adapter/'dynamic_table_height.py');changes.append('dynamic_table_height.py')
for name in set(changes):ast.parse((adapter/name).read_text())
report=dict(status='isolated_adapter_built',adapter=str(adapter),backup=str(backup),archive=str(archive),archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),predecessor=str(prior),
    files=[dict(name=n,sha256=hashlib.sha256((adapter/n).read_bytes()).hexdigest()) for n in sorted(set(changes))],
    canonical_sources_changed=False,robot_commands_sent=False,live_validated=False)
(adapter/'dynamic-height-adapter.json').write_text(json.dumps(report,indent=2));(b/'pepper-dynamic-height-adapter-latest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
