"""Snapshot live source and explicit exception sites, without importing robot code."""
from competition_paths import is_run_path
import ast,hashlib,json,tarfile,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');frozen=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
roots=[b,frozen]
seeds=['start_fresh_cigarette_session_20261101','run_cigarette_transfer_20261101','navigate_cigarette_source_20261101','navigate_cigarette_bedroom_20261101','observe_grasp_object_20261101','flat_box_grasp','flat_box_place','flat_box_carry','outcome_finite_adjustment','inspect_transfer_ready','navigate_carry_trial','table_localization_service','table_localization_client','launch_cigarette_source_20261101','launch_flat_box_grasp_20261101','launch_flat_box_stage_20261101','horizontal_can_hand_trial','canned_grasp_runtime','table_localization','outcome_transfer_runtime','outcome_transfer_policy','source_table_front','flat_box_place_geometry','thin_box_clearance','thin_object_table_safety','stop_observe_act','capture_grasp_scene','synced_capture_selection','gripper_close_gate','table_approach_reachability']
pending=seeds[:];seen=set();files=[];exceptions=[]
stamp=time.strftime('%Y%m%d-%H%M%S');out=b/('transfer-gates-source-'+stamp);out.mkdir()
while pending:
    name=pending.pop(0)
    if name in seen:continue
    seen.add(name)
    candidates=[r/(name.replace('.','/')+'.py') for r in roots]
    source=next((p for p in candidates if p.is_file()),None)
    if source is None:continue
    text=source.read_text();tree=ast.parse(text);relative=source.relative_to(b) if is_run_path(source) else Path('frozen')/source.relative_to(frozen)
    target=out/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_text(text)
    files.append(dict(path=str(source),relative=str(relative),sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    for node in ast.walk(tree):
        if isinstance(node,ast.ImportFrom) and node.module:pending.append(node.module)
        elif isinstance(node,ast.Import):pending.extend(v.name for v in node.names)
        elif isinstance(node,ast.Raise):exceptions.append(dict(file=str(source),line=node.lineno,exception=ast.get_source_segment(text,node)))
for p in [b/'outcome_transfer_policy.json',b/'table_localization.json',frozen/'config/apple_transfer_auto_v1.json',b/'demo-standoff-recovery-deployment.json']:
    dest=out/('frozen-config.json' if p.parent.name=='config' else p.name);dest.write_bytes(p.read_bytes())
manifest=dict(robot_commands_sent=False,source_root=str(out),files=files,explicit_exception_sites=exceptions)
(out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
archive=out.with_suffix('.tar.gz')
with tarfile.open(archive,'w:gz') as tar:tar.add(out,arcname='active')
print(json.dumps(dict(archive=str(archive),files=len(files),exception_sites=len(exceptions))))
