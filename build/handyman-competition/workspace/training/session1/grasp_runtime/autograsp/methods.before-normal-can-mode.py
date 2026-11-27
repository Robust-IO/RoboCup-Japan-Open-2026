"""No ROS or motion: choose an execution family, never historical target XYZ."""
import json
from pathlib import Path

def select_method(target):
    target={'cannedjuice':'canned_juice'}.get(target,target)
    registry=json.loads((Path(__file__).resolve().parents[1]/'grasp_methods.json').read_text())
    profile=registry['profiles'].get(target)
    return dict(target=target,**(profile or registry['fallback']),known_profile=profile is not None)

def local_command(target,source_table,output,root,legacy_root,runs_root):
    p=select_method(target)
    if p['method']=='legacy_upright_can':
        return [str(root/'run_verified_can.py'),'--run','--simulation-ignore-collision',
                '--source-table',source_table,'--require-upright','--record-close-pose','--output',str(output)]
    if p['method']=='legacy_round_body':
        return [str(runs_root/'generic-grasp-v1/run_round_body_grasp.py'),'--target',p['target'],
                '--source-table',source_table,'--output',str(output)]
    if p['method'] in ('geometry','geometry_overhead'):return None
    raise ValueError('verified_method_requires_configured_entry: '+p['entry']+'; local adapter not integrated, do not silently replace verified method')
