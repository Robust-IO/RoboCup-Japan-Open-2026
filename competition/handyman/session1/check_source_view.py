"""Live diagnostic: visit an exported source candidate and observe; no grasp/protocol."""
import argparse,json
from pathlib import Path
from threading import Event
from runtime_resources import RuntimeResources
from stage_bindings import StageBindings
p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--candidate',type=int,required=True);args=p.parse_args()
owner=RuntimeResources(args.config,Path('/__HANDYMAN_PACKAGE__/runtime/autograsp-v1/replay-inputs/hand.json'),args.output)
try:
 b=owner.open();rt=b['runtime'];m=b['motion_backend']
 stages=StageBindings(rt,m,b['grasp_backend'],b['config'],b['environment'],b['output'])
 goal=stages.source_candidates[args.candidate]
 effect={'operation_id':'view-check/1','stage':'navigate_source'}
 print(json.dumps({'event':'source_view_goal','goal':goal}),flush=True)
 stages.navigate(effect,Event(),goal,'living_room')
 result=stages.observe_target(effect,Event())
 (args.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
finally:owner.close()
