"""Navigate empty-handed to configured bin and capture geometry views; never release."""
import argparse,json
from pathlib import Path
from runtime_resources import RuntimeResources
from candidate_selection import load_selector
from execution_plan import furniture_candidates

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    owner=RuntimeResources(a.config,Path('/__HANDYMAN_PACKAGE__/runtime/autograsp-v1/replay-inputs/hand.json'),a.output)
    try:
        b=owner.open();rt=b['runtime'];motion=b['motion_backend'];cfg=b['config']
        rt.sample(profile='state');state=rt.state()
        if max(abs(x-y) for x,y in zip(state['joints'][:5],[0.,0.,0.,-1.57079632679,0.]))>.05:raise ValueError('initial_empty_posture_required')
        goal=list(state['joints']);goal[5]=.65
        rt.move_joints(goal,'inspection-initial-empty-hand',hand=True)
        rt.sample(profile='state');state=rt.state()
        source=motion.room_at(state['poses']['map'])
        selector=load_selector(cfg,b['semantic']);candidates=furniture_candidates(b['environment'],cfg['destination_id'])
        ranked,audit=selector.rank(state['poses']['map'],candidates,cfg['destination_id'])
        (a.output/'candidate-selection.json').write_text(json.dumps(audit,indent=2))
        if not ranked:raise ValueError('no_reachable_bin_view')
        motion.stage='navigate_destination'
        result=motion.navigate(source,cfg['destination_room'],ranked[0],state['joints'][5],'container-inspection')
        (a.output/'navigation-result.json').write_text(json.dumps(result,indent=2))
        if not result.get('verified'):raise ValueError('inspection_navigation_failed')
        for i,tilt in enumerate([-.5,-.7]):
            rt.face_source((0.,tilt));item=b['grasp_backend']._sample();rt.save(item,f'bin-view-{i}')
        print(json.dumps({'status':'captured','output':str(a.output),'bin_goal':ranked[0],'grasp_or_release_sent':False}),flush=True)
    finally:owner.close()
if __name__=='__main__':main()
