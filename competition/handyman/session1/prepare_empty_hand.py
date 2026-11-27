"""Explicitly open empty gripper after user-confirmed Session1 initial reset."""
import argparse,json
from pathlib import Path
from runtime_resources import RuntimeResources

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--user-confirmed-initial-empty',action='store_true');a=p.parse_args()
    if not a.user_confirmed_initial_empty:raise ValueError('initial_empty_confirmation_required')
    owner=RuntimeResources(a.config,Path('/__HANDYMAN_PACKAGE__/runtime/autograsp-v1/replay-inputs/hand.json'),a.output)
    try:
        from startup_hand import prepare
        report=prepare(owner.open()['runtime'])
        report['user_confirmed_initial_empty']=True
        (a.output/'result.json').write_text(json.dumps(report));print(json.dumps(report))
    finally:owner.close()
if __name__=='__main__':main()
