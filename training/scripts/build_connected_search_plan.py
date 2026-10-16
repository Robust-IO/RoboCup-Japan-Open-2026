"""Regenerate connectivity and search plans together from current files; offline only."""
import argparse,hashlib,json
from pathlib import Path
import yaml
from analyze_search_connectivity import analyze
from build_search_plan import build
from static_room_policy import decide
from rehearse_search import rehearse

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();package=a.repo/'src/handyman_rebuild_ros2';plans=[]
    paths=sorted((package/'config/environments').glob('*.yaml'))
    if not paths: p.error('No environment YAML found')
    for path in paths:
        content=path.read_bytes();env=yaml.safe_load(content)
        prefix='package://handyman_rebuild_ros2/'
        if not env['map'].startswith(prefix): raise ValueError('Unexpected map reference')
        map_path=(package/env['map'][len(prefix):]).resolve()
        report=analyze(env,map_path);plan=build(env)
        plan.update(source=str(path),source_sha256=hashlib.sha256(content).hexdigest(),
                    map_bundle_sha256=report['map_bundle_sha256'],connectivity=report)
        for room in plan['rooms']:
            room['static_policy']=decide(report,room['room'],report['map_bundle_sha256'],enable_static_rule=True)
        plans.append(plan)
        print(plan['layout'],[(r['room'],r['static_policy']['decision']) for r in plan['rooms']],flush=True)
    result=dict(schema='connected-search-offline-v1',plans=plans,rehearsal=rehearse(plans),actionable=False,
                note='Frozen offline snapshot; regenerate after map, room, or initial-pose changes. No protocol sender.')
    with a.output.open('x') as stream: json.dump(result,stream,indent=2,allow_nan=False)
    from collections import Counter
    print('Rehearsal:',dict(Counter(r['state'] for r in result['rehearsal']['results'])))
    print('Saved:',a.output)

if __name__=='__main__': main()
