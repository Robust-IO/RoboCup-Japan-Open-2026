"""Build Session2 config from existing environment plus reviewed bin opening.

Configuration only. Does not initialize ROS, switch maps, or move the robot.
"""
import argparse,hashlib,json
from pathlib import Path
from copy import deepcopy
from task_profile import resolve
from container_transfer import release_pose

def build(base,profile,semantic,environment,environment_path,opening):
    checked=resolve(profile,semantic,environment)
    if opening['destination_id']!=profile['destination']['furniture_id']:raise ValueError('opening_must_match_living_room_bin')
    release_pose(opening,0.)  # Validate finite usable polygon/height without commands.
    c=deepcopy(base)
    c.update(profile='session2_living_room_can_to_container',layout=profile['layout'],
             task=dict(object=profile['target'],source_room=profile['source']['room'],destination=profile['destination']['furniture_id'].split('#')[0]),
             source_table_id=profile['source']['furniture_id'],destination_id=profile['destination']['furniture_id'],
             destination_room=profile['destination']['room'],environment=str(environment_path),
             session_profile=deepcopy(profile),container_opening=deepcopy(opening),
             container_destination=checked['resolved']['destination']['furniture'],
             source_staging_goal=checked['resolved']['source']['navigation_candidates'][0],
             destination_staging_goal=checked['resolved']['destination']['navigation_candidates'][0])
    c['scope']='Session2 container live prototype; requires confirmed initial pose and correct LayoutB services.'
    return c

def main():
    import yaml
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True)
    p.add_argument('--environment',type=Path,required=True);p.add_argument('--semantic-map',type=Path,required=True)
    p.add_argument('--opening',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    profile=json.loads((Path(__file__).parent/'profiles/session2.json').read_text())
    c=build(json.loads(a.base.read_text()),profile,json.loads(a.semantic_map.read_text()),yaml.safe_load(a.environment.read_text()),a.environment,json.loads(a.opening.read_text()))
    c['semantic_map']=str(a.semantic_map);c['environment_sha256']=hashlib.sha256(a.environment.read_bytes()).hexdigest()
    a.output.write_text(json.dumps(c,ensure_ascii=False,indent=2));print(json.dumps({'config':str(a.output),'robot_commands_sent':False}))
if __name__=='__main__':main()
