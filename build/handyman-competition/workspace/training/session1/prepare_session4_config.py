"""Resolve Session4 furniture without reading target object truth coordinates."""
import argparse,hashlib,json
from pathlib import Path
from copy import deepcopy
from task_profile import resolve
from container_transfer import release_pose

def build(base,profile,semantic,environment,environment_path,opening=None):
    checked=resolve(profile,semantic,environment);c=deepcopy(base)
    c.update(profile='session4_lobby_tumbler_to_container',layout='LayoutC',
        task=dict(object='tumbler',source_room='lobby',destination='trash_box_for_burnable'),
        source_table_id='white_side_table#lobby',destination_id='trash_box_for_burnable#lobby',destination_room='lobby',
        environment=str(environment_path),session_profile=deepcopy(profile),
        container_destination=checked['resolved']['destination']['furniture'],
        source_staging_goal=checked['resolved']['source']['navigation_candidates'][0],
        destination_staging_goal=checked['resolved']['destination']['navigation_candidates'][0],
        scope='Session4 LayoutC known source furniture; RGBD target localization; shared container workflow.')
    # Never inherit Session2's physically different bin opening.
    c.pop('container_opening',None);c['container_geometry_pending']=opening is None
    if opening is not None:
        if opening['destination_id']!=c['destination_id']:raise ValueError('session4_opening_destination_mismatch')
        release_pose(opening,0.);c['container_opening']=deepcopy(opening)
    else:c['container_opening']={'destination_id':c['destination_id'],'source':'pending_session4_rgbd_measurement'}
    return c

def main():
    import yaml
    p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--environment',type=Path,required=True)
    p.add_argument('--semantic-map',type=Path,required=True);p.add_argument('--opening',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    profile=json.loads(Path(__file__).with_name('profiles').joinpath('session4.json').read_text())
    c=build(json.loads(a.base.read_text()),profile,json.loads(a.semantic_map.read_text()),yaml.safe_load(a.environment.read_text()),a.environment,json.loads(a.opening.read_text()) if a.opening else None)
    c['semantic_map']=str(a.semantic_map);c['environment_sha256']=hashlib.sha256(a.environment.read_bytes()).hexdigest()
    a.output.write_text(json.dumps(c,ensure_ascii=False,indent=2));print(json.dumps(dict(config=str(a.output),container_geometry_pending=c['container_geometry_pending'],robot_commands_sent=False)))
if __name__=='__main__':main()
