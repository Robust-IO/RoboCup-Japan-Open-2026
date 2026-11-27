"""Session6: explicitly supplied source furniture, RGBD identity, shelf delivery."""
import argparse,json,copy,hashlib
from pathlib import Path
INSTRUCTION='Go to the lobby, grasp the filled_ketchup and put the filled_ketchup on the wooden_shelf in the living room.'
def profile(source):
 if not source.endswith('#lobby'):raise ValueError('session6_source_must_be_in_lobby')
 return dict(schema='handyman_session_profile_v1',session_number=6,layout='LayoutD',target='filled_ketchup',source=dict(room='lobby',furniture_id=source),destination=dict(room='living_room',furniture_id='wooden_shelf#living_room',mode='table_place'),expected_instruction=INSTRUCTION)
def build(base,source,semantic,environment,path):
 from task_profile import resolve
 p=profile(source);resolved=resolve(p,semantic,environment);c=copy.deepcopy(base)
 c.update(profile='session6_lobby_ketchup_to_living_room_shelf',layout='LayoutD',task=dict(object='filled_ketchup',source_room='lobby',destination='wooden_shelf'),source_table_id=source,destination_id='wooden_shelf#living_room',destination_room='living_room',session_profile=p,environment=str(path),source_staging_goal=resolved['resolved']['source']['navigation_candidates'][0],destination_staging_goal=resolved['resolved']['destination']['navigation_candidates'][0],scope='User supplied source furniture; measured target geometry; cross-room shelf placement.')
 for k in ('container_opening','container_destination','container_geometry_pending','session4_geometry_audit'):c.pop(k,None)
 from live_config import validate
 validate(c);return c

def main():
 import yaml
 p=argparse.ArgumentParser()
 for name in ('base','semantic-map','environment','output'):p.add_argument('--'+name,type=Path,required=True)
 p.add_argument('--source',required=True);a=p.parse_args()
 c=build(json.loads(a.base.read_text()),a.source,json.loads(a.semantic_map.read_text()),yaml.safe_load(a.environment.read_text()),a.environment)
 c['semantic_map']=str(a.semantic_map);c['environment_sha256']=hashlib.sha256(a.environment.read_bytes()).hexdigest()
 a.output.write_text(json.dumps(c,indent=2));print(json.dumps(dict(config=str(a.output),robot_commands_sent=False)))
if __name__=='__main__':main()
