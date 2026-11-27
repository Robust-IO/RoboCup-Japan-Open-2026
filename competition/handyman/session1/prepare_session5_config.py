"""Build an isolated LayoutD furniture binding using named static cupboard geometry."""
import argparse,json,hashlib,copy,math
from pathlib import Path

def build(base,profile,semantic,environment,environment_path):
 from task_profile import resolve
 resolved=resolve(profile,semantic,environment);c=copy.deepcopy(base)
 c.update(profile='session5_kitchen_rabbit_to_living_room',layout='LayoutD',task=dict(object='rabbit_doll',source_room='kitchen',destination='square_low_table'),source_table_id='body#kitchen',destination_id='square_low_table#living_room',destination_room='living_room',session_profile=copy.deepcopy(profile),environment=str(environment_path),source_staging_goal=resolved['resolved']['source']['navigation_candidates'][0],destination_staging_goal=resolved['resolved']['destination']['navigation_candidates'][0],scope='Known wooden cupboard source; RGBD object localization; cross-room table transfer. No object truth positions.')
 for k in ('container_opening','container_destination','container_geometry_pending','session4_geometry_audit'):c.pop(k,None)
 return c

def main():
 import yaml
 from session5_source_geometry import extract
 from source_view_candidates import near_front_candidates
 from candidate_selection import load_selector
 p=argparse.ArgumentParser()
 for name in ('base','scene','semantic-map','environment','output'):p.add_argument('--'+name,type=Path,required=True)
 a=p.parse_args();a.output.parent.mkdir(parents=True,exist_ok=True)
 source=extract(a.scene.read_text(encoding='utf-8-sig'));semantic=json.loads(a.semantic_map.read_text());env=yaml.safe_load(a.environment.read_text())
 if semantic['environment']!='LayoutD':raise ValueError('layout_d_semantic_required')
 semantic=copy.deepcopy(semantic);semantic['destinations']=[x for x in semantic['destinations'] if x['id']!=source['id']]+[source]
 candidates=near_front_candidates(source,env['rooms']['kitchen']['region'],standoff=.8)
 if not candidates:raise ValueError('no_kitchen_observation_candidate')
 env['destinations']['body']=[dict(room='kitchen',x=p[0],y=p[1],yaw=p[2]) for p in candidates]
 derived_env=a.environment.with_name('layout_d_session5.yaml');derived_sem=a.output.with_name('session5-semantic-map.json')
 profile=json.loads(Path(__file__).with_name('profiles').joinpath('session5.json').read_text())
 c=build(json.loads(a.base.read_text()),profile,semantic,env,derived_env)
 # Keep package URI resolution from the original environment location.
 derived_env.write_text(yaml.safe_dump(env,sort_keys=False));derived_sem.write_text(json.dumps(semantic,indent=2))
 c['semantic_map']=str(derived_sem);c['environment_sha256']=hashlib.sha256(derived_env.read_bytes()).hexdigest()
 selector=load_selector(c,semantic)
 initial=env.get('initial_pose')
 if initial is None:
  initial=semantic['robot_initial_pose']
 if isinstance(initial,dict):initial=[initial['x'],initial['y'],initial.get('yaw',0)]
 ranked,audit=selector.rank(initial,candidates,source['id'])
 if not ranked:raise ValueError('no_reachable_body_observation_candidate:'+json.dumps(audit))
 env['destinations']['body']=[dict(room='kitchen',x=p[0],y=p[1],yaw=p[2]) for p in ranked]
 derived_env.write_text(yaml.safe_dump(env,sort_keys=False));c['environment_sha256']=hashlib.sha256(derived_env.read_bytes()).hexdigest();c['source_staging_goal']=ranked[0]
 from live_config import validate
 validate(c);a.output.write_text(json.dumps(c,indent=2));a.output.with_name('session5-source-audit.json').write_text(json.dumps(dict(source=source,candidates=audit,robot_commands_sent=False),indent=2))
 print(json.dumps(dict(config=str(a.output),source=source['model_path'],candidate_count=len(ranked),source_bounds=source['axis_aligned_bounds'],support_height_m=source['support_height_m'],robot_commands_sent=False)))
if __name__=='__main__':main()
