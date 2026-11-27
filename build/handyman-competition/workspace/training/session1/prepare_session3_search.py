"""Build search config from maps and existing main/backup generator; no ROS."""
import argparse,json,hashlib,ast
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from session3_search_policy import choose_points

def main():
 p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--base-config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 from build_active_search_points import select_points
 c=json.loads(a.base_config.read_text());share=a.repo/'src/handyman_rebuild_ros2'
 envpath=share/'config/environments/layout_a.yaml';env=yaml.safe_load(envpath.read_text())
 mp=share/env['map'].removeprefix('package://handyman_rebuild_ros2/');meta=yaml.safe_load(mp.read_text());image=mp.parent/meta['image']
 pixels=np.flipud(np.array(Image.open(image)));prob=pixels/255. if meta['negate'] else (255.-pixels)/255.;free=prob<meta['free_thresh']
 params=yaml.safe_load((a.repo/'src/handyman_ros2/param/nav2_params.yaml').read_text())['global_costmap']['global_costmap']['ros__parameters']
 footprint=params['footprint'];footprint=ast.literal_eval(footprint) if isinstance(footprint,str) else footprint
 radius=max(np.hypot(*p) for p in footprint)+params.get('footprint_padding',.01)
 initial=env['initial_pose'];rooms=select_points(free,meta['resolution'],meta['origin'],[initial['x'],initial['y']],env['rooms'],radius)
 room=next(r for r in rooms if r['room']=='lobby')
 profile=json.loads((Path(__file__).parent/'profiles/session3.json').read_text())
 plan=choose_points(room,env['rooms']['lobby']['region'],profile['search']['aspect_ratio_threshold'])
 plan['map_bundle_sha256']=hashlib.sha256(mp.read_bytes()+b'\0'+image.read_bytes()).hexdigest();plan['map_yaml']=str(mp);plan['map_image']=str(image)
 c.update(profile='session3_search',session_profile=profile,task={'object':'dog_doll','source_room':'lobby','destination':'Moderator'},
   source_table_id='corner_sofa#lobby',destination_id='Moderator',destination_room='lobby',search_plan=plan,
   environment=str(envpath),environment_sha256=hashlib.sha256(envpath.read_bytes()).hexdigest())
 c['scope']='Search bear, wait for correction, then visually confirm and grasp dog at user-specified corner_sofa. Deliver while holding to Moderator using the separately prepared handover config.'
 a.output.write_text(json.dumps(c,indent=2));print(json.dumps(plan))
if __name__=='__main__':main()
