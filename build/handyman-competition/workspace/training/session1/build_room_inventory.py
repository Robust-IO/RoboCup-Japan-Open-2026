"""Import configured Unity placements, not template transforms or task answer names."""
from pathlib import Path
import argparse,json,hashlib
import yaml
from perception_bridge import inside
from execution_plan import furniture_candidates
from task_generalization import config_name

def bind_point(xyz,room,semantic):
    x,y,z=xyz;matches=[]
    for f in semantic['destinations']:
        if f['room']!=room:continue
        b=f['axis_aligned_bounds'];c=b['center'];size=b['size']
        if abs(x-c['x'])<=size['x']/2+.025 and abs(y-c['y'])<=size['y']/2+.025:matches.append(f['id'])
    return matches[0] if len(matches)==1 else None

def main():
    root=Path(__file__).resolve().parents[3];p=argparse.ArgumentParser();p.add_argument('unity_config_dir',type=Path);a=p.parse_args()
    out=root/'config/room-inventories';out.mkdir(exist_ok=True);reports=[]
    for n in range(1,7):
        cfg=json.loads((root/'workspace/training/session1'/config_name(n)).read_text());path=a.unity_config_dir/f'EnvironmentInfo{n:02}.json';raw=path.read_bytes();data=json.loads(raw.decode('utf-8-sig'))
        if data['environmentName']!=cfg['layout']:raise ValueError('layout_mismatch:'+str(path))
        if 'graspablesPositions' not in data:raise ValueError('placement_list_missing')
        env=yaml.safe_load(Path(cfg['environment']).read_text());sem=json.loads(Path(cfg['semantic_map']).read_text());objects=[]
        for item in data['graspablesPositions']:
            pos=item['position'];xyz=[pos['z'],-pos['x'],pos['y']]
            rooms=[k for k,v in env['rooms'].items() if inside(*xyz[:2],v['region'])];room=rooms[0] if len(rooms)==1 else None
            fid=bind_point(xyz,room,sem) if room else None
            if fid:
                try:furniture_candidates(env,fid)
                except ValueError:fid=None
            objects.append(dict(target=item['name'].split('#')[0],instance=item['name'],room=room,furniture_id=fid,preset_position_map_m=xyz))
        report=dict(schema='handyman_room_inventory_v1',session_number=n,layout=cfg['layout'],complete=True,assumption='preset_positions_not_randomized',source_sha256=hashlib.sha256(raw).hexdigest(),source_filename=path.name,objects=objects)
        (out/f'session{n}.json').write_text(json.dumps(report,indent=2)+'\n');reports.append(dict(session=n,objects=len(objects),bound=sum(x['furniture_id'] is not None for x in objects),unassigned_room=sum(x['room'] is None for x in objects)))
    print(json.dumps(reports))
if __name__=='__main__':main()
