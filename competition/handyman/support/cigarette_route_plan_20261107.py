"""Plan a new destination using existing furniture/route data; no ROS/control."""
import json,math
from pathlib import Path

def plan(task,semantic,environment,base_config):
    if task.get('version')!=1 or task.get('object')!='cigarette' or task.get('layout')!=environment.get('internal_name'):
        raise ValueError('unsupported_route_test_task')
    # First route variant isolates destination/room changes and reuses the
    # verified source observation and thin-box grasp without global config edits.
    if task['source_table_id']!=base_config['source_table_id'] or task['source_room']!='bedroom':
        raise ValueError('this_route_variant_preserves_the_verified_bedroom_source')
    destinations={d['id']:d for d in semantic['destinations']}
    source=destinations[task['source_table_id']];dest=destinations[task['destination_id']]
    if source['room']!=task['source_room'] or dest['room']!=task['destination_room']:raise ValueError('furniture_room_mismatch')
    if dest['semantic_class'] not in ('dining_table','white_side_table','square_low_table'):raise ValueError('table_placement_adapter_required')
    route=next((r for r in environment['routes'] if r['from']==source['room'] and r['to']==dest['room']),None)
    if route is None:raise ValueError('configured_room_route_missing')
    b=dest['axis_aligned_bounds']
    goal=[b['center']['x']+b['size']['x']/2+.85,b['center']['y'],math.pi]
    if not all(math.isfinite(x) for x in goal):raise ValueError('invalid_destination_staging_goal')
    return dict(task=task,source_table=source['id'],destination_table=dest['id'],destination_room=dest['room'],
        destination_staging_goal=goal,carry_waypoints=[[w[k] for k in ('x','y','yaw')] for w in route['waypoints']],
        table_height_source='live_RGBD_only',object_position_source='live_detection_only',
        base_configuration_changed=False,robot_commands_sent=False,live_route_validated=False)

if __name__=='__main__':
    import argparse,yaml
    p=argparse.ArgumentParser();p.add_argument('--task',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text())
    result=plan(json.loads(a.task.read_text()),json.loads(Path(cfg['semantic_map']).read_text()),yaml.safe_load(Path(cfg['environment']).read_text()),cfg)
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False))
