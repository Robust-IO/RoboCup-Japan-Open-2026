"""Configured furniture-to-furniture penguin tasks; no object coordinates or ROS."""
import copy
import hashlib
import json
import math
from pathlib import Path

DEFAULT_TASK=Path(__file__).parent/'config/penguin_bedroom_to_living_room.json'
REVERSE_TASK=Path(__file__).parent/'config/penguin_living_room_to_bedroom.json'


def validate_task(task):
    if (not isinstance(task,dict) or set(task)!={'version','name','layout','object','source','destination'}
            or task['version']!=1 or task['layout']!='LayoutA' or task['object']!='toy_penguin'
            or not isinstance(task['name'],str) or not task['name']):
        raise ValueError('invalid_penguin_task')
    for side in ('source','destination'):
        endpoint=task[side]
        required={'room','table_id','observation_pose'}|({'head_pose'} if side=='source' else set())
        if not isinstance(endpoint,dict) or set(endpoint)!=required:
            raise ValueError('invalid_task_endpoint:'+side)
        if any(not isinstance(endpoint[k],str) or not endpoint[k] for k in ('room','table_id')):
            raise ValueError('invalid_task_furniture:'+side)
        pose=endpoint['observation_pose']
        if (not isinstance(pose,list) or len(pose)!=3
                or any(type(v) not in (int,float) or not math.isfinite(v) for v in pose)):
            raise ValueError('invalid_task_observation_pose:'+side)
    head=task['source']['head_pose']
    if (not isinstance(head,list) or len(head)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in head)
            or not -.6<=head[0]<=.6 or not -.8<=head[1]<=0):
        raise ValueError('invalid_task_head_pose')
    if task['source']['table_id']==task['destination']['table_id']:
        raise ValueError('source_and_destination_must_differ')
    return copy.deepcopy(task)


def load_task(path=None):
    return validate_task(json.loads(Path(path or DEFAULT_TASK).read_text()))


def configured_transfer(baseline,task,environment,tables):
    def inside(x,y,polygon):
        result=False
        for a,b in zip(polygon,polygon[1:]+polygon[:1]):
            if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:result=not result
        return result
    task=validate_task(task)
    if environment['internal_name']!=task['layout']:raise ValueError('task_layout_mismatch')
    for side in ('source','destination'):
        endpoint=task[side];room=endpoint['room']
        selected=[table for table in tables if table['id']==endpoint['table_id']]
        if len(selected)!=1 or selected[0]['room']!=room or room not in environment['rooms']:
            raise ValueError('task_furniture_room_mismatch:'+side)
        if not inside(*endpoint['observation_pose'][:2],environment['rooms'][room]['region']):
            raise ValueError('task_observation_pose_outside_room:'+side)
    a,b=task['source']['room'],task['destination']['room']
    if a!=b and not any(r['from']==a and r['to']==b and r['waypoints'] for r in environment['routes']):
        raise ValueError('task_room_route_missing')
    cfg=copy.deepcopy(baseline)
    cfg.update(profile='configured_toy_penguin_transfer_v1',
        task=dict(object='toy_penguin',source_room=a,destination=task['destination']['table_id'].split('#')[0]),
        source_room=a,destination_room=b,source_table_id=task['source']['table_id'],
        destination_id=task['destination']['table_id'],source_staging_goal=task['source']['observation_pose'],
        destination_staging_goal=task['destination']['observation_pose'],source_head_pose=task['source']['head_pose'],
        configured_task=task,joint_feedback_max_age_s=.6,navigation_joint_feedback_max_age_s=.85,navigation_feedback_recovery=True)
    return cfg


def task_signature(config):
    return dict(source_room=config.get('source_room',config['task']['source_room']),
        destination_room=config['destination_room'],source_table_id=config['source_table_id'],
        destination_id=config['destination_id'],source_staging_goal=config['source_staging_goal'],
        destination_staging_goal=config['destination_staging_goal'],source_head_pose=config.get('source_head_pose',[0.,-.5]))


def validate_predecessor_task(config,previous_config):
    if task_signature(config)!=task_signature(previous_config):raise ValueError('continuation_task_changed')


def task_digest(task):
    return hashlib.sha256(json.dumps(validate_task(task),sort_keys=True,separators=(',',':')).encode()).hexdigest()
