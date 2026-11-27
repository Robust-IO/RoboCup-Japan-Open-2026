"""Re-grasp an actually observed rim/floor target through the shared grasp flow."""
from copy import deepcopy
from task_recovery import motion_cancel

def recover(stages,detections):
    row=next((d for d in detections if d.get('target')==stages.target and d.get('center_map_m')),None)
    if row is None:return False
    cancel=motion_cancel(stages.motion)
    if cancel is None:raise ValueError('container_recovery_requires_task_cancellation')
    backend=stages.grasp_backend
    if backend.holding_state()!='empty':return False
    table=deepcopy(stages.config['container_destination'])
    if row.get('location')!='rim':
        x,y,z=row['center_map_m'];table['id']='observed_floor_recovery#'+stages.config['destination_room']
        table['axis_aligned_bounds']={'center':{'x':x,'y':y,'z':-.01},'size':{'x':.5,'y':.5,'z':.02}}
        table.pop('oriented_bounds',None)
    else:
        bounds=table['axis_aligned_bounds'];top=stages.config['container_opening']['rim_height_m']
        bounds['center']['z']=top-bounds['size']['z']/2
    saved=(stages.source_id,stages.source_room,backend.table,stages.context)
    selector=stages.candidate_selector;previous=selector.tables.get(table['id'])
    selector.tables[table['id']]=table
    try:
        stages.source_id=table['id'];stages.source_room=stages.config['destination_room'];backend.table=table
        backend.identity_confirmed=False;backend.pose_locked=False
        serial=getattr(stages,'container_recovery_serial',0)+1;stages.container_recovery_serial=serial
        effect=dict(operation_id=f'container-recovery-{serial}',stage='grasp')
        observation=stages.observe_target(effect,cancel)
        if observation.get('confirmed') is not True:return False
        result=stages.grasp(effect,cancel)
        if result.get('grasp_verified') is not True:return False
        return stages.context
    finally:
        stages.source_id,stages.source_room,backend.table,old_context=saved
        if previous is None:selector.tables.pop(table['id'],None)
        else:selector.tables[table['id']]=previous
