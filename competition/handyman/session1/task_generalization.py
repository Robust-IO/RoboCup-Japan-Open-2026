"""Offline task compiler: no ROS imports and no robot commands."""
from copy import deepcopy
from pathlib import Path
import argparse,datetime,json,os,shutil
import yaml
from execution_plan import furniture_candidates

LAYOUTS=('LayoutA','LayoutB','LayoutA','LayoutC','LayoutD','LayoutD')
def config_name(n):return 'session3-search-config.json' if n==3 else f'session{n}-live-config.json'
def validate_generalized(c):
    p=c['session_profile'];n=p['session_number'];t=c['task']
    if n not in range(1,7) or c['layout']!=LAYOUTS[n-1] or p['layout']!=c['layout']:raise ValueError('session_layout_mismatch')
    source=p['corrected_source'] if n==3 else p['source'];dest=p['destination']
    target=p['corrected_target'] if n==3 else p['target']
    if (target!=t['object'] or source['room']!=t['source_room'] or source['furniture_id']!=c['source_table_id'] or dest['furniture_id']!=c['destination_id'] or dest['room']!=c['destination_room']):raise ValueError('task_fields_not_synchronized')
    if c['policy'].get('carry_drop_recovery')!=(n in (2,5,6)):raise ValueError('drop_recovery_scope_mismatch')
    if not p.get('expected_instruction','').strip():raise ValueError('expected_instruction_required')
    if n==3:
        if dest['mode']!='avatar_handover' or dest['furniture_id']!='Moderator':raise ValueError('session3_requires_moderator_delivery')
        if not p.get('corrected_instruction') or not c.get('search_plan',{}).get('points'):raise ValueError('session3_search_and_correction_required')
    elif dest['mode'] not in ('table_place','container_drop'):raise ValueError('unsupported_destination_mode')
    if dest['mode']=='container_drop':
        from live_config import require_container_execution_ready
        require_container_execution_ready(c)
    if c.get('semantic_map') and c.get('environment'):
        semantic=json.loads(Path(c['semantic_map']).read_text())
        env=yaml.safe_load(Path(c['environment']).read_text())
        for role in ([source] if n==3 else [source,dest]):
            fid=role['furniture_id']
            if not any(x['id']==fid and x['room']==role['room'] for x in semantic['destinations']):raise ValueError('unknown_room_furniture:'+fid)
            furniture_candidates(env,fid)
    return c

def compile_task(base,spec,n,classes,geometry):
    c=deepcopy(base)
    if spec['session_number']!=n or spec['layout']!=LAYOUTS[n-1]:raise ValueError('fixed_layout_required')
    target=spec['target'];source=spec['source'];dest=spec['destination']
    if target not in classes:raise ValueError('unknown_model_class:'+target)
    env=yaml.safe_load(Path(c['environment']).read_text());semantic=json.loads(Path(c['semantic_map']).read_text())
    tables={x['id']:x for x in semantic['destinations']}
    def furniture(s):
        fid=s['furniture_id'];room=s['room']
        if fid not in tables or tables[fid]['room']!=room or fid.rsplit('#',1)[-1]!=room or room not in env['rooms']:raise ValueError('unknown_room_furniture:'+fid)
        return furniture_candidates(env,fid)[0]
    c['source_staging_goal']=furniture(source)
    p=deepcopy(c.get('session_profile') or {})
    p.update(schema='handyman_session3_search_v1' if n==3 else 'handyman_session_profile_v1',session_number=n,layout=spec['layout'],destination=deepcopy(dest),expected_instruction=spec['expected_instruction'])
    if n==3:
        # Existing search plan and Moderator geometry are calibrated to this room.
        # New room/recipient needs an explicitly rebuilt plan, not stale coordinates.
        if source['room']!=base['task']['source_room'] or dest['room']!=base['destination_room']:raise ValueError('rebuild_session3_search_or_handover_plan_before_changing_room')
        if spec['initial_target'] not in classes:raise ValueError('unknown_initial_target')
        p.update(target=spec['initial_target'],corrected_target=target,corrected_source=deepcopy(source),corrected_instruction=spec['corrected_instruction'])
    else:
        p.update(target=target,source=deepcopy(source))
        c['destination_staging_goal']=furniture(dest)
    c.update(task=dict(object=target,source_room=source['room'],destination=dest['furniture_id'].split('#')[0]),source_table_id=source['furniture_id'],destination_id=dest['furniture_id'],destination_room=dest['room'],session_profile=p,task_configuration_schema='handyman_configurable_tasks_v1')
    c.setdefault('policy',{})['carry_drop_recovery']=n in (2,5,6)
    if dest['mode']=='container_drop':
        key=c['layout']+'|'+dest['furniture_id']
        if key not in geometry:raise ValueError('calibrated_container_geometry_required:'+key)
        c['container_opening']=deepcopy(geometry[key]['opening']);c['container_destination']=deepcopy(geometry[key]['destination']);c['container_geometry_pending']=False
    else:
        for key in ('container_opening','container_destination','container_geometry_pending','session4_geometry_audit'):c.pop(key,None)
    return validate_generalized(c)

def compile_all(root,document):
    d=root/'workspace/training/session1'
    if document.get('schema')!='handyman_competition_tasks_v1':raise ValueError('unknown_tasks_schema')
    specs=document['sessions']
    if len(specs)!=6 or {x['session_number'] for x in specs}!=set(range(1,7)):raise ValueError('exactly_six_unique_sessions_required')
    names=yaml.safe_load((root/'project/training/dataset.yaml').read_text())['names'];classes=set(names.values() if isinstance(names,dict) else names)
    geometry=json.loads((root/'config/container-geometries.json').read_text())
    return {config_name(n):compile_task(json.loads((d/config_name(n)).read_text()),next(x for x in specs if x['session_number']==n),n,classes,geometry) for n in range(1,7)}

def main():
    root=Path(__file__).resolve().parents[3]
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,default=root/'competition_tasks.json');parser.add_argument('--apply',action='store_true');a=parser.parse_args()
    compiled=compile_all(root,json.loads(a.config.read_text()))
    if a.apply:
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:args=(proc/'cmdline').read_bytes().split(b'\0')
            except OSError:continue
            if any(Path(x.decode(errors='replace')).name in ('run_live_session1.py','run_dynamic_session.py','run_session3_search.py','run_sessions12.py') for x in args if x):raise RuntimeError('stop_active_task_before_applying_configuration')
        d=root/'workspace/training/session1';backup=root/'audit'/('task-config-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'));backup.mkdir(parents=True)
        for name in compiled:shutil.copy2(d/name,backup/name)
        try:
            for name,c in compiled.items():
                tmp=d/(name+'.pending');tmp.write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n');os.replace(tmp,d/name)
        except BaseException:
            for name in compiled:shutil.copy2(backup/name,d/name)
            raise
        print(json.dumps({'applied':6,'backup':str(backup),'robot_commands_sent':False}))
    else:print(json.dumps({'validated':6,'applied':False,'drop_recovery_sessions':[2,5,6],'robot_commands_sent':False}))
if __name__=='__main__':main()
