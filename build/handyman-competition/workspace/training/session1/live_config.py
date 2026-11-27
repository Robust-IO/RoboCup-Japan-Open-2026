"""Known session profiles accepted by the live runner; no side effects."""
def validate(config):
    if config.get("task_configuration_schema")=="handyman_configurable_tasks_v1":
        from task_generalization import validate_generalized
        return validate_generalized(config)
    if config.get('session_profile',{}).get('schema')=='handyman_session3_search_v1':
        if config.get('source_binding_requires_reexport'):
            raise ValueError('corner_sofa_export_binding_needs_refresh')
        if config.get('layout')!='LayoutA' or config.get('task',{}).get('source_room')!='lobby' or not config.get('search_plan',{}).get('points'):
            raise ValueError('session3_lobby_search_configuration_required')
        return config
    if config.get('session_profile',{}).get('session_number')==6:
        p=config['session_profile'];source=p.get('source',{});dest=p.get('destination',{})
        if (config.get('layout')!='LayoutD' or p.get('layout')!='LayoutD' or
            config.get('task',{}).get('object')!='filled_ketchup' or p.get('target')!='filled_ketchup' or
            source.get('room')!='lobby' or not source.get('furniture_id','').endswith('#lobby') or
            config.get('source_table_id')!=source.get('furniture_id') or
            config.get('task',{}).get('source_room')!='lobby' or
            config.get('destination_id')!='wooden_shelf#living_room' or config.get('destination_room')!='living_room' or
            dest.get('furniture_id')!='wooden_shelf#living_room' or dest.get('room')!='living_room' or dest.get('mode')!='table_place'):
            raise ValueError('matching_session6_profile_required')
        return config
    expected={
        'LayoutD':('rabbit_doll','body#kitchen','square_low_table#living_room'),
        'LayoutA':('dog_doll','white_side_table#living_room','square_low_table#living_room'),
        'LayoutB':('canned_juice','wooden_side_table#living_room','trash_box_for_bottle_can#living_room'),
        'LayoutC':('tumbler','white_side_table#lobby','trash_box_for_burnable#lobby'),
    }
    actual=(config.get('task',{}).get('object'),config.get('source_table_id'),config.get('destination_id'))
    if expected.get(config.get('layout'))!=actual:raise ValueError('known_session_configuration_required')
    if config['layout']=='LayoutD':
        p=config.get('session_profile',{})
        if (p.get('target')!='rabbit_doll' or p.get('layout')!='LayoutD' or p.get('source',{}).get('furniture_id')!='body#kitchen' or p.get('source',{}).get('room')!='kitchen' or p.get('destination',{}).get('furniture_id')!='square_low_table#living_room' or p.get('destination',{}).get('room')!='living_room' or p.get('destination',{}).get('mode')!='table_place'):
            raise ValueError('matching_session5_profile_required')
    if config['layout'] in ('LayoutB','LayoutC'):
        p=config.get('session_profile',{})
        if (p.get('target')!=actual[0] or p.get('layout')!=config['layout'] or
            p.get('source',{}).get('furniture_id')!=actual[1] or
            p.get('destination',{}).get('furniture_id')!=actual[2] or
            p.get('destination',{}).get('mode')!='container_drop'):
            raise ValueError('matching_container_session_profile_required')
        if config.get('container_opening',{}).get('destination_id')!=actual[2]:raise ValueError('container_opening_required')
    return config


def require_container_execution_ready(config):
    if config.get('session_profile',{}).get('destination',{}).get('mode')!='container_drop':return
    if config.get('container_geometry_pending'):raise ValueError('measure_current_session_container_before_execution')
    from container_transfer import release_pose
    opening=config.get('container_opening',{})
    if opening.get('destination_id')!=config.get('destination_id'):raise ValueError('container_opening_destination_mismatch')
    release_pose(opening,0.)
