"""Offline profile resolution. Furniture poses never supply object coordinates."""
from execution_plan import furniture_candidates

def resolve(profile,semantic,environment):
    if profile.get('schema')!='handyman_session_profile_v1':raise ValueError('unknown_profile_schema')
    resolved={}
    for role in ('source','destination'):
        spec=profile[role];fid=spec['furniture_id']
        if '#' not in fid or fid.rsplit('#',1)[1]!=spec['room']:raise ValueError('explicit_furniture_room_required')
        matches=[x for x in semantic['destinations'] if x['id']==fid and x['room']==spec['room']]
        if len(matches)!=1:raise ValueError('unique_room_qualified_furniture_required:'+fid)
        resolved[role]=dict(furniture=matches[0],navigation_candidates=furniture_candidates(environment,fid))
    if profile['destination']['mode'] not in ('table_place','container_drop'):raise ValueError('unsupported_destination_mode')
    return dict(session_number=profile['session_number'],layout=profile['layout'],target=profile['target'],
                destination_mode=profile['destination']['mode'],resolved=resolved,
                configuration_resolved=True,execution_ready=False,robot_commands_sent=False)
