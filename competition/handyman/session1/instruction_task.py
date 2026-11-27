"""Deterministic competition grammar and room inventory routing; no ROS."""
import re

def normalized(text):return re.sub(r'\s+',' ',text.strip().lower()).rstrip('.!?')
def words(name):return name.lower().replace('_',' ')
def mentions(text,name):
    return re.search(r'(?<![a-z0-9])'+re.escape(words(name))+r'(?![a-z0-9])',words(text)) is not None

def parse_instruction(text,classes,rooms,furniture):
    s=normalized(text)
    match=re.fullmatch(r'go to (?:the )?(.+?)\s*,?\s+grasp (.+?)\s+and\s+(put|throw|give)\s+(.+)',s)
    if not match:raise ValueError('unsupported_instruction_grammar')
    room_text,grasp,verb,delivery=match.groups();room_text=room_text.rstrip(',').strip()
    room_matches=[r for r in rooms if words(r)==words(room_text)]
    if len(room_matches)!=1:raise ValueError('unknown_source_room:'+room_text)
    room=room_matches[0];targets=[c for c in classes if mentions(grasp,c)]
    # Names such as empty_ketchup and filled_ketchup remain distinct tokens.
    if len(targets)!=1:raise ValueError('ambiguous_or_unknown_target')
    target=targets[0]
    def resolve_furniture(fragment,room_hint=None):
        primary=re.split(r'\b(?:close to|near|next to)\b',fragment,maxsplit=1)[0]
        matches=[f for f in furniture if mentions(primary,f['id'].split('#')[0])]
        if room_hint:matches=[f for f in matches if f['room']==room_hint]
        if len(matches)!=1:raise ValueError('ambiguous_or_unknown_furniture:'+fragment)
        return matches[0]
    source=None
    loc=re.search(r'\b(?:on|in)\s+(?:the )?(.+)$',grasp)
    if loc:source=resolve_furniture(loc.group(1),room)['id']
    if verb=='give' and re.search(r'\bto me\b',delivery):
        destination=dict(furniture_id='Moderator',room=None,mode='avatar_handover')
    else:
        prep=re.search(r'\b(on|in|into)\s+(?:the )?(.+)$',delivery)
        if not prep:raise ValueError('destination_preposition_required')
        fragment=prep.group(2);explicit_rooms=[r for r in rooms if mentions(fragment,r)]
        if len(explicit_rooms)>1:raise ValueError('ambiguous_destination_room')
        hint=explicit_rooms[0] if explicit_rooms else None
        try:f=resolve_furniture(fragment,hint)
        except ValueError:
            if hint is not None:raise
            # Competition grammar leaves the room implicit for same-room delivery.
            # Only accept a unique match within the explicitly requested source room.
            f=resolve_furniture(fragment,room)
        mode='container_drop' if verb=='throw' or f['id'].startswith('trash_box') else 'table_place'
        destination=dict(furniture_id=f['id'],room=f['room'],mode=mode)
    return dict(raw_instruction=text,target=target,source_room=room,source_table=source,destination=destination)

def source_route(task,inventory):
    """Inventory absence changes search order; NEVER authorizes Does_not_exist."""
    if task['source_table']:
        return dict(route='explicit_furniture',furniture_ids=[task['source_table']],room_search_required_on_miss=True)
    rows=[r for r in inventory.get('objects',[]) if r['target']==task['target'] and r.get('room')==task['source_room']]
    uncertain=any(r['target']==task['target'] and r.get('room') is None for r in inventory.get('objects',[]))
    complete=inventory.get('complete') is True
    if not rows and complete and not uncertain:
        return dict(route='room_search',inventory_presence='absent_in_preset',furniture_ids=[],absence_authorized=False)
    bound=sorted({r['furniture_id'] for r in rows if r.get('furniture_id')})
    return dict(route='bound_then_candidates_then_room' if bound else 'candidates_then_room',inventory_presence='present_in_preset' if rows else 'unknown',furniture_ids=bound,absence_authorized=False)
