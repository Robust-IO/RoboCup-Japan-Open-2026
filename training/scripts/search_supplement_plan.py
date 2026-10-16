"""Bounded alternative viewpoints, not measured visibility/room coverage."""
import math
from active_search_policy import ring_views
from search_room_gate import RoomGate


def supplement_views(room_row,regions,room,maximum=1):
    if type(maximum) is not int or not 0<=maximum<=2:
        raise ValueError('invalid_supplement_limit')
    if maximum==0:return []
    gate=RoomGate(regions,room)
    selected=[room_row['main']];views=[]
    for p in room_row.get('backups',[]):
        if len(selected)-1>=maximum:break
        if gate.classify([p['x'],p['y'],0],[0,0,0],[0,0,0,1])['state']!='inside':continue
        if any(math.hypot(p['x']-q['x'],p['y']-q['y'])<1 for q in selected):continue
        selected.append(p)
        views.extend(ring_views(p['x'],p['y'],p['yaw']))
    return views
