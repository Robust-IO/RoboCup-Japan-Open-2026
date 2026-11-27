"""Reachable, same-room nearby views; no truth-object coordinates."""
import math
import numpy as np
from perception_bridge import inside

def choose_near_view(pose,target,polygon,selector):
    target=np.asarray(target,float);bearing=math.atan2(pose[1]-target[1],pose[0]-target[0]);poses=[]
    for radius in (.7,.85,1.):
        for offset in (0.,math.pi/6,-math.pi/6,math.pi/3,-math.pi/3,math.pi):
            angle=bearing+offset;x=target[0]+radius*math.cos(angle);y=target[1]+radius*math.sin(angle)
            if not inside(x,y,polygon):continue
            # Sight line excludes the final object/support footprint, inspected at arrival.
            clear=True
            for q in np.linspace([x,y],target[:2],30)[:22]:
                cell=selector.cell(q)
                if not selector.valid(cell) or not selector.free[cell]:clear=False;break
            if clear:poses.append([x,y,math.atan2(target[1]-y,target[0]-x)])
    lengths=selector.lengths(selector.cell(pose),[selector.cell(p) for p in poses])
    valid=[p for p in poses if selector.cell(p) in lengths]
    if not valid:return None
    p=min(valid,key=lambda p:lengths[selector.cell(p)])
    return dict(x=float(p[0]),y=float(p[1]),yaw=float(p[2]))
