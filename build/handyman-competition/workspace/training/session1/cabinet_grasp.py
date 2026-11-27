"""Cabinet grasps use the exported front, never a side or oblique approach."""
import math
import numpy as np

def is_cabinet(table):
    text=' '.join(str(table.get(k,'')) for k in ('id','semantic_class','model_path')).lower()
    return any(word in text for word in ('cabinet','cupboard','wardrobe','chest_of_drawers','drawer'))

def front_heading(table):
    yaw=table.get('front_yaw')
    if yaw is None or not math.isfinite(float(yaw)):raise ValueError('cabinet_front_direction_required')
    return math.atan2(math.sin(float(yaw)+math.pi),math.cos(float(yaw)+math.pi))

def front_aligned(table,pose,tolerance=.04):
    if not is_cabinet(table):return True
    heading=front_heading(table);a=float(table['front_yaw']);normal=np.array([math.cos(a),math.sin(a)])
    b=table.get('oriented_bounds') or table['axis_aligned_bounds'];c=b['center']
    delta=np.asarray(pose[:2])-[c['x'],c['y']]
    return float(delta@normal)>0 and abs(math.atan2(math.sin(pose[2]-heading),math.cos(pose[2]-heading)))<=tolerance
