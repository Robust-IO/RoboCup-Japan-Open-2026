import json
import sys
import inspect
from pathlib import Path
import cv2
import numpy as np
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from canned_grasp_geometry import measure_near,predicted_before_contact
import tracked_can_cap_geometry as tracked
root=Path('/__HANDYMAN_PACKAGE__/runtime/can-extra-seat35-live-20261024-02')
cap=root/'grasp/captures/rejected-geometry-000'
read=lambda p:json.loads(p.read_text())
snapshot=read(cap/'snapshot.json');ref=read(root/'grasp/locked-can-reference.json')
expected=predicted_before_contact(ref,snapshot,'grasp')
rgb=cv2.cvtColor(cv2.imread(str(cap/'rgb.png')),cv2.COLOR_BGR2RGB);depth=np.load(cap/'depth_m.npy')
out=dict(expected=expected.tolist())
def trace(frame,event,arg):
    if event=='exception' and frame.f_code.co_filename.endswith('tracked_can_cap_geometry.py'):
        v=frame.f_locals
        if 'center' in v:
            for k in ('center','axis','actual_radius','residual','arc','capcenter','center2'):
                if k in v:out[k]=v[k].tolist() if isinstance(v[k],np.ndarray) else float(v[k])
            out['center_shift_m']=float(np.linalg.norm(v['center']-expected))
            out['metal_extent_p95_m']=float(np.quantile(np.linalg.norm(v['projection']-v['center2'],axis=1),.95))
            out['radius_extent_limit_m']=float(v['actual_radius']+.008)
            out['selected_cap_points']=len(v['selected']);out['side_points']=len(v['side'])
            out['cap_points_xy_span']=np.ptp(v['selected'],axis=0).tolist()
    return trace
source=inspect.getsource(tracked.measure)
source=source.replace("raise ValueError('current_cap_side_or_position_disagree')", "raise ValueError('current_cap_side_or_position_disagree:'+json.dumps(dict(center=center.tolist(),expected=expected.tolist(),axis=axis.tolist(),actual_radius=actual_radius,center_shift_m=float(np.linalg.norm(center-expected)),metal_extent_p95_m=float(np.quantile(np.linalg.norm(projection-center2,axis=1),.95)),selected_cap_points=len(selected),side_points=len(side))))")
namespace=dict(tracked.__dict__,json=json)
exec(source,namespace)
try:
    out['measurement']=namespace['measure'](depth,rgb,snapshot,expected,ref['radius_m'],ref['height_m'],ref['source_appearance'])
except Exception as exc:out['error']=str(exc)
(root/'geometry-diagnostic.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
