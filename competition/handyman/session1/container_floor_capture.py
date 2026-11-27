"""Live external-floor observations through existing RGBD and resident detector.

Only observed floor around the destination is checked. Unseen space is not
certified, and no claim of seeing inside an opaque bin is made.
"""
import json
from pathlib import Path
import numpy as np

class ContainerFloorCapture:
    def __init__(self,backend,worker,destination,*,target='canned_juice',floor_z_m=0.,
                 radius_m=1.2,confidence=.5,map_point=None,opening=None,views=None):
        if map_point is None:
            from apple_transfer_auto_plan import map_point
        self.views=views
        self.backend=backend;self.worker=worker;self.destination=destination;self.target=target
        self.floor=floor_z_m;self.radius=radius_m;self.confidence=confidence;self.map_point=map_point;self.serial=0;self.opening=opening;self.last_rim_stamp=0;self.last_rim_targets=[]

    def __call__(self):
        self.serial+=1
        # Three views after withdrawing: center and both sides of the bin.
        pan=(0.,-.3,.3)[((self.serial-1)//2)%3]
        tilt=-.45 if self.serial%2 else -.8
        if self.views is not None:pan,tilt=self.views[((self.serial-1)//2)%len(self.views)]
        self.backend.rt.face_source((pan,tilt))
        item=self.backend._sample();snapshot=item['snapshot']
        capture=Path(self.backend.rt.save(item,f'container-floor-{self.serial:03d}'))
        detection=self.worker.infer(capture,pump=lambda:self.backend.rt.pump(0))
        if not isinstance(detection.get('detections'),list):raise ValueError('invalid_detector_response')
        (capture/'detections.json').write_text(json.dumps(detection))
        points,_,indices=self.backend._cloud(item)
        origin=np.asarray(self.map_point(snapshot,np.zeros(3)))
        basis=np.stack([np.asarray(self.map_point(snapshot,a))-origin for a in np.eye(3)],axis=1)
        mapped=points@basis.T+origin
        bounds=self.destination['axis_aligned_bounds'];c=bounds['center'];size=bounds['size']
        center=np.array([c['x'],c['y']]);half=np.array([size['x'],size['y']])/2
        external=(abs(mapped[:,:2]-center)>half).any(1)
        nearby=np.linalg.norm(mapped[:,:2]-center,axis=1)<=self.radius
        floor=external&nearby&(abs(mapped[:,2]-self.floor)<=.03)
        floor_targets=[];rim_targets=[];unlocalized=False
        rim_visible=False
        if self.opening is not None:
            rim=float(self.opening['rim_height_m']);mouth=np.asarray(self.opening['polygon_map_m'])
            close=(mapped[:,:2]>=mouth.min(0)-.06).all(1)&(mapped[:,:2]<=mouth.max(0)+.06).all(1)
            rim_band=close&(abs(mapped[:,2]-rim)<.04)
            rim_visible=int(rim_band.sum())>=10
        v,u=np.unravel_index(indices,item['depth'].shape)
        for d in detection['detections']:
            if d['name']!=self.target or d['confidence']<self.confidence:continue
            x1,y1,x2,y2=d['box'];roi=(u>=x1)&(u<x2)&(v>=y1)&(v<y2)
            if not roi.any():unlocalized=True;continue
            if self.opening is not None:
                # Depth in the central box avoids counting the bin itself as the detected object.
                core=roi&(u>=x1+.2*(x2-x1))&(u<x2-.2*(x2-x1))&(v>=y1+.2*(y2-y1))&(v<y2-.2*(y2-y1))
                perched=mapped[core&close&(mapped[:,2]>=rim-.025)&(mapped[:,2]<=rim+.35)]
                if len(perched)>=8:rim_targets.append(dict(target=self.target,confidence=d['confidence'],center_map_m=np.median(perched,axis=0).tolist(),evidence_ref=str(capture),location='rim'))
            object_points=mapped[roi&external&nearby&(mapped[:,2]>self.floor+.006)&(mapped[:,2]<self.floor+.3)]
            if len(object_points)<3:continue
            floor_targets.append(dict(target=self.target,confidence=d['confidence'],center_map_m=np.median(object_points,axis=0).tolist(),evidence_ref=str(capture)))
        stamp=snapshot['streams']['depth']['stamp_ns']
        if rim_visible:
            self.last_rim_stamp=stamp;self.last_rim_targets=list(rim_targets)
        elif self.opening is not None and 0<=stamp-self.last_rim_stamp<=3_000_000_000:
            rim_visible=True;rim_targets=list(self.last_rim_targets)
        row=dict(destination_id=self.destination['id'],depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
                 valid_frame=bool(item['rgb'].size and np.isfinite(mapped).all()),floor_visible=bool(floor.any()),
                 floor_targets=floor_targets,rim_targets=rim_targets,rim_visible=rim_visible,rim_observation_required=self.opening is not None,unlocalized_target=unlocalized,evidence_ref=str(capture),
                 floor_height_m=self.floor,observed_floor_points=int(floor.sum()),inspection_radius_m=self.radius,
                 head_pan_rad=pan,head_tilt_rad=tilt,rim_observation_stamp_ns=self.last_rim_stamp,unseen_space_certified=False)
        (capture/'external-floor-observation.json').write_text(json.dumps(row))
        return row
