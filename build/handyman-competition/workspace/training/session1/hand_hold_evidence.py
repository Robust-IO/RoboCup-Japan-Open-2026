"""Hand-camera appearance continuity inside the projected measured object surface.

This is a supplemental check, not a depth measurement or proof of force/contact.
Failure means uncertain, never a fabricated positive or confirmed floor location.
"""
import numpy as np
import cv2

def object_mask(snapshot,shape,context,parts,rotation):
    from grasp_runtime.autograsp.tracking import visible_reference
    from hand_camera_ray import HAND_RASTER_TO_FRAME
    tf=snapshot['palm_from_hand'];r=rotation(tf['quaternion']);origin=np.asarray(tf['translation'])
    points=np.asarray(context['vertices_palm_m'],float)
    visible=visible_reference(points,origin,[np.asarray(p['vertices_palm_m']) for p in parts if not p.get('is_trigger')])
    optical=((points-origin)@r)@HAND_RASTER_TO_FRAME
    valid=visible&(optical[:,2]>.005)
    if np.count_nonzero(valid)<15:raise ValueError('held_surface_not_visible_in_hand_camera')
    k=np.asarray(snapshot['hand_k']).reshape(3,3)
    if np.any(np.asarray(snapshot['hand_d'])!=0) or snapshot['streams']['hand']['frame']!='hand_camera_rgb_frame':raise ValueError('unsupported_hand_projection')
    uv=optical[valid]@k.T;uv=uv[:,:2]/uv[:,2:]
    h,w=shape[:2];uv=uv[(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)]
    if len(uv)<15:raise ValueError('held_surface_outside_hand_image')
    mask=np.zeros((h,w),np.uint8);cv2.fillConvexPoly(mask,cv2.convexHull(np.round(uv).astype(np.int32)),255)
    mask=cv2.erode(mask,np.ones((5,5),np.uint8))
    if np.count_nonzero(mask)<100:raise ValueError('insufficient_hand_object_pixels')
    return mask

def compare(reference,current,mask):
    if reference.shape!=current.shape or mask.shape!=current.shape[:2]:raise ValueError('hand_reference_shape_changed')
    # Small slips in the grasp are tolerated; background replacement is not.
    best=0.
    for dx in (-4,0,4):
        for dy in (-4,0,4):
            shifted=cv2.warpAffine(current,np.float32([[1,0,dx],[0,1,dy]]),(current.shape[1],current.shape[0]))
            a=reference[mask>0].astype(float);b=shifted[mask>0].astype(float)
            if len(a)<100:raise ValueError('insufficient_hand_comparison_pixels')
            colour=float(np.mean(np.linalg.norm(a-b,axis=1)<60.))
            # Spatial texture continuity helps reject similarly coloured furniture.
            aa=a-a.mean(axis=0);bb=b-b.mean(axis=0)
            denom=float(np.linalg.norm(aa)*np.linalg.norm(bb))
            corr=float(np.sum(aa*bb)/denom) if denom>1e-6 else 0.
            best=max(best,min(colour,max(0.,corr)))
    return dict(appearance_match_score=best,matched=best>=.60)

class HandHoldEvidence:
    def __init__(self,backend,context,rotation):self.backend=backend;self.context=context;self.rotation=rotation
    def __call__(self,label):
        rt=self.backend.rt;item=rt.sample(profile='hand',force_new=True);s=item['snapshot'];rt.fresh_measurement(s)
        opening=float(s['joints']['hand_motor_joint'])
        if abs(opening-self.context['holding_opening_rad'])>.03:raise ValueError('current_grip_changed')
        image=item['hand'];parts=self.backend.physical.sample(opening)['parts']
        mask=object_mask(s,image.shape,self.context,parts,self.rotation)
        stamp=s['streams']['hand']['stamp_ns'];ref=self.context.get('hand_hold_reference')
        if ref is None:
            if label!='before-tuck' or self.context.get('lift_evidence',{}).get('basis')!='lift_and_reclose_joint_feedback':
                raise ValueError('fresh_verified_grasp_hand_reference_required')
            # This is continuity reference creation immediately after verified lift,
            # not a newly inferred successful grasp from the hand image alone.
            path=rt.save(item,'carry-hand-reference')
            cv2.imwrite(str(__import__('pathlib').Path(path)/'object-mask.png'),mask)
            self.context['hand_hold_reference']=dict(path=str(path),stamp_ns=stamp,transform=s['palm_from_hand'])
            return dict(holding_verified=True,visual_confirmation=True,source='fresh_lift_hand_reference',evidence_ref=str(path),hand_stamp_ns=stamp)
        if stamp<=ref['stamp_ns']:raise ValueError('fresh_hand_frame_required')
        if not all(np.allclose(s['palm_from_hand'][k],ref['transform'][k],atol=.002) for k in ('translation','quaternion')):
            raise ValueError('hand_camera_geometry_changed')
        from pathlib import Path
        reference=cv2.imread(str(Path(ref['path'])/'hand.png'));oldmask=cv2.imread(str(Path(ref['path'])/'object-mask.png'),0)
        if reference is None or oldmask is None:raise ValueError('hand_reference_missing')
        metrics=compare(reference,image,cv2.bitwise_and(mask,oldmask))
        path=rt.save(item,'carry-hand-check-'+str(stamp),metrics)
        if not metrics['matched']:raise ValueError('held_object_visual_uncertain:'+str(path))
        return dict(holding_verified=True,visual_confirmation=True,source='current_hand_surface_appearance_continuity',
            evidence_ref=str(path),hand_stamp_ns=stamp,**metrics)
