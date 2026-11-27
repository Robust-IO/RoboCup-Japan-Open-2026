"""Static RGBD landmarks predict a near-search ROI despite simulated odom noise.

The returned target points are predictions from an older observed object and a
CURRENT background transform. They never replace fitting current object depth.
"""
import cv2
import numpy as np
from hand_camera_ray import rotation
from canned_grasp_geometry import transform_point


def fit_rigid(previous,current):
    a=np.asarray(previous,float);b=np.asarray(current,float)
    if a.ndim!=2 or a.shape!=b.shape or a.shape[1]!=3 or len(a)<15 or not np.isfinite([a,b]).all():
        raise ValueError('insufficient_static_rgbd_matches')
    def solve(indices):
        aa=a[indices];bb=b[indices];ac=aa.mean(0);bc=bb.mean(0)
        u,s,v=np.linalg.svd((aa-ac).T@(bb-bc))
        if s[1]<1e-5:raise ValueError('collinear_static_landmarks')
        flip=np.eye(3);flip[2,2]=np.linalg.det(v.T@u.T)
        r=v.T@flip@u.T
        return r,bc-r@ac
    rng=np.random.default_rng(711);best=None
    for _ in range(96):
        try:r,t=solve(rng.choice(len(a),3,replace=False))
        except ValueError:continue
        residual=np.linalg.norm(a@r.T+t-b,axis=1);selected=residual<.012
        if best is None or selected.sum()>best.sum():best=selected
    if best is None or best.sum()<max(15,.55*len(a)):raise ValueError('static_background_motion_not_consistent')
    for _ in range(3):
        r,t=solve(best);residual=np.linalg.norm(a@r.T+t-b,axis=1);best=residual<.012
        if best.sum()<max(15,.55*len(a)):raise ValueError('static_background_motion_not_consistent')
    # Pixel matches next to depth edges can survive the broad RANSAC band.
    # Refine on the tight measured consensus, rather than rejecting a good
    # transform because its broad preliminary set includes those edge points.
    refined=residual<.006
    if refined.sum()<max(15,.5*len(a)):raise ValueError('static_background_tight_consensus_missing')
    r,t=solve(refined);residual=np.linalg.norm(a@r.T+t-b,axis=1);best=refined
    extent=np.linalg.svd(a[best]-a[best].mean(0),compute_uv=False)
    if extent[1]/np.sqrt(best.sum())<.05:raise ValueError('static_background_spatial_extent_insufficient')
    p95=float(np.quantile(residual[best],.95))
    if p95>.008:raise ValueError('static_background_fit_residual')
    return r,t,dict(matches=len(a),inliers=int(best.sum()),residual_p95_m=p95,
        source='current_rgbd_static_landmarks',object_depth_measured=False,motion_authorized=False)


def _features(sample,excluded):
    rgb=sample['rgb'];depth=sample['depth'];s=sample['snapshot']
    mask=np.ones(depth.shape,np.uint8)*255;mask[int(depth.shape[0]*.74):]=0
    points,desc=cv2.SIFT_create(nfeatures=1200).detectAndCompute(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY),mask)
    if desc is None:raise ValueError('static_background_features_not_observed')
    xy=np.asarray([k.pt for k in points]);uv=np.rint(xy).astype(int)
    k=np.asarray(s['rgb_k']).reshape(3,3);z=depth[uv[:,1],uv[:,0]]
    camera=np.c_[(xy[:,0]-k[0,2])*z/k[0,0],(xy[:,1]-k[1,2])*z/k[1,1],z]
    tf=s['base_from_rgb'];p=camera@rotation(tf['quaternion']).T+tf['translation']
    good=np.isfinite(p).all(1)&(z>.05)&(z<4.)&(p[:,0]>.65)&(p[:,2]>.40)
    good&=np.linalg.norm(p-excluded,axis=1)>.15
    for i,(u,v) in enumerate(uv):
        if good[i] and np.ptp(depth[max(0,v-1):v+2,max(0,u-1):u+2])>.03:good[i]=False
    if good.sum()<15:raise ValueError('static_background_depth_not_observed')
    return p[good],desc[good]


class NearAnchor:
    def __init__(self,sample,measurement):
        self.sample=sample;self.measurement=measurement;self.features=None

    def predict(self,current,odom_body,odom_cap):
        old=self.sample['snapshot'];new=current['snapshot'];m=self.measurement
        if (old['tracking_epoch']!=new['tracking_epoch']
                or new['streams']['depth']['stamp_ns']<=m['stamp_ns']):
            raise ValueError('same_task_new_visual_motion_frame_required')
        tf_old=old['tracking_transforms']['odom_from_base_at_depth'];tf_new=new['tracking_transforms']['odom_from_base_at_depth']
        translation=float(np.linalg.norm(np.array(tf_new['translation'])-tf_old['translation']))
        relative=rotation(tf_new['quaternion']).T@rotation(tf_old['quaternion'])
        angle=float(np.arccos(np.clip((np.trace(relative)-1)/2,-1,1)))
        if translation<.003 and angle<.003:return None
        if self.features is None:self.features=_features(self.sample,np.asarray(m['center_m']))
        a,da=self.features;b,db=_features(current,np.asarray(odom_body))
        pairs=[x for x,y in cv2.BFMatcher().knnMatch(da,db,k=2) if x.distance<.65*y.distance]
        if len(pairs)<15:raise ValueError('static_background_matches_not_observed')
        r,t,evidence=fit_rigid(a[[x.queryIdx for x in pairs]],b[[x.trainIdx for x in pairs]])
        body=r@np.asarray(m['center_m'])+t;cap=r@np.asarray(m['cap_center_m'])+t
        correction=float(np.linalg.norm(body-odom_body))
        if correction>max(.08,.25*translation):raise ValueError('visual_motion_disagrees_with_bounded_odom_prior')
        evidence.update(reference_depth_stamp_ns=m['stamp_ns'],current_depth_stamp_ns=new['streams']['depth']['stamp_ns'],
            odom_prediction_correction_m=correction,reference_epoch=old['tracking_epoch'],
            prediction_only=True,current_object_depth_still_required=True)
        odom_step=rotation(tf_old['quaternion']).T@(np.asarray(tf_new['translation'])-tf_old['translation'])
        actual_step=-r.T@t
        distance=float(np.linalg.norm(odom_step[:2]));actual_distance=float(np.linalg.norm(actual_step[:2]))
        if distance>=.02 and .7<=actual_distance/distance<=1.3:
            angle=np.arctan2(actual_step[1],actual_step[0])-np.arctan2(odom_step[1],odom_step[0])
            angle=float(np.arctan2(np.sin(angle),np.cos(angle)))
            if abs(angle)<=.35:
                evidence.update(controller_to_visual_angle_rad=angle,
                    commanded_step_old_base_m=odom_step.tolist(),actual_step_old_base_m=actual_step.tolist())
        return body,cap,evidence
