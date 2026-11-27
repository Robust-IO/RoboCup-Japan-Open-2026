"""Independent replay of can-colored features, excluding gripper/silver ROI."""
import argparse,json,sys
from pathlib import Path
import cv2,numpy as np
p=argparse.ArgumentParser();p.add_argument('--base',type=Path,default=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime'));p.add_argument('--code',type=Path,default=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'))
p.add_argument('--after-root',type=Path);p.add_argument('--output-name',default='can08-green-depth-final-verification-20261024-01');args=p.parse_args()
sys.path.insert(0,str(args.code))
from canned_grasp_geometry import transform_point
from hand_camera_ray import rotation
read=lambda p:json.loads(p.read_text())
root=args.base/'can08-deeper-lift-probe-20261024-01';out=args.base/args.output_name;out.mkdir(exist_ok=False)
source=root/'captures/before-lift';snap0=read(source/'snapshot.json')
def load(path):return dict(rgb=cv2.cvtColor(cv2.imread(str(path/'rgb.png')),cv2.COLOR_BGR2RGB),
    hand=cv2.imread(str(path/'hand.png')),depth=np.load(path/'depth_m.npy'),snapshot=read(path/'snapshot.json'))
def green(rgb):
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    return ((hsv[:,:,0]>=40)&(hsv[:,:,0]<=54)&(hsv[:,:,1]>=70)&(hsv[:,:,2]>=30)).astype(np.uint8)*255
def xyz(sample,pixels):
    s=sample['snapshot'];k=np.asarray(s['rgb_k']).reshape(3,3);pix=np.rint(pixels).astype(int);h,w=sample['depth'].shape
    valid=(pix[:,0]>=0)&(pix[:,0]<w)&(pix[:,1]>=0)&(pix[:,1]<h);z=np.full(len(pix),np.nan)
    z[valid]=sample['depth'][pix[valid,1],pix[valid,0]];valid&=np.isfinite(z)&(z>0)&(z<3.)
    camera=np.c_[(pixels[:,0]-k[0,2])*z/k[0,0],(pixels[:,1]-k[1,2])*z/k[1,1],z]
    tf=s['base_from_rgb'];pts=camera@rotation(tf['quaternion']).T+tf['translation'];result=np.full_like(pts,np.nan)
    for i in np.flatnonzero(valid):result[i]=transform_point(s,pts[i])
    return result,valid
before=load(source);head0=cv2.cvtColor(before['rgb'],cv2.COLOR_RGB2GRAY);mask=green(before['rgb']);roi=np.zeros_like(mask);roi[305:385,230:335]=255;mask&=roi
head_points=cv2.goodFeaturesToTrack(head0,maxCorners=100,qualityLevel=.001,minDistance=1,mask=mask,blockSize=3)
if head_points is None:raise ValueError('no_can_colored_head_texture')
f0=head_points[:,0,:];xyz0,valid=xyz(before,f0);f0=f0[valid];xyz0=xyz0[valid]
if len(f0)<5:raise ValueError('too_few_actual_green_head_depth_features')
hand0=cv2.cvtColor(before['hand'],cv2.COLOR_BGR2GRAY);handmask=green(cv2.cvtColor(before['hand'],cv2.COLOR_BGR2RGB))
h0=cv2.goodFeaturesToTrack(hand0,maxCorners=200,qualityLevel=.001,minDistance=5,mask=handmask,blockSize=3)
if h0 is None or len(h0)<20:raise ValueError('no_can_colored_hand_texture')
rows=[];palm0=transform_point(snap0,snap0['palm_base_at_depth'])
paths=sorted(args.after_root.glob('frame-*')) if args.after_root else sorted((root/'captures').glob('after-lift-*'))
for path in paths:
    after=load(path);head1=cv2.cvtColor(after['rgb'],cv2.COLOR_RGB2GRAY)
    tracked,ok,_=cv2.calcOpticalFlowPyrLK(head0,head1,f0.astype(np.float32).reshape(-1,1,2),None,winSize=(21,21),maxLevel=3)
    reverse,back,_=cv2.calcOpticalFlowPyrLK(head1,head0,tracked,None,winSize=(21,21),maxLevel=3)
    f1=tracked[:,0,:];xyz1,valid=xyz(after,f1)
    good=ok[:,0].astype(bool)&back[:,0].astype(bool)&valid&(np.linalg.norm(reverse[:,0,:]-f0,axis=1)<1.5)
    if good.sum()<5:raise ValueError('too_few_green_depth_tracks')
    delta=xyz1[good]-xyz0[good];motion=np.median(delta,axis=0);inliers=np.linalg.norm(delta-motion,axis=1)<.015
    if inliers.sum()<5 or inliers.mean()<.6:raise ValueError('green_surface_motion_not_consistent')
    motion=np.median(delta[inliers],axis=0)
    s=after['snapshot'];palm=transform_point(s,s['palm_base_at_depth'])-palm0;error=float(np.linalg.norm(motion-palm))
    hand1=cv2.cvtColor(after['hand'],cv2.COLOR_BGR2GRAY)
    h1,hok,_=cv2.calcOpticalFlowPyrLK(hand0,hand1,h0,None,winSize=(21,21),maxLevel=3)
    hr,hback,_=cv2.calcOpticalFlowPyrLK(hand1,hand0,h1,None,winSize=(21,21),maxLevel=3)
    hg=hok[:,0].astype(bool)&hback[:,0].astype(bool)&(np.linalg.norm(hr[:,0,:]-h0[:,0,:],axis=1)<1.5)
    relative_flow=np.linalg.norm(h1[:,0,:]-h0[:,0,:],axis=1)[hg]
    hand_pass=len(relative_flow)>=20 and np.quantile(relative_flow,.95)<=2.
    head_pass=motion[2]>=.03 and palm[2]>=.03 and error<=.02
    hand_after_mask=green(cv2.cvtColor(after['hand'],cv2.COLOR_BGR2RGB))>0
    hand_before_mask=handmask>0
    rows.append(dict(capture=str(path),green_head_depth_features=int(inliers.sum()),
        green_surface_delta_odom_m=motion.tolist(),actual_palm_delta_odom_m=palm.tolist(),
        green_surface_palm_error_m=error,green_hand_features=len(relative_flow),
        green_hand_relative_flow_median_px=float(np.median(relative_flow)),
        green_hand_relative_flow_p95_px=float(np.quantile(relative_flow,.95)),
        green_hand_mask_iou=float(np.sum(hand_before_mask&hand_after_mask)/np.sum(hand_before_mask|hand_after_mask)),
        head_depth_follow_pass=bool(head_pass),hand_pixel_flow_pass=bool(hand_pass)))
if len(rows)!=3:raise ValueError('three_actual_frames_required')
passed=all(r['head_depth_follow_pass'] for r in rows)
result=dict(status='actual_green_can_depth_follows_lift_verified' if passed else 'independent_green_depth_audit_failed',rows=rows,
    source_features_only_can_green_pixels=True,gripper_features_excluded=True,silver_roi_not_used=True,
    robot_commands_sent=False,current_grasp_follow_verified=passed,full_object_pose_estimated=False,
    original_probe_roi_included_noncan_pixels=True,
    acceptance='Three frames of real3D motion on source-colored can body; original20mm palm-follow limit. Hand optical-flow percentiles remain diagnostics and are not object3D measurements.',
    hand_pixel_flow_outlier_warning=not all(r['hand_pixel_flow_pass'] for r in rows))
with (out/'result.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result),flush=True)
