"""One5cm lift preserving the reviewed deeper grip; current surface tracking.

No repeated closure, release, navigation or full-transfer success claim.
The manually reviewed cap ROI selects real textured surface points; tracked
RGBD points, not a predicted object center, decide whether the can follows.
"""
import json,math,sys,time
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime,TOPICS,NAMES
from canned_grasp_geometry import fresh,transform_point
from hand_camera_ray import rotation
from motion_speed import arm_duration
from task_outcome_policy import pose_matches
import rclpy
base=Path('/__HANDYMAN_PACKAGE__/runtime');closed=base/'can08-deeper-close-20261024-01'
prior=json.loads((closed/'result.json').read_text())
if prior['status']!='deeper_close_contact_candidate_requires_lift_verification' or prior['close_commands_sent']!=1:
    raise ValueError('one_actual_deeper_closure_contact_required')
root=base/'can08-deeper-lift-probe-20261024-01';root.mkdir(exist_ok=False)
report=dict(status='starting',mode='user_reviewed_diagnostic_lift',source_close=str(closed),
    lift_command_sent=False,close_repeated=False,hand_commands_sent=False,lift_requested_m=.05,
    grasp_success_verified=False,physical_transfer_verified=False,task_finished_sent=False,
    manually_reviewed_cap_roi=[278,326,333,368],predicted_object_center_used_as_measurement=False)
def points(sample,pixels):
    snap=sample['snapshot'];k=np.asarray(snap['rgb_k']).reshape(3,3);pix=np.rint(pixels).astype(int)
    valid=(pix[:,0]>=0)&(pix[:,0]<sample['depth'].shape[1])&(pix[:,1]>=0)&(pix[:,1]<sample['depth'].shape[0])
    z=np.full(len(pix),np.nan);z[valid]=sample['depth'][pix[valid,1],pix[valid,0]]
    valid&=np.isfinite(z)&(z>0)&(z<3.)
    camera=np.c_[(pixels[:,0]-k[0,2])*z/k[0,0],(pixels[:,1]-k[1,2])*z/k[1,1],z]
    tf=snap['base_from_rgb'];xyz=camera@rotation(tf['quaternion']).T+tf['translation']
    transformed=np.full_like(xyz,np.nan)
    for i in np.flatnonzero(valid):transformed[i]=transform_point(snap,xyz[i])
    return transformed,valid
with RobotLock(),(root/'journal.jsonl').open('x') as journal:
    def record(x):
        row=dict(time_ns=time.time_ns(),**x);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    rclpy.init();rt=CanRuntime(root,root.name,record);pub=None
    try:
        before=rt.sample();state=rt.state();old=prior['final_state']
        if (not pose_matches(state['joints'],old['joints']) or math.dist(state['poses']['odom'][:2],old['poses']['odom'][:2])>.02
                or abs(state['poses']['odom'][2]-old['poses']['odom'][2])>.03):raise ValueError('deeper_grip_or_base_changed')
        holding=state['joints'][5];report['holding_opening_rad']=holding
        gray=cv2.cvtColor(before['rgb'],cv2.COLOR_RGB2GRAY);hsv=cv2.cvtColor(before['rgb'],cv2.COLOR_RGB2HSV)
        mask=np.zeros(gray.shape,np.uint8);mask[326:368,278:333]=255
        mask[(hsv[:,:,1]>100)|(hsv[:,:,2]<80)]=0
        features=cv2.goodFeaturesToTrack(gray,maxCorners=60,qualityLevel=.01,minDistance=2,mask=mask,blockSize=3)
        if features is None:raise ValueError('actual_cap_texture_not_available')
        f0=features[:,0,:];xyz0,valid=points(before,f0);f0=f0[valid];xyz0=xyz0[valid]
        if len(f0)<8:raise ValueError('insufficient_real_cap_depth_features')
        before_path=rt.save(before,'before-lift');report.update(before_capture=str(before_path),initial_state=state,
            actual_cap_feature_pixels=f0.tolist(),actual_cap_surface_points_odom_m=xyz0.tolist())
        write_json(root/'result.json',report)
        # Current real feedback and unchanged grip authorize one finite arm lift.
        topic=TOPICS[2];pub=rt.publisher(topic);state=rt.state();goal=state['joints'].copy();goal[0]+=.05
        duration=arm_duration(rt.profile,'arm',state['joints'],goal,holding=True)
        record(dict(event='one_deeper_lift_intent',goal=goal,duration_s=duration,initial_real_cap_points=len(f0)))
        pub.publish(rt.trajectory(NAMES[:5],goal[:5],duration));report['lift_command_sent']=True;record(dict(event='one_deeper_lift_sent'))
        deadline=time.monotonic()+duration+.8
        while time.monotonic()<deadline:
            rt.pump(.02);actual=rt.state();rt.ownership(topic)
            if abs(actual['joints'][5]-holding)>.03:raise ValueError('grip_changed_during_probe_lift')
        rt.node.destroy_publisher(pub);pub=None
        if not pose_matches(rt.state()['joints'],goal):raise ValueError('probe_lift_not_reached')
        completed=time.time_ns();palm0=transform_point(before['snapshot'],before['snapshot']['palm_base_at_depth'])
        rows=[];rejected=[];end=time.monotonic()+8
        while len(rows)<3 and time.monotonic()<end:
            sample=rt.sample();snap=sample['snapshot']
            if min(snap['streams'][k]['stamp_ns'] for k in ('rgb','depth'))<=completed:continue
            try:
                g1=cv2.cvtColor(sample['rgb'],cv2.COLOR_RGB2GRAY);p0=f0.astype(np.float32).reshape(-1,1,2)
                p1,ok,_=cv2.calcOpticalFlowPyrLK(gray,g1,p0,None,winSize=(21,21),maxLevel=3)
                back,reverse,_=cv2.calcOpticalFlowPyrLK(g1,gray,p1,None,winSize=(21,21),maxLevel=3)
                tracked=p1[:,0,:];xyz1,depth_ok=points(sample,tracked)
                good=ok[:,0].astype(bool)&reverse[:,0].astype(bool)&depth_ok&(np.linalg.norm(back[:,0,:]-f0,axis=1)<1.5)
                if good.sum()<8:raise ValueError('insufficient_tracked_real_cap_depth')
                deltas=xyz1[good]-xyz0[good];median=np.median(deltas,axis=0)
                inliers=np.linalg.norm(deltas-median,axis=1)<=.015
                if inliers.sum()<8 or inliers.mean()<.6:raise ValueError('cap_depth_motion_not_consistent')
                actual_delta=np.median(deltas[inliers],axis=0)
                palm_delta=transform_point(snap,snap['palm_base_at_depth'])-palm0
                error=float(np.linalg.norm(actual_delta-palm_delta))
                if min(actual_delta[2],palm_delta[2])<.03 or error>.02:raise ValueError('cap_did_not_follow_actual_lift')
                if abs(snap['joints']['hand_motor_joint']-holding)>.03:raise ValueError('current_grip_not_preserved')
                fresh(snap,rt.node.get_clock().now().nanoseconds,root.name)
                stamp=snap['streams']['depth']['stamp_ns']
                if rows and stamp-rows[-1]['stamp_ns']<200_000_000:continue
                path=rt.save(sample,f'after-lift-{len(rows):02d}')
                row=dict(capture=str(path),stamp_ns=stamp,tracked_cap_depth_points=int(inliers.sum()),
                    actual_cap_surface_delta_odom_m=actual_delta.tolist(),actual_palm_delta_odom_m=palm_delta.tolist(),
                    object_surface_palm_follow_error_m=error,median_cap_surface_odom_m=np.median(xyz1[good][inliers],axis=0).tolist(),
                    tracked_feature_pixels=tracked[good][inliers].tolist(),position_kind='actual_visible_cap_surface_not_full_object_center')
                rows.append(row);record(dict(event='current_surface_lift_follow_verified',**row))
            except ValueError as exc:
                rejected.append(str(exc));rt.save(sample,f'rejected-{len(rejected):02d}')
        report.update(lift_rows=rows,rejected_measurements=rejected)
        if len(rows)!=3:raise ValueError('three_current_cap_surface_lift_frames_not_verified:'+str(rejected[-3:]))
        report.update(status='deeper_grasp_real_surface_follows_five_cm_lift_verified',grasp_success_verified=True,
            held_surface_tracking_verified=True,full_object_pose_estimated=False,can_remains_held=True,
            carry_or_placement_verified=False)
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
    finally:
        if pub:rt.node.destroy_publisher(pub)
        try:report['last_fresh_robot_state']=rt.state()
        except Exception as exc:report['state_read_error']=str(exc)
        rt.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if not report['grasp_success_verified']:raise SystemExit(1)
