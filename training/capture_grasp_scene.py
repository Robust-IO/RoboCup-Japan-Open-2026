"""Read-only RGBD and hand-camera capture; no motion or grasp authorization."""
import argparse
import json
import math
from pathlib import Path
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import cv2
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from synced_capture_selection import select_stamps
from capture_processing_budget import ProcessingBudget,result_timing,enforce_final_timing
from latest_capture_index import publish as publish_latest
from capture_file_writer import write_capture_files


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--hand-geometry',action='store_true');p.add_argument('--tracking-epoch')
    p.add_argument('--duration',type=float,default=0,help='Persistent read-only recording, seconds; zero is one shot')
    p.add_argument('--interval',type=float,default=.2)
    p.add_argument('--inference-python');p.add_argument('--registry')
    p.add_argument('--inference-socket',type=Path,help='Owned read-only model service for this transfer')
    p.add_argument('--stop-after-frames',type=int,default=0)
    p.add_argument('--min-frame-span',type=float,default=0.)
    p.add_argument('--inference-imgsz',type=int,choices=[640,960,1280])
    p.add_argument('--confirm-apple-roi',action='store_true',help='Opt-in same-frame near-grasp identity verification only')
    p.add_argument('--near-apple-geometry',action='store_true',help='After confirmed identity, near precontact uses current local RGBD without YOLO')
    p.add_argument('--near-apple-gap-m',type=float,default=.15,help='Remaining forward distance to contact, NOT camera depth')
    p.add_argument('--environment-yaml',type=Path);p.add_argument('--expected-layout');p.add_argument('--target-room')
    p.add_argument('--identity-tracking-task',help='Opt-in read-only identity tracking for this task; never enables motion')
    p.add_argument('--compact-progress',action='store_true',help='Print only frame/stage tracking status; full evidence remains in observation.json')
    p.add_argument('--map-geometry',action='store_true',help='Include image-time map transform without starting detector')
    a=p.parse_args()
    if not math.isfinite(a.duration) or not 0<=a.duration<=600 or not math.isfinite(a.interval) or not .1<=a.interval<=2:raise ValueError('Invalid recording duration or interval')
    if a.duration and not a.tracking_epoch:raise ValueError('Persistent recording requires tracking epoch')
    if (a.stop_after_frames < 0 or not math.isfinite(a.min_frame_span) or a.min_frame_span < 0
            or a.stop_after_frames and not a.duration
            or a.min_frame_span and not a.stop_after_frames):raise ValueError('invalid_capture_burst')
    if a.inference_socket and not a.inference_python:raise ValueError('shared_inference_requires_model_context')
    if bool(a.inference_python)!=bool(a.registry) or a.inference_python and not a.duration:raise ValueError('Live inference requires duration, python and registry')
    if a.confirm_apple_roi and not a.identity_tracking_task:raise ValueError('ROI confirmation requires explicit identity-tracking task')
    if a.near_apple_geometry:
        from near_apple_geometry import check_gap
        check_gap(a.near_apple_gap_m)
        if not a.identity_tracking_task:raise ValueError('Near geometry requires explicit identity-tracking task')
    if a.tracking_epoch and not a.hand_geometry:raise ValueError('Tracking requires hand geometry')
    if a.map_geometry and not a.tracking_epoch:raise ValueError('Map geometry requires tracking epoch')
    if a.output.exists():raise ValueError('Output exists')
    room_environment=None;center_gate=None;environment_sha=None
    identity_observer=None
    if a.identity_tracking_task and not all((a.environment_yaml,a.tracking_epoch,a.duration,a.inference_python)):
        raise ValueError('Identity tracking requires live recording, epoch, and room configuration')
    if any((a.environment_yaml,a.expected_layout,a.target_room)):
        if not all((a.environment_yaml,a.expected_layout,a.target_room,a.inference_python)):raise ValueError('Room check requires explicit environment/layout/room and live inference')
        import yaml,hashlib
        from apple_room_evidence import room_check,CenterConsistency
        data=a.environment_yaml.read_bytes();room_environment=yaml.safe_load(data)
        if room_environment['internal_name']!=a.expected_layout or a.target_room not in room_environment['rooms']:raise ValueError('environment_binding_mismatch')
        environment_sha=hashlib.sha256(data).hexdigest();center_gate=CenterConsistency()
        if a.identity_tracking_task:
            from apple_tracking_observer import TrackingObserver
            identity_observer=TrackingObserver(a.identity_tracking_task,a.tracking_epoch,room_environment,a.target_room,
                near_grasp_roi=a.confirm_apple_roi,near_geometry=a.near_apple_geometry,
                near_geometry_gap_m=a.near_apple_gap_m)
    rclpy.init();node=rclpy.create_node('grasp_scene_readonly');bridge=CvBridge();buf=Buffer();listener=TransformListener(buf,node)
    received={};subs=[]
    topics={'rgb':('/hsrb/head_rgbd_sensor/rgb/image_raw',Image),'depth':('/hsrb/head_rgbd_sensor/depth_registered/image_raw',Image),
      'hand':('/hsrb/hand_camera/image_raw',Image),'rgb_info':('/hsrb/head_rgbd_sensor/rgb/camera_info',CameraInfo),'depth_info':('/hsrb/head_rgbd_sensor/depth_registered/camera_info',CameraInfo)}
    if a.hand_geometry:
        topics.update(hand_info=('/hsrb/hand_camera/camera_info',CameraInfo),joints=('/hsrb/joint_states',JointState))
    stamp=lambda m:Time.from_msg(m.header.stamp)
    history={k:{} for k in ('rgb','depth','hand')}
    def receive(key,msg):
        received[key]=msg
        if key in history:
            history[key][stamp(msg).nanoseconds]=msg
            while len(history[key])>40:history[key].pop(next(iter(history[key])))
    for key,(topic,kind) in topics.items():subs.append(node.create_subscription(kind,topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data))
    end=time.monotonic()+(a.duration or 20);error='missing_streams';captured=None;worker=None
    save_pool=ThreadPoolExecutor(max_workers=3,thread_name_prefix='lossless_capture')
    recorded=[];last_stamps=None;next_save=0;processing_budget=ProcessingBudget()
    if a.duration:a.output.mkdir(parents=True)
    def transforms_ready(stamps):
        rgb_ns,depth_ns,hand_ns=stamps
        rgb=history['rgb'][rgb_ns];dep=history['depth'][depth_ns];hand=history['hand'][hand_ns]
        depth_time=Time(nanoseconds=depth_ns);hand_time=Time(nanoseconds=hand_ns)
        required=[('base_link',rgb.header.frame_id,depth_time),
                  (rgb.header.frame_id,dep.header.frame_id,depth_time)]
        if a.tracking_epoch:
            required.extend([('odom','base_link',depth_time),('base_link','odom',hand_time)])
            if room_environment or a.map_geometry:required.append(('map','base_link',depth_time))
        if a.hand_geometry:
            required.extend((target,hand.header.frame_id,hand_time) for target in ('base_link','hand_palm_link'))
            required.extend((hand.header.frame_id,link,hand_time) for link in
                ('hand_l_spring_proximal_link','hand_r_spring_proximal_link','hand_l_distal_link','hand_r_distal_link'))
        return all(buf.can_transform(target,source,when) for target,source,when in required)
    def save_capture(capture,output):
        msgs,tf,ext,hand_geometry,tracking=capture;output.mkdir(parents=True)
        rgb_pixels=bridge.imgmsg_to_cv2(msgs['rgb'],'bgr8')
        hand_pixels=bridge.imgmsg_to_cv2(msgs['hand'],'bgr8')
        dep=msgs['depth'];depth=bridge.imgmsg_to_cv2(dep,'passthrough').astype(np.float32)
        if dep.encoding=='16UC1':depth*=.001
        elif dep.encoding!='32FC1':raise ValueError('Unsupported depth encoding')
        def transform(t):
            return dict(translation=[t.translation.x,t.translation.y,t.translation.z],quaternion=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
        report=dict(streams={k:dict(stamp_ns=stamp(m).nanoseconds,frame=m.header.frame_id,width=m.width,height=m.height) for k,m in msgs.items() if k!='joints'},
            rgb_k=list(msgs['rgb_info'].k),depth_k=list(msgs['depth_info'].k),rgb_d=list(msgs['rgb_info'].d),depth_d=list(msgs['depth_info'].d),
            base_from_rgb=transform(tf.transform),rgb_from_depth=transform(ext.transform),depth_encoding=dep.encoding,
            valid_depth_pixels=int(np.count_nonzero(np.isfinite(depth)&(depth>0))),motion_authorized=False)
        if a.hand_geometry:
            report.update({k:transform(v) for k,v in hand_geometry.items()})
            report.update(hand_k=list(msgs['hand_info'].k),hand_d=list(msgs['hand_info'].d),hand_distortion_model=msgs['hand_info'].distortion_model,
                          joints=dict(zip(msgs['joints'].name,msgs['joints'].position)),joint_stamp_ns=stamp(msgs['joints']).nanoseconds)
        if a.tracking_epoch:
            report['tracking_epoch']=a.tracking_epoch
            report['tracking_transforms']={key:dict(**transform(t),stamp_ns=ns,source_frame=source,target_frame=target) for key,(t,ns,source,target) in tracking.items()}
        write_capture_files(output,rgb_pixels,hand_pixels,depth,report,save_pool)
        return report
    try:
        if a.inference_python:
            if a.inference_socket:
                from apple_shared_inference import SharedWorker
                worker=SharedWorker(a.inference_socket,a.registry,imgsz=a.inference_imgsz,
                    confirm_apple_roi=a.confirm_apple_roi)
            else:
                from apple_worker_client import Worker
                worker=Worker(a.inference_python,a.registry,a.output/'worker.stderr.log',imgsz=a.inference_imgsz,
                    confirm_apple_roi=a.confirm_apple_roi)
            end=time.monotonic()+a.duration
            print(json.dumps(worker.ready),flush=True)
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if a.duration:
                for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
            if len(received)!=len(topics):
                error='missing_streams: '+','.join(sorted(set(topics)-set(received)));continue
            sample=dict(received)
            budget_decision=None
            if a.duration:
                selection_args=dict(ready=transforms_ready,
                    joint_stamp_ns=stamp(sample['joints']).nanoseconds if a.hand_geometry else None)
                if worker:
                    mode='geometry' if identity_observer and identity_observer.geometry and identity_observer.geometry.active else 'detector'
                    selected,budget_decision=processing_budget.select(history,node.get_clock().now().nanoseconds,
                        last_stamps,mode=mode,monotonic_s=time.monotonic(),**selection_args)
                else:
                    selected=select_stamps(history,node.get_clock().now().nanoseconds,last_stamps,**selection_args)
                if selected is None:
                    error='waiting_for_fresh_joint_image_pair_and_image_time_tf';continue
                sample.update({k:history[k][ns] for k,ns in zip(('rgb','depth','hand'),selected)})
            rgb,dep,hand=[sample[k] for k in ('rgb','depth','hand')]
            stamps=tuple(stamp(m).nanoseconds for m in (rgb,dep,hand))
            if a.duration and (time.monotonic()<next_save or last_stamps is not None and any(x<=y for x,y in zip(stamps,last_stamps))):continue
            if any(not 0<=(node.get_clock().now()-stamp(m)).nanoseconds<=500_000_000 for m in (rgb,dep,hand)):continue
            if abs((stamp(rgb)-stamp(dep)).nanoseconds)>30_000_000:continue
            if a.tracking_epoch and not 0<=(stamp(hand)-stamp(dep)).nanoseconds<=200_000_000:
                error='waiting_for_hand_frame_after_rgbd_anchor';continue
            try:
                tf=buf.lookup_transform('base_link',rgb.header.frame_id,stamp(dep))
                ext=buf.lookup_transform(rgb.header.frame_id,dep.header.frame_id,stamp(dep))
                hand_geometry={}
                tracking={}
                if a.tracking_epoch:
                    for key,target,source,when in [('odom_from_base_at_depth','odom','base_link',stamp(dep)),('base_from_odom_at_hand','base_link','odom',stamp(hand))]:
                        tracked_tf=buf.lookup_transform(target,source,when)
                        tracking[key]=(tracked_tf.transform,when.nanoseconds,source,target)
                    if room_environment or a.map_geometry:
                        mapped=buf.lookup_transform('map','base_link',stamp(dep))
                        tracking['map_from_base_at_depth']=(mapped.transform,stamp(dep).nanoseconds,'base_link','map')
                if a.hand_geometry:
                    if not -20_000_000<=(node.get_clock().now()-stamp(sample['joints'])).nanoseconds<=400_000_000:continue
                    info=received['hand_info']
                    if (info.width,info.height)!=(hand.width,hand.height) or info.header.frame_id!=hand.header.frame_id:
                        raise ValueError('Hand image/CameraInfo mismatch')
                    for target,key in [('base_link','base_from_hand'),('hand_palm_link','palm_from_hand')]:
                        hand_geometry[key]=buf.lookup_transform(target,hand.header.frame_id,stamp(hand)).transform
                    for link in ('hand_l_spring_proximal_link','hand_r_spring_proximal_link','hand_l_distal_link','hand_r_distal_link'):
                        hand_geometry['hand_from_'+link]=buf.lookup_transform(hand.header.frame_id,link,stamp(hand)).transform
                captured=(sample,tf,ext,hand_geometry,tracking)
            except Exception as exc:error=str(exc)
            else:
                if not a.duration:break
                sample_started=time.monotonic()
                selection_now_ns=node.get_clock().now().nanoseconds
                output=a.output/f'{len(recorded):06d}'
                report=save_capture(captured,output)
                save_seconds=time.monotonic()-sample_started
                recorded.append(dict(capture=str(output),stamps_ns=stamps,arm_lift=report['joints']['arm_lift_joint']))
                if worker:
                    def pump_inference():
                        for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
                    sent=time.monotonic();result=None
                    if identity_observer and identity_observer.geometry_ready(report):
                        rgb_array=bridge.imgmsg_to_cv2(sample['rgb'],'rgb8')
                        depth_array=bridge.imgmsg_to_cv2(sample['depth'],'passthrough').astype(np.float32)
                        if sample['depth'].encoding=='16UC1':depth_array*=.001
                        result=identity_observer.observe_geometry(rgb_array,depth_array,report,node.get_clock().now().nanoseconds)
                    geometry_result=result is not None
                    if not geometry_result:result=worker.infer(output,pump=pump_inference)
                    result.setdefault('detector_ran',True)
                    # Both the appearance and depth must be fresh, not depth alone.
                    timing=result_timing(report,node.get_clock().now().nanoseconds,time.monotonic()-sample_started)
                    timing['basis']='oldest_rgb_or_depth_stamp'
                    timing.update(selection_now_ns=selection_now_ns,
                        request_roundtrip_s=time.monotonic()-sent,
                        sampling_budget=budget_decision,
                        oldest_image_age_at_selection_s=(selection_now_ns-min(stamps[:2]))/1e9,
                        joint_age_at_selection_s=(selection_now_ns-report['joint_stamp_ns'])/1e9,
                        image_and_depth_save_s=save_seconds,
                        joint_age_at_result_s=(node.get_clock().now().nanoseconds-report['joint_stamp_ns'])/1e9,
                        original_snapshot_stamps_preserved=True,
                        upstream_stamp_caveat='Images/TF currently bridge-receipt time; joints Unity source time. Source synchronization must be checked independently.')
                    result.update(timing=timing,tracking_epoch=a.tracking_epoch,
                                  eligible_fresh_candidate=result['status']=='candidate_only' and timing['freshness_passed'],
                                  grasp_center_verified=False,room_confirmed=False,motion_authorized=False)
                    if room_environment:
                        result['environment_sha256']=environment_sha
                        if 'candidate' in result:
                            room=room_check(result['candidate'],report,room_environment,a.target_room)
                            result.update(room_evidence=room,room_confirmed=room['room_confirmed'])
                            result['center_evidence']=center_gate.observe(result['candidate'],room,a.tracking_epoch,result['eligible_fresh_candidate'])
                        else:
                            center_gate.history=[]
                        result['eligible_fresh_candidate'] &= result['room_confirmed']
                    if identity_observer:
                        if not geometry_result:
                            result['identity_tracking']=identity_observer.observe(result,report,node.get_clock().now().nanoseconds,capture=output)
                        elif (not timing['freshness_passed'] or not 0<=(node.get_clock().now().nanoseconds-min(stamps[:2]))/1e9<=.4):
                            identity_observer.track.reset();identity_observer.geometry.reset()
                            result['identity_tracking']=dict(status='reobserve',reason='near_geometry_result_expired',
                                identity_confirmed=False,motion_authorized=False,grasp_center_verified=False)
                        result['identity_tracking_task_id']=a.identity_tracking_task
                        recorded[-1]['identity_tracking']=result['identity_tracking']
                    enforce_final_timing(result,result_timing(report,node.get_clock().now().nanoseconds,
                        time.monotonic()-sample_started),identity_observer)
                    if not result['timing']['freshness_passed']:
                        if center_gate:center_gate.history=[]
                    if identity_observer:recorded[-1]['identity_tracking']=result['identity_tracking']
                    (output/'observation.json').write_text(json.dumps(result,indent=2,allow_nan=False))
                    recorded[-1].update(timing=timing,status=result['status'],eligible_fresh_candidate=result['eligible_fresh_candidate'],worker_processing_s=result['worker_processing_s'])
                    if room_environment:recorded[-1].update(room_confirmed=result['room_confirmed'],center_evidence=result.get('center_evidence'))
                    if 'support_evidence' in result:recorded[-1]['support_evidence']=result['support_evidence']
                publish_latest(a.output,output.name,a.tracking_epoch,observation=bool(worker))
                if worker:
                    processing_budget.observe('geometry' if geometry_result else 'detector',time.monotonic()-sample_started)
                last_stamps=stamps;next_save=sample_started+a.interval
                if a.compact_progress:
                    evidence=recorded[-1].get('identity_tracking',{})
                    print(json.dumps(dict(frame=output.name,lift=report.get('joints',{}).get('arm_lift_joint'),
                        tracking=evidence.get('status'),reason=evidence.get('reason'))),flush=True)
                else:print(json.dumps(recorded[-1]),flush=True)
                if a.stop_after_frames:
                    from transfer_efficiency import burst_complete
                    if burst_complete([x['stamps_ns'][1] for x in recorded],a.stop_after_frames,a.min_frame_span):
                        break
        if captured is None:raise RuntimeError(error)
        if not a.duration:print(json.dumps(save_capture(captured,a.output)))
    finally:
        save_pool.shutdown(wait=True)
        if worker:worker.close()
        if a.duration:(a.output/'manifest.json').write_text(json.dumps(dict(captures=recorded,last_wait_reason=error,motion_authorized=False),indent=2))
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
