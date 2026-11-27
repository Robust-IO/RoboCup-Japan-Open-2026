"""Read-only RGBD and hand-camera capture; no motion or grasp authorization."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import cv2
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--hand-geometry',action='store_true');p.add_argument('--tracking-epoch');a=p.parse_args()
    if a.tracking_epoch and not a.hand_geometry:raise ValueError('Tracking requires hand geometry')
    if a.output.exists():raise ValueError('Output exists')
    rclpy.init();node=rclpy.create_node('grasp_scene_readonly');bridge=CvBridge();buf=Buffer();listener=TransformListener(buf,node)
    received={};subs=[]
    topics={'rgb':('/hsrb/head_rgbd_sensor/rgb/image_raw',Image),'depth':('/hsrb/head_rgbd_sensor/depth_registered/image_raw',Image),
      'hand':('/hsrb/hand_camera/image_raw',Image),'rgb_info':('/hsrb/head_rgbd_sensor/rgb/camera_info',CameraInfo),'depth_info':('/hsrb/head_rgbd_sensor/depth_registered/camera_info',CameraInfo)}
    if a.hand_geometry:
        topics.update(hand_info=('/hsrb/hand_camera/camera_info',CameraInfo),joints=('/hsrb/joint_states',JointState))
    for key,(topic,kind) in topics.items():subs.append(node.create_subscription(kind,topic,lambda m,k=key:received.__setitem__(k,m),qos_profile_sensor_data))
    stamp=lambda m:Time.from_msg(m.header.stamp)
    end=time.monotonic()+20;error='missing_streams';captured=None
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if len(received)!=len(topics):
                error='missing_streams: '+','.join(sorted(set(topics)-set(received)));continue
            rgb,dep,hand=[received[k] for k in ('rgb','depth','hand')]
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
                if a.hand_geometry:
                    if not 0<=(node.get_clock().now()-stamp(received['joints'])).nanoseconds<=500_000_000:continue
                    info=received['hand_info']
                    if (info.width,info.height)!=(hand.width,hand.height) or info.header.frame_id!=hand.header.frame_id:
                        raise ValueError('Hand image/CameraInfo mismatch')
                    for target,key in [('base_link','base_from_hand'),('hand_palm_link','palm_from_hand')]:
                        hand_geometry[key]=buf.lookup_transform(target,hand.header.frame_id,stamp(hand)).transform
                    for link in ('hand_l_spring_proximal_link','hand_r_spring_proximal_link','hand_l_distal_link','hand_r_distal_link'):
                        hand_geometry['hand_from_'+link]=buf.lookup_transform(hand.header.frame_id,link,stamp(hand)).transform
                captured=(dict(received),tf,ext,hand_geometry,tracking);break
            except Exception as exc:error=str(exc)
        if captured is None:raise RuntimeError(error)
        msgs,tf,ext,hand_geometry,tracking=captured;a.output.mkdir(parents=True)
        for key in ('rgb','hand'):
            if not cv2.imwrite(str(a.output/(key+'.png')),bridge.imgmsg_to_cv2(msgs[key],'bgr8')):raise RuntimeError('Image write failed')
        dep=msgs['depth'];depth=bridge.imgmsg_to_cv2(dep,'passthrough').astype(np.float32)
        if dep.encoding=='16UC1':depth*=.001
        elif dep.encoding!='32FC1':raise ValueError('Unsupported depth encoding')
        np.save(a.output/'depth_m.npy',depth)
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
        (a.output/'snapshot.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
