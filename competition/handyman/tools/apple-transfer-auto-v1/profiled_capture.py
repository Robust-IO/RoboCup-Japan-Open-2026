"""Demand-driven acquisition on the existing runtime subscriptions; no control."""
import time
import numpy as np
from observation_profiles import channels,anchor_key,select_profile_stamps

def transform_requests(runtime,profile,selected):
    key=anchor_key(profile);ns=selected[key];requests=[]
    if 'rgb' in selected:
        rgb=runtime.history['rgb'][selected['rgb']];depth=runtime.history['depth'][selected['depth']]
        requests += [('base_from_rgb','base_link',rgb.header.frame_id,ns),('rgb_from_depth',rgb.header.frame_id,depth.header.frame_id,ns)]
    if 'hand' in selected:
        hand=runtime.history['hand'][selected['hand']];h=selected['hand']
        requests += [('base_from_hand','base_link',hand.header.frame_id,h),('palm_from_hand','hand_palm_link',hand.header.frame_id,h)]
        requests.append(('base_from_odom_at_hand','base_link','odom',h))
    requests += [(f'odom_from_base_at_{key}','odom','base_link',ns),(f'map_from_base_at_{key}','map','base_link',ns)]
    if profile!='state':requests.append((f'base_from_palm_at_{key}','base_link','hand_palm_link',ns))
    return requests

def acquire(runtime,profile,*,timeout=1.,processing_reserve_ns=0,time_factory=None,capture_after_ns=0):
    if time_factory is None:
        from rclpy.time import Time
        time_factory=lambda ns:Time()  # Current transforms while the robot is held stationary.
    required=channels(profile);infos={k:k+'_info' for k in required};deadline=time.monotonic()+timeout
    last=getattr(runtime,'_profile_last_stamps',{})
    while time.monotonic()<deadline:
        runtime.pump(.01)
        if 'joints' not in runtime.received or not all(v in runtime.received for v in infos.values()):continue
        joint=runtime.received['joints'];now=runtime.node.get_clock().now().nanoseconds
        def ready(selected):
            return all(runtime.buf.can_transform(t,s,time_factory(ns)) for _,t,s,ns in transform_requests(runtime,profile,selected))
        selected=select_profile_stamps(runtime.history,now,profile,last=last,ready=ready,joint_stamp_ns=runtime.stamp(joint),processing_reserve_ns=0,capture_after_ns=capture_after_ns)
        if selected is None:continue
        messages={k:runtime.history[k][selected[k]] for k in required}
        requests=transform_requests(runtime,profile,selected)
        snapshot=dict(observation_profile=profile,tracking_epoch=runtime.epoch,transform_basis='stationary_transform_snapshot',
            streams={k:dict(stamp_ns=selected[k],frame=m.header.frame_id,width=m.width,height=m.height) for k,m in messages.items()},
            joints=dict(zip(joint.name,joint.position)),joint_stamp_ns=runtime.stamp(joint),tracking_transforms={},motion_authorized=False)
        for name,t,s,ns in requests:
            tf=runtime.buf.lookup_transform(t,s,time_factory(ns));value=runtime.tf_dict(tf.transform)
            # stamp_ns associates this frozen transform with the image. Preserve
            # the actual TF stamp separately; no historical TF lookup is claimed.
            actual=runtime.stamp(tf) if hasattr(tf,'header') else None
            if name.startswith(('odom_from_base','map_from_base','base_from_odom')):snapshot['tracking_transforms'][name]=dict(value,stamp_ns=ns,actual_transform_stamp_ns=actual,basis='stationary_snapshot',source_frame=s,target_frame=t)
            else:snapshot[name]=value
        sample=dict(snapshot=snapshot,rgb=None,depth=None,hand=None)
        if 'rgb' in required:
            for k in ('rgb','depth'):
                info=runtime.received[infos[k]];snapshot[k+'_k']=list(info.k);snapshot[k+'_d']=list(info.d)
            snapshot['depth_encoding']=messages['depth'].encoding
            sample['rgb']=runtime.bridge.imgmsg_to_cv2(messages['rgb'],'rgb8')
            depth=runtime.bridge.imgmsg_to_cv2(messages['depth'],'passthrough').astype(np.float32)
            if snapshot['depth_encoding']=='16UC1':depth*=.001
            elif snapshot['depth_encoding']!='32FC1':raise ValueError('unsupported_can_depth_encoding')
            sample['depth']=depth
            snapshot['palm_base_at_depth']=snapshot['base_from_palm_at_depth']['translation']
        if 'hand' in required:
            info=runtime.received[infos['hand']];snapshot['hand_k']=list(info.k);snapshot['hand_d']=list(info.d)
            sample['hand']=runtime.bridge.imgmsg_to_cv2(messages['hand'],'bgr8')
        # The caller refreshes actual joint/base feedback after decoding. A
        # stationary capture does not expire because decoding took time.
        runtime._profile_last_stamps=dict(last,**selected)
        return sample
    raise RuntimeError('observation_slice_pending:'+profile)
