#!/usr/bin/env python3
"""Replay saved RGB-D frames with their recorded pose landmarks and camera TF.

Runs current hand inference/geometry without publishing robot commands. Missing
raw frames and original TF errors cannot be repaired by this replay. Recorded
pose results isolate downstream changes from VIDEO tracker's sampling history.
"""
import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_matrix

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/cleanup_vision_ros2/scripts'))
import cleanup_detection_node as vision


def replay(directory, phase):
    rows={r['rgb_stamp_ns']:r for r in
          (json.loads(line) for line in (directory/'observations.jsonl').open())}
    rclpy.init()
    logger=Node('cleanup_offline_gesture_replay')
    detector=vision.CleanupDetectionNode.__new__(vision.CleanupDetectionNode)
    detector.get_logger=logger.get_logger
    detector.pose_timestamp_ms=0
    detector.hand_detector=vision.mp_vision.HandLandmarker.create_from_options(
        vision.mp_vision.HandLandmarkerOptions(
            base_options=vision.BaseOptions(model_asset_path=vision._find_hand_model_path()),
            running_mode=vision.VisionTaskRunningMode.IMAGE,num_hands=2,
            min_hand_detection_confidence=.3,min_hand_presence_confidence=.3,
            min_tracking_confidence=.3))
    output=[]
    try:
        for path in sorted(directory.glob(f'phase{phase:02d}-sample*.npz')):
            a=np.load(path);stamp=int(a['rgb_stamp_ns']);row=rows[stamp]
            poses=row.get('gesture_audit',{}).get('poses',[])
            result=SimpleNamespace(pose_landmarks=[],pose_world_landmarks=[])
            for pose in poses:
                result.pose_landmarks.append([SimpleNamespace(x=p[0],y=p[1],z=p[2],visibility=p[3],presence=1.) for p in pose['image']])
                result.pose_world_landmarks.append([SimpleNamespace(x=p[0],y=p[1],z=p[2]) for p in pose['world']] if pose.get('world') else [])
            detector.pose_detector=SimpleNamespace(detect_for_video=lambda *args:result)
            detector.fx,detector.fy,detector.cx_cam,detector.cy_cam=map(float,a['intrinsics'])
            detector.depth_image=a['depth'];detector.depth_scale=float(a['depth_scale'])
            detector.depth_frame_usable=row['depth_usable']
            detector.frame_camera_transform_resolved=True
            detector.frame_camera_transform=None
            matrix=row['camera_to_odom']
            if matrix is not None:
                matrix=np.array(matrix);t=TransformStamped()
                q=quaternion_from_matrix(matrix)
                for i,k in enumerate(('x','y','z')):setattr(t.transform.translation,k,float(matrix[i,3]))
                for i,k in enumerate(('x','y','z','w')):setattr(t.transform.rotation,k,float(q[i]))
                detector.frame_camera_transform=t
            info=detector._estimate_pointing(a['rgb_bgr'],rclpy.time.Time(nanoseconds=stamp))
            output.append(dict(frame=path.name,stamp_ns=stamp,old_valid=row['pointing']['is_valid'],
                               new=info))
    finally:
        detector.hand_detector.close();logger.destroy_node();rclpy.shutdown()
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path);p.add_argument('--phase',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result=replay(args.directory,args.phase)
    args.output.write_text(json.dumps(result,indent=2,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else float(x)))
    from collections import Counter
    print(dict(Counter((r['new'] or {}).get('source','no gesture') for r in result)))
