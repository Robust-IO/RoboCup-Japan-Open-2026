#!/usr/bin/env python3
"""Compare native and production pyramid detections on saved camera frames.

Reports detections and runtime, not precision/recall (frames lack box labels).
No ROS publications and no class/phase-specific inference rules.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/cleanup_vision_ros2/scripts'))
from cleanup_detection_node import CleanupDetectionNode, YOLO, _find_yolo_model_path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
n=CleanupDetectionNode.__new__(CleanupDetectionNode)
n.model=YOLO(_find_yolo_model_path());n.detection_scales=(640,320,160)
rows=[]
for f in sorted(a.directory.glob('phase*-sample*.npz')):
    data=np.load(f);image=data['rgb_bgr']
    native=n.model.predict(image,imgsz=640,conf=.25,iou=.7,verbose=False)[0].boxes
    merged=n._detect_object_boxes(image)
    def describe(boxes):
        return [dict(name=n.model.names[int(b.cls[0])],confidence=float(b.conf[0]),
                     xyxy=b.xyxy[0].tolist()) for b in boxes]
    rows.append(dict(frame=f.name,native=describe(native),pyramid=describe(merged),
                     inference_sec=n.last_object_inference_sec))
a.output.write_text(json.dumps(rows,indent=2))
print('frames',len(rows),'native boxes',sum(len(r['native']) for r in rows),
      'pyramid boxes',sum(len(r['pyramid']) for r in rows),
      'median seconds',float(np.median([r['inference_sec'] for r in rows])))
