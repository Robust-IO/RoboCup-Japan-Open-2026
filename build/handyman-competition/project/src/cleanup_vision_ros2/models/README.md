# models/

Place Interactive Cleanup model assets here.

YOLO object detection searches in order:

1. `cleanup_model.pt` — custom-trained on competition objects
2. `last.pt` — any custom training output
3. `yolo12n.pt` — COCO pretrained
4. `src/vision_ros2/models/yolo12n.pt` — local fallback from the Handyman package

COCO pretrained can detect ~80 common classes (person, bottle, cup,
dining table, chair, etc.) which covers many competition scenarios.

Avatar pointing uses MediaPipe Tasks and expects one of these files:

1. `pose_landmarker.task` — recommended fixed filename
2. `pose_landmarker_full.task`
3. `pose_landmarker_lite.task`
4. `pose_landmarker_heavy.task`

No pose model is downloaded automatically. In offline competition setups,
place the `.task` bundle here before launching the node.
