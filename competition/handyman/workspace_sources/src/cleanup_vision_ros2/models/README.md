# models/

Place Interactive Cleanup model assets here.

YOLO object detection searches in order:

1. `cleanup_model.pt` — bundled 13-class Interactive Cleanup model
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

The bundled YOLO model contains these 13 classes:

`bear_doll`, `canned_juice`, `dog_doll`, `filled_ketchup`,
`filled_plastic_bottle`, `hourglass`, `rabbit_doll`, `sauce`, `soysauce`,
`spray_bottle`, `sugar`, `toy_penguin`, and `tumbler`.

SHA-256 of `cleanup_model.pt`:
`e6c92ef0f3692c6260a3f97afbe7a6c72917db5c4742b41cf26baae824ada9a5`.
