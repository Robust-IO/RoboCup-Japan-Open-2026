# models/

Place your trained YOLO weights here as `last.pt`.

The node (`object_detection_node.py`) will:
1. First look for `models/last.pt` (your custom-trained weights).
2. If not found, use `models/yolo12n.pt` if it is present locally.
3. If neither file exists, it will try to download `yolo12n.pt`
   from the ultralytics hub.

This repository manages Python packages with `pixi`, but model assets remain
manual files. Keep the desired YOLO weights under `models/` so the node can
run without network access.
