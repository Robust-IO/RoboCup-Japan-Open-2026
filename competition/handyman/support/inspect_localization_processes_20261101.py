import json
from pathlib import Path
rows=[]
for d in Path('/proc').iterdir():
 if not d.name.isdigit():continue
 try:
  raw=(d/'cmdline').read_bytes();parts=[p.decode(errors='replace') for p in raw.split(b'\0') if p];cmd=' '.join(parts)
  if parts and any(t in cmd for t in ('publish_reviewed_map_anchor','initial_pose_localizer','static_transform_publisher','amcl','map_anchor','handyman_navigation')):
   rows.append(dict(pid=int(d.name),args=parts))
 except (PermissionError,FileNotFoundError,ProcessLookupError):pass
print(json.dumps(rows,indent=2))
