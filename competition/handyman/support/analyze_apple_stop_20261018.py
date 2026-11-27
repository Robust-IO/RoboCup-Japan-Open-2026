"""Read-only summary of the stopped trial; preserve every original frame/verdict."""
import json
from pathlib import Path
import tarfile

root = Path('/__HANDYMAN_PACKAGE__/runtime/automatic-apple-20261018-01')
result = json.loads((root/'result.json').read_text())
if result['status'] != 'stopped': raise ValueError('trial_must_be_stopped')
observer = root/'source-observe'
rows = []
for path in sorted(observer.glob('*/observation.json')):
    d = json.loads(path.read_text()); s = json.loads((path.parent/'snapshot.json').read_text())
    identity = d.get('identity_tracking', {})
    roi = d.get('near_grasp_roi_confirmation', {})
    row = dict(frame=path.parent.name, tracking=identity, roi=roi,
        proposals=d.get('tracking_proposals'), timing=d.get('timing'),
        detections=d.get('detections'), joints=s.get('joints'),
        base_transform=s.get('tracking_transforms',{}).get('map_from_base_at_depth'))
    rows.append(row)
report = dict(result=result, rows=rows, robot_commands_sent=False, old_results_modified=False)
output = root.parent/'automatic-apple-20261018-01-stop-audit.json'
with output.open('x') as stream: json.dump(report,stream,indent=2)
archive = root.parent/'automatic-apple-20261018-01-stop-frames.tar.gz'
with tarfile.open(archive,'x:gz') as bundle:
    for name in ('000087','000100','000109','000115','000124'):
        bundle.add(observer/name,arcname=name)
for row in rows[-35:]:
    print(json.dumps(dict(frame=row['frame'],tracking=row['tracking'].get('status'),
        reason=row['tracking'].get('reason'),roi_status=row['roi'].get('status'),roi_reason=row['roi'].get('reason'),
        roi_confidence=row['roi'].get('confidence'),
        candidates=[dict(confidence=p.get('detection',{}).get('confidence'),
            center=p.get('candidate',{}).get('center_m'),
            radius=p.get('candidate',{}).get('radius_m'),rejected=p.get('rejected_reason'),
            support=p.get('support',{}).get('support_consistent')) for p in row['proposals']])),flush=True)
