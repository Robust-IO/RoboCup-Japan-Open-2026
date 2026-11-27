"""Read-only latency decomposition of the completed failed source validation."""
import json
from pathlib import Path
import statistics

root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/apple-capture-budget-r13-source-readonly-20261018-01')
rows=[]
for path in sorted((root/'source-observe').glob('*/observation.json')):
    obs=json.loads(path.read_text());t=obs.get('timing',{})
    rows.append(dict(frame=path.parent.name,status=obs['status'],tracking=obs.get('identity_tracking',{}).get('status'),
        reason=obs.get('identity_tracking',{}).get('reason'),timing=t,
        worker_processing_s=obs.get('worker_processing_s'),
        apples=[d for d in obs.get('detections',[]) if d['name']=='apple']))
keys=['oldest_image_age_at_selection_s','joint_age_at_selection_s','image_and_depth_save_s',
      'request_roundtrip_s','processing_including_save_s','capture_to_result_age_s','joint_age_at_result_s']
summary={k:dict(minimum=min(r['timing'][k] for r in rows),median=statistics.median(r['timing'][k] for r in rows),
    maximum=max(r['timing'][k] for r in rows)) for k in keys} if rows else {}
worker=[r['worker_processing_s'] for r in rows]
summary['worker_processing_s']=dict(minimum=min(worker),median=statistics.median(worker),maximum=max(worker)) if worker else None
report=dict(frames=len(rows),fresh_frames=sum(r['timing'].get('freshness_passed') is True for r in rows),
    tracked_frames=sum(r['tracking']=='identity_tracked' for r in rows),summary=summary,
    first=rows[0] if rows else None,last=rows[-1] if rows else None,rows=rows,robot_commands_sent=False)
with (root/'latency-diagnosis.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
