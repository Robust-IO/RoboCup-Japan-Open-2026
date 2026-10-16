"""Real diagnostic replay into search logic; arrivals and schedule time are simulated.

No ROS, navigation, inference rerun or competition messages. Original sensor
stamps are preserved. Freshness is evaluated at the recorded processing time,
not against today's clock. Never authorizes room absence.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_scan import SearchScan
from search_vision_adapter import SearchVisionAdapter


def load(path):
    data=path.read_bytes()
    rows=[json.loads(line) for line in data.splitlines()]
    rows=[row for row in rows if 'rgb_stamp_ns' in row]
    if not rows: raise ValueError('No timestamped diagnostics: '+str(path))
    return rows,hashlib.sha256(data).hexdigest()


def replay_view(scan,rows,now):
    if scan.phase!='navigate': raise ValueError('Expected a navigation request')
    view_id=scan.request()['view_id']
    scan.arrived(view_id,True,True,now) # Explicitly simulated, NOT a real arrival.
    first=min(rows[0]['rgb_stamp_ns'],rows[0]['depth_stamp_ns'])
    adapter=SearchVisionAdapter(scan,view_id,first-1)
    consumed=[]
    for row in rows:
        stamp=min(row['rgb_stamp_ns'],row['depth_stamp_ns'])
        age=float(row['age_s'])
        if not math.isfinite(age) or age<0: raise ValueError('Invalid recorded age')
        relative=(stamp-first)/1e9+age
        if relative>=scan.view_s: break
        result=adapter.consume(row,now+relative,sensor_now_ns=stamp+round(age*1e9))
        consumed.append(dict(sample=row.get('sample'),state=result['state'],
                             reason=result.get('adapter_reason')))
        if scan.phase!='observe': break
    if scan.phase=='observe':scan.tick(now+scan.view_s+.001)
    return dict(events=consumed,status=scan.status(),position=scan.position)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('present','removed','repositioned','output'):
        p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('Output exists; refusing overwrite')
    inputs={name:load(getattr(args,name)) for name in ('present','removed','repositioned')}
    points=[dict(x=0,y=0,yaw=0),dict(x=1,y=0,yaw=0)] # Dummy plan, not map coordinates for execution.
    cases={}
    for name in inputs:
        scan=SearchScan('recorded-'+name,'canned_juice',points[:1],0,timeout_s=60,view_s=5)
        cases[name]=replay_view(scan,inputs[name][0],0)
    assert cases['present']['status']['state']=='found', cases['present']
    assert cases['repositioned']['status']['state']=='found', cases['repositioned']
    assert cases['removed']['status']['state']=='incomplete', cases['removed']
    assert cases['removed']['position'] is None
    distance=math.dist(cases['present']['position'],cases['repositioned']['position'])
    assert distance>.05, distance
    scan=SearchScan('recorded-two-view','canned_juice',points,0,timeout_s=60,view_s=5)
    first=replay_view(scan,inputs['removed'][0],0)
    assert first['status']['state']=='navigate' and scan.index==1
    second=replay_view(scan,inputs['repositioned'][0],6)
    assert second['status']['state']=='found'
    assert all(not case['status']['does_not_exist_authorized'] and not case['status']['actionable']
               for case in list(cases.values())+[first,second])
    report=dict(passed=True,simulated_arrival=True,simulated_schedule=True,
                freshness_reference='recorded_processing_time',robot_control_started=False,
                does_not_exist_authorized=False,position_frame='odom',
                position_semantics='visible_surface_not_grasp_pose',
                source_hashes={name:dict(path=str(getattr(args,name)),sha256=value[1]) for name,value in inputs.items()},
                cases=cases,two_view_sequence=[first,second],position_change_m=distance)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    print(json.dumps(dict(passed=True,case_states={name:value['status']['state'] for name,value in cases.items()},
                         two_view_states=[first['status']['state'],second['status']['state']],
                         simulated_arrival=True,position_change_m=distance,output=str(args.output))))


if __name__=='__main__':main()
