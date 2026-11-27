"""Deterministic simulated event replay; no ROS, navigation, or camera access."""
import argparse
import json
from pathlib import Path
from search_policy import room_scan

def rehearse(plans):
    rows=[]
    for plan in plans:
        for room in plan['rooms']:
            name=room['room']
            for scenario in ['found','all_negative','navigation_failure','camera_failure','timeout','cancel']:
                scan,reason=room_scan(plan,name,f"{plan['layout']}/{name}/{scenario}",'canned_juice')
                row=dict(layout=plan['layout'],room=name,scenario=scenario,policy=reason,requests=[])
                if scan is None:
                    row.update(state='blocked',actionable=False,does_not_exist_authorized=False)
                    rows.append(row); continue
                now=0.; stamp=0
                while scan.phase in ('navigate','observe'):
                    req=scan.request(); row['requests'].append(req)
                    now+=.1
                    if scenario=='timeout': scan.tick(61); break
                    if scenario=='cancel': scan.cancel(); break
                    scan.arrived(req['view_id'],scenario!='navigation_failure',True,now)
                    if scenario=='navigation_failure': continue
                    if scenario=='camera_failure':
                        scan.tick(now+6); now+=6; continue
                    if scenario=='found':
                        stamp+=1; scan.observe(req['view_id'],'canned_juice',stamp,now,True,True,[1,2,3]); break
                    for _ in range(3):
                        stamp+=1; now+=.1
                        scan.observe(req['view_id'],'canned_juice',stamp,now,True,absent=True)
                row.update(scan.status()); rows.append(row)
    return dict(simulated=True,actionable=False,results=rows)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); report=rehearse(json.loads(a.plan.read_text())['plans'])
    with a.output.open('x') as f: json.dump(report,f,indent=2,allow_nan=False)
    from collections import Counter
    print(dict(Counter(r['state'] for r in report['results'])))
    print('Excluded-room request count:',sum(len(r['requests']) for r in report['results'] if r['state']=='blocked'))
    print('SIMULATION ONLY:',a.output)

if __name__=='__main__': main()
