"""Replay saved diagnostics with simulated arrival; not a live search test."""
import argparse,json
from pathlib import Path
from search_scan import SearchScan
from search_vision_adapter import SearchVisionAdapter

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--log',type=Path,required=True);a=p.parse_args()
    rows=[json.loads(l) for l in a.log.read_text().splitlines()]
    first=next(r for r in rows if 'rgb_stamp_ns' in r)
    start=min(first['rgb_stamp_ns'],first['depth_stamp_ns'])-1
    scan=SearchScan('replay','canned_juice',[dict(x=0,y=0,yaw=0)],0,view_s=30)
    scan.arrived('replay:0',True,True,0); adapter=SearchVisionAdapter(scan,'replay:0',start)
    for row in rows:
        if 'rgb_stamp_ns' not in row: continue
        sensor_now=min(row['rgb_stamp_ns'],row['depth_stamp_ns'])+int(row.get('age_s',0)*1e9)
        result=adapter.consume(row,(sensor_now-start)/1e9,sensor_now_ns=sensor_now)
        if scan.phase!='observe':
            print(json.dumps(dict(simulated_arrival=True,source_sample=row.get('sample'),result=result)));return
    print(json.dumps(dict(simulated_arrival=True,result=scan.status())))

if __name__=='__main__': main()
