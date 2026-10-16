"""Read-only replay of recorded head feedback; no ROS or robot commands.

This checks settling eligibility, not a live ready/stop proof. Original receive
times are approximated by the immediately recorded monotonic trace time.
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import head_view_trial as h


def replay(rows,bound):
    command=next(r for r in rows if r['event']=='head_command_sent')
    start=command['monotonic_s'];gate=h.Settling(tuple(command['positions']))
    previous=h.MAX_FUTURE_SKEW_NS;h.MAX_FUTURE_SKEW_NS=bound
    eligible=[];reasons={};count=0
    try:
        for row in rows:
            now=row['monotonic_s']
            if row['event']!='feedback' or not start<=now<=start+8.:continue
            stamp=row['stamp_ns'];sensor_now=stamp+round(row['age_s']*1e9)
            ok,_=gate.sample(row['raw_names'],row['raw_positions'],stamp,sensor_now,now)
            count+=1;reasons[gate.reason]=reasons.get(gate.reason,0)+1
            if ok and now>=start+2.:eligible.append(now-start)
    finally:h.MAX_FUTURE_SKEW_NS=previous
    return dict(future_bound_ms=bound/1e6,samples=count,reasons=reasons,
                first_ready_eligible_s=eligible[0] if eligible else None,
                live_success_proven=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trace',type=Path)
    args=p.parse_args();rows=[json.loads(s) for s in args.trace.read_text().splitlines()]
    print(json.dumps(dict(scope='offline recorded feedback only',source=str(args.trace),
                         old=replay(rows,1_000_000),new=replay(rows,h.MAX_FUTURE_SKEW_NS)),indent=2))
