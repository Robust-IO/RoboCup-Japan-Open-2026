"""Offline replay of recorded estimates, not a replay of original TF messages."""
import argparse
import json
import math
from pathlib import Path
from tf_derived_odometry import OdometryDiagnostics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    source=[json.loads(line) for line in args.input.read_text().splitlines()]
    diagnostics=OdometryDiagnostics()
    rows=[diagnostics.annotate(row) for row in source if row.get('record_type','estimate')=='estimate']
    valid=[row for row in rows if row['valid']]
    summary=dict(input=str(args.input),valid=len(valid),total=len(rows),
                 replay_scope='recorded_estimates_only',navigation_authorized=False)
    summary['velocity_comparison']={key:{kind:dict(
        peak=max(abs(row[field][key]) for row in valid),
        rms=math.sqrt(sum(row[field][key]**2 for row in valid)/len(valid)))
        for kind,field in [('raw','twist'),('filtered','diagnostic_filtered_twist')]}
        for key in ('vx','vy','wz')} if valid else {}
    with args.output.open('x') as stream:
        json.dump(dict(summary=summary,rows=rows),stream,indent=2,allow_nan=False)
    print(json.dumps(summary))


if __name__=='__main__':main()
