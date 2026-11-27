"""Exercise the REAL source-observation gate, with all motion calls disabled.

Uses fresh task/epoch and a new recording, never an old journal or retimestamped
image. Passing means observation handoff works while idle, NOT grasp eligibility
or grasp success. Production grasp-planning rejections are reported separately;
they are never bypassed or used to authorize movement.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from apple_transfer_auto_backend import LiveBackend, read
from apple_transfer_auto_plan import fresh_snapshot, map_pose, plan_grasp
from apple_transfer_automation import check_inputs, write_json


def assess_grasp_plan(observation, snapshot, table, evidence, config):
    """Report the unmodified production gate without hiding timing failures."""
    try:
        plan=plan_grasp(observation,snapshot,table,evidence,config)
    except ValueError as exc:
        return dict(grasp_plan_eligible=False,grasp_plan_rejection=str(exc))
    return dict(grasp_plan_eligible=True,grasp_plan_rejection=None,
                measured_grasp_palm_height_m=plan['palm_height_m'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--samples',type=int,default=15)
    parser.add_argument('--config',type=Path,default=Path(__file__).parent/'config/apple_transfer_auto_v1.json')
    args=parser.parse_args()
    if not 5<=args.samples<=30:parser.error('samples must be5..30')
    if (os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or not args.output.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))):
        parser.error('explicit domain71 localhost environment and NEW D-drive output required')
    args.output.mkdir(parents=True,exist_ok=False)
    config=read(args.config);task='source-timing-'+uuid.uuid4().hex
    backend=None;rows=[]
    result=dict(status='stopped',robot_commands_sent=False,grasp_commands_sent=False,
                protocol_task_finished_sent=False,whole_transfer_verified=False,
                scope='Current source timing/identity handoff while idle ONLY; NOT grasp eligibility')
    log=(args.output/'journal.jsonl').open('x')
    def record(value):
        log.write(json.dumps(dict(time_ns=time.time_ns(),**value))+'\n');log.flush()
    def forbidden(*_args,**_kwargs):
        raise RuntimeError('motion_not_allowed_in_readonly_timing_validation')
    def readiness(name):
        output=args.output/(name+'.json')
        process=subprocess.run([sys.executable,str(Path(__file__).parent/'inspect_transfer_ready.py'),
            '--output',str(output)],capture_output=True,text=True,timeout=20)
        value=read(output)
        if process.returncode or value.get('ready') is not True:
            raise ValueError(name+':'+str(value.get('reason','readiness_failed')))
        return value
    try:
        inputs=check_inputs(config);write_json(args.output/'input-files-audit.json',inputs)
        before=readiness('before')
        backend=LiveBackend(config,inputs);backend.bind(args.output,task,record)
        backend.stage='readonly_source_timing'
        backend.command=backend.arm=backend.adjust=backend.navigate=forbidden
        backend.start_observer('source-observe',True)
        seen=set();end=time.monotonic()+120
        while len(rows)<args.samples:
            if time.monotonic()>=end:raise RuntimeError('distinct_fresh_observation_budget_expired')
            capture,snapshot,observation,evidence=backend.latest(100 if not rows else 5)
            if str(capture) in seen:
                time.sleep(.02);continue
            geometry=assess_grasp_plan(observation,snapshot,backend.tables[config['source_table_id']],evidence,config)
            now=time.time_ns();fresh_snapshot(snapshot,now,backend.epoch)
            pose=map_pose(snapshot)
            if (math.dist(pose[:2],before['base_map_pose'][:2])>.01
                    or abs(math.atan2(math.sin(pose[2]-before['base_map_pose'][2]),
                                      math.cos(pose[2]-before['base_map_pose'][2])))>.02):
                raise ValueError('robot_moved_during_readonly_validation')
            row=dict(capture=str(capture),depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
                joint_stamp_ns=snapshot['joint_stamp_ns'],checked_ns=now,
                oldest_image_age_s=(now-min(snapshot['streams'][k]['stamp_ns'] for k in ('rgb','depth')))/1e9,
                joint_age_s=(now-snapshot['joint_stamp_ns'])/1e9,
                rgb_depth_delta_s=abs(snapshot['streams']['rgb']['stamp_ns']-snapshot['streams']['depth']['stamp_ns'])/1e9,
                identity_confirmed=observation['identity_tracking']['identity_confirmed'],
                pipeline_timing=observation['timing'],**geometry)
            rows.append(row);seen.add(str(capture))
            print(json.dumps(dict(sample=len(rows),capture=capture.name,image_age_s=row['oldest_image_age_s'],
                                  joint_age_s=row['joint_age_s'],
                                  grasp_plan_rejection=row['grasp_plan_rejection'])),flush=True)
        backend.close();backend=None
        after=readiness('after')
        if math.dist(before['base_map_pose'][:2],after['base_map_pose'][:2])>.01:
            raise ValueError('readiness_pose_changed')
        result.update(status='source_timing_readonly_passed',samples=len(rows),
            min_joint_age_s=min(x['joint_age_s'] for x in rows),
            max_joint_age_s=max(x['joint_age_s'] for x in rows),
            max_image_age_s=max(x['oldest_image_age_s'] for x in rows),
            max_rgb_depth_delta_s=max(x['rgb_depth_delta_s'] for x in rows),
            pose_before=before['base_map_pose'],pose_after=after['base_map_pose'],
            publishers_before=before['command_publishers'],publishers_after=after['command_publishers'],
            all_grasp_plans_eligible=all(x['grasp_plan_eligible'] for x in rows),
            grasp_plan_rejections=sorted({x['grasp_plan_rejection'] for x in rows if x['grasp_plan_rejection']}),
            original_stamps_preserved=True,freshness_thresholds_changed=False)
    except (Exception,KeyboardInterrupt) as exc:
        result['reason']=str(exc) or 'interrupted'
    finally:
        if backend is not None:backend.close()
        log.close();result['rows']=rows
        write_json(args.output/'result.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
    if result['status']!='source_timing_readonly_passed':raise SystemExit(1)


if __name__=='__main__':main()
