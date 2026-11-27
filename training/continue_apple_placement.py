"""Explicit reviewed placement-only continuation, NEVER automatic journal resume.

Allowed only when an earlier transfer verified holding and preparation, then
stopped before issuing ANY placement approach/release. New epoch/output/journal;
fresh unchanged held pose, table and clear patch are mandatory before movement.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import uuid

import numpy as np

from apple_transfer_auto_backend import LiveBackend, read
from apple_transfer_auto_plan import clear_patch, depth_in_map, map_pose
from apple_transfer_automation import RobotLock, STAGES, check_inputs, run
from measure_destination_surface import measure
from tabletop_support import support_from_plan


def validate_source(result,journal,plan,preparation,grasp):
    prefix=list(STAGES[:STAGES.index('approach_placement')])
    if (result.get('status')!='stopped' or result.get('stage')!='approach_placement'
            or result.get('confirmed_stages')!=prefix or result.get('placement_verified') is not False):
        raise ValueError('unsupported_stopped_context')
    if any(row.get('event')=='command_intent' and row.get('stage') in ('approach_placement','release','withdraw') for row in journal):
        raise ValueError('previous_placement_command_cannot_be_replayed')
    if (preparation.get('status')!='passed' or preparation.get('inverse_place_stage')!='prepare'
            or grasp.get('grasp_success_verified') is not True or plan.get('task_id')!=result.get('task_id')
            or plan.get('destination_id')!='square_low_table#living_room'):
        raise ValueError('verified_holding_and_preparation_required')
    return True


class PlacementOnlyBackend(LiveBackend):
    placement_continuation_authorized=True

    def __init__(self,config,source,input_audit):
        super().__init__(config,input_audit)
        self.source=Path(source)
        result=read(self.source/'result.json')
        journal=[json.loads(line) for line in (self.source/'journal.jsonl').read_text().splitlines()]
        self.place_plan=read(self.source/'placement-plan.json')
        self.preparation=read(self.source/'place-prepare/result.json')
        validate_source(result,journal,self.place_plan,self.preparation,read(self.source/'grasp/close-lift/visual-lift-verification.json'))
        prior_inputs=read(self.source/'input-files-audit.json')
        if any(prior_inputs[k]!=input_audit[k] for k in ('environment_sha256','semantic_map_sha256','model_sha256')):
            raise ValueError('source_assets_changed')
        self.held=read(self.source/'verified-held-geometry.json')
        self.holding=self.held['holding_opening_rad']
        if abs(self.place_plan['holding_opening_rad']-self.holding)>1e-8:
            raise ValueError('source_holding_mismatch')

    def continuation_preflight(self):
        # A previous edge-unsafe or legacy plan cannot inherit permission to move.
        support = support_from_plan(self.place_plan)
        self.start_observer('placement-observe',False)
        capture,snapshot,_,_=self.latest(20)
        pose=map_pose(snapshot);before=self.preparation['base_anchor']
        yaw_error=abs(math.atan2(math.sin(pose[2]-before[2]),math.cos(pose[2]-before[2])))
        if math.dist(pose[:2],before[:2])>.02 or yaw_error>.02:
            raise ValueError('base_changed_since_stopped_preparation')
        names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        joints=[snapshot['joints'][k] for k in names]
        if any(not math.isfinite(x) or abs(x-y)>.003 for x,y in zip(joints,self.preparation['final_joints'])):
            raise ValueError('prepared_arm_or_holding_changed')
        current_held=self.held_measurement(capture,snapshot)
        if not support.query(self.place_plan['release_xy_map_m'], current_held['radius_m'])['safe']:
            raise ValueError('continuation_requires_edge_safe_replanning')
        depth=np.load(capture/'depth_m.npy',allow_pickle=False)
        table=self.tables[self.config['destination_id']]
        self.plane=measure(depth,snapshot,table)
        xy=self.place_plan['release_xy_map_m']
        height=float(np.r_[xy,1.]@self.plane['plane_map'])
        if abs(height-self.place_plan['table_height_at_release_m'])>.003:
            raise ValueError('table_height_changed_requires_replanning')
        patch=clear_patch(depth_in_map(depth,snapshot),xy,self.plane['plane_map'],.09)
        if patch['clear'] is not True:
            raise ValueError('planned_patch_not_currently_observed_clear')
        self.place_plan.update(task_id=self.task_id,plane_map=self.plane['plane_map'],
                               source_run=str(self.source),reviewed_continuation=True)
        self.place_plan_path=self.output/'placement-plan.json'
        self.evidence('placement-plan',self.place_plan)
        self.evidence('destination-surface',self.plane)
        return self.evidence('continuation-readiness',dict(verified=True,source_run=str(self.source),
            capture=str(capture),base_map_pose=pose,joints=joints,held=current_held,patch=patch,
            previous_commands_replayed=False,grasp_or_preparation_repeated=False))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--source-run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--confirm-layout')
    p.add_argument('--confirm-localization',action='store_true');p.add_argument('--simulation-ignore-collision',action='store_true')
    a=p.parse_args()
    if not a.run or a.confirm_layout!='LayoutA' or not a.confirm_localization or not a.simulation_ignore_collision or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('explicit reviewed simulation continuation required')
    root=Path('/mnt/d/handyman-training-runs')
    if not a.output.resolve().is_relative_to(root) or not a.source_run.resolve().is_relative_to(root):
        p.error('source and new output must be on D drive')
    def interrupted(_signal,_frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    config=read(a.source_run/'config.snapshot.json')
    with RobotLock():
        inputs=check_inputs(config)
        backend=PlacementOnlyBackend(config,a.source_run,inputs)
        result=run(config,'apple-place-reviewed-'+uuid.uuid4().hex,a.output,backend,placement_only=True)
    print(json.dumps(result))
    if result.get('task_objective_completed') is not True:raise SystemExit(1)


if __name__=='__main__':main()
