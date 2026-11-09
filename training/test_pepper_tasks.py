import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest
import numpy as np
from pepper_tasks import load_task,configured_transfer,validate_task,validate_predecessor_task,DEFAULT_TASK,REVERSE_TASK
from pepper_place import extension_clearance

BASE=json.loads((Path(__file__).parent/'config/apple_transfer_auto_v1.json').read_text())


def inputs():
    env=dict(internal_name='LayoutA',rooms=dict(
        bedroom=dict(region=[[5,-10],[13,-10],[13,-1],[5,-1]]),
        living_room=dict(region=[[0,0],[6,0],[6,6],[0,6]])),routes=[
            dict(**{'from':a,'to':b},waypoints=[dict(x=5,y=0,yaw=0)])
            for a,b in [('bedroom','living_room'),('living_room','bedroom')]])
    tables=[dict(id='white_side_table#bedroom_1',room='bedroom'),dict(id='square_low_table#living_room',room='living_room')]
    return env,tables


class ConfiguredTaskTests(unittest.TestCase):
    def test_forward_preserves_all_existing_motion_policy_and_poses(self):
        env,tables=inputs();original=copy.deepcopy(BASE)
        cfg=configured_transfer(BASE,load_task(),env,tables)
        for field in ('motion_profile','navigation_tolerances','policy','execution','source_staging_goal',
                'destination_staging_goal','source_table_id','destination_id'):
            self.assertEqual(cfg[field],BASE[field])
        self.assertEqual(BASE,original);self.assertEqual(cfg['task']['object'],'ground_pepper')
    def test_reverse_swaps_furniture_rooms_and_observation_poses(self):
        env,tables=inputs();cfg=configured_transfer(BASE,load_task(REVERSE_TASK),env,tables)
        self.assertEqual(cfg['source_room'],'living_room');self.assertEqual(cfg['destination_room'],'bedroom')
        self.assertEqual(cfg['source_staging_goal'],BASE['destination_staging_goal'])
        self.assertEqual(cfg['destination_staging_goal'],[9.70,-6.61,0.])
        self.assertEqual(cfg['source_table_id'],BASE['destination_id'])
        self.assertEqual(cfg['destination_id'],BASE['source_table_id'])
    def test_furniture_ownership_observation_room_and_route_are_checked(self):
        env,tables=inputs();task=load_task(REVERSE_TASK)
        for kind in ('table_room','pose','route'):
            e,t,job=copy.deepcopy(env),copy.deepcopy(tables),copy.deepcopy(task)
            if kind=='table_room':t[1]['room']='bedroom'
            if kind=='pose':job['source']['observation_pose']=[9.9,-6.6,0.]
            if kind=='route':e['routes']=[]
            with self.subTest(kind=kind),self.assertRaises(ValueError):configured_transfer(BASE,job,e,t)
    def test_task_cannot_override_motion_or_supply_object_coordinates(self):
        for name,value in (('policy',{}),('object_position',[1,2,3]),('motion_profile','anything')):
            task=load_task();task[name]=value
            with self.assertRaises(ValueError):validate_task(task)
        for pose in ([float('nan'),0,0],[1,2],[True,2,3]):
            task=load_task();task['source']['observation_pose']=pose
            with self.assertRaises(ValueError):validate_task(task)
    def test_forward_legacy_continuation_still_matches_but_reverse_does_not(self):
        env,tables=inputs();forward=configured_transfer(BASE,load_task(),env,tables)
        validate_predecessor_task(forward,BASE)
        reverse=configured_transfer(BASE,load_task(REVERSE_TASK),env,tables)
        with self.assertRaisesRegex(ValueError,'continuation_task_changed'):validate_predecessor_task(reverse,BASE)
    def test_default_and_reverse_cli_are_readonly(self):
        for path in (DEFAULT_TASK,REVERSE_TASK):
            proc=subprocess.run([sys.executable,str(Path(__file__).with_name('run_object_transfer.py')),
                '--object','ground_pepper','--task-config',str(path)],capture_output=True,text=True,timeout=15)
            self.assertEqual(proc.returncode,0,proc.stderr)
            result=json.loads(proc.stdout);self.assertFalse(result['robot_commands_sent'])
            self.assertEqual(result['configured_task'],load_task(path))


class ExtensionDirectionTests(unittest.TestCase):
    def check(self,yaw,center,half,predicted):
        snapshot=dict(tracking_transforms=dict(map_from_base_at_depth=dict(quaternion=[0,0,math.sin(yaw/2),math.cos(yaw/2)])))
        plane=dict(observed_xy_min=(np.asarray(center)-half).tolist(),observed_xy_max=(np.asarray(center)+half).tolist())
        return extension_clearance(predicted,snapshot,plane,dict(footprint_center=center,footprint_half_size=half,radius_m=.0316))
    def test_forward_heading_retains_original_map_y_clearance(self):
        row=self.check(math.pi/2,[2.,3.5],[.5,.4],[2.,2.8,.5])
        self.assertTrue(row['ready']);self.assertAlmostEqual(row['front_along_heading_m'],3.1)
        self.assertFalse(self.check(math.pi/2,[2.,3.5],[.5,.4],[2.,3.05,.5])['ready'])
    def test_reverse_heading_uses_map_x_rather_than_y(self):
        good=self.check(0.,[10.8,-6.6],[.4,.7],[10.1,-6.6,.5])
        self.assertTrue(good['ready']);self.assertAlmostEqual(good['front_along_heading_m'],10.4)
        self.assertFalse(self.check(0.,[10.8,-6.6],[.4,.7],[10.35,-6.6,.5])['ready'])
    def test_rotating_scene_and_heading_keeps_clearance(self):
        a=self.check(0.,[2.,.3],[.4,.7],[1.2,.3,.5])
        b=self.check(math.pi/2,[-.3,2.],[.7,.4],[-.3,1.2,.5])
        self.assertAlmostEqual(a['clearance_m'],b['clearance_m']);self.assertEqual(a['ready'],b['ready'])

if __name__=='__main__':unittest.main()
