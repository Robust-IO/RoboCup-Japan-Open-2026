import unittest
import numpy as np
from destination_observer import DestinationObserver
class Runtime:
    def __init__(self):self.stamp=10;self.saved=[]
    def sample(self,**kw):return {'depth':'synthetic-depth','snapshot':{'streams':{'depth':{'stamp_ns':self.stamp}}}}
    def palm_pose(self,s):return {'translation':[1,2,3],'quaternion':[0,0,0,1]}
    def fresh_measurement(self,s):pass
    def save(self,item,label,volume):self.saved.append(volume);return '/synthetic/'+label
    def state(self):return {'joints':[0,0,0,0,0,.32]}
class ObserverTests(unittest.TestCase):
    def setup_observer(self):
        rt=Runtime();self.calls=[]
        context=dict(schema='handyman_held_surface_v1',frame='hand_palm_link',target='dog_doll',vertices_palm_m=[[-.1,-.05,-.08],[.1,.05,.02]])
        def measure(depth,s,d):return {'destination_id':d['id'],'source_depth_stamp_ns':s['streams']['depth']['stamp_ns']}
        def planner(*args,**kw):self.calls.append((args,kw));return {'palm_height_m':.5}
        o=DestinationObserver(rt,context,{'id':'square_low_table#living_room'}, {},'task',np.eye(3),measure=measure,planner=planner,rotation=lambda q:np.eye(3))
        return rt,o
    def test_current_frame_reaches_existing_planner(self):
        rt,o=self.setup_observer();plan=o.measure_and_plan()
        args,kw=self.calls[0]
        self.assertEqual(args[0],'synthetic-depth');self.assertEqual(args[4]['holding_opening_rad'],.32)
        np.testing.assert_allclose(kw['held_volume']['center_m'],[1.,2.,2.97],atol=1e-12)
        self.assertEqual(plan['class_name'],'dog_doll');self.assertTrue(plan['held_pose_is_kinematic'])
        self.assertFalse(kw['held_volume']['current_target_depth_measured'])
    def test_duplicate_frame_not_reused(self):
        rt,o=self.setup_observer();o.measure_and_plan()
        with self.assertRaises(ValueError):o.measure_and_plan()
        rt.stamp+=1;o.measure_and_plan();self.assertEqual(len(self.calls),2)
    def test_wrong_plane_never_reaches_planner(self):
        rt,o=self.setup_observer();o.measure=lambda *a:{'destination_id':'wrong','source_depth_stamp_ns':10}
        with self.assertRaises(ValueError):o.measure_and_plan()
        self.assertFalse(self.calls)

    def test_release_reference_does_not_remeasure_or_retimestamp_plane(self):
        rt,o=self.setup_observer()
        o.measure=lambda *a:self.fail('must not select another shelf layer')
        plan=dict(destination_id=o.destination['id'],plane_map=[0,0,.4],source_depth_stamp_ns=1)
        row=o.observe('release-center',reference_plan=plan)
        self.assertEqual(row['plane']['plane_map'],[0,0,.4])
        self.assertEqual(row['plane']['source_depth_stamp_ns'],1)
        self.assertEqual(row['plane']['geometry_source'],'fixed_planned_support_plane')
        np.testing.assert_allclose(row['volume']['center_m'],[1,2,2.97])
