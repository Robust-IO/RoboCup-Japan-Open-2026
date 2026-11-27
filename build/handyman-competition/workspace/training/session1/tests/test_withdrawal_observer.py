import unittest
from types import SimpleNamespace
import numpy as np
from withdrawal_observer import WithdrawalObserver
class ObserverTests(unittest.TestCase):
    def build(self,held):
        snapshot=dict(streams={'depth':{'stamp_ns':2}},joints={'hand_motor_joint':.3},tracking_transforms={'map_from_base_at_depth':{'quaternion':[0,0,0,1]}})
        rt=SimpleNamespace(palm_pose=lambda s:dict(translation=[.5,0,.5],quaternion=[0,0,0,1]),fresh_measurement=lambda s:None,save=lambda *a:'/synthetic')
        physical=SimpleNamespace(sample=lambda opening:{'parts':[dict(vertices_palm_m=[[0,0,0],[.1,.1,.1]])]})
        backend=SimpleNamespace(rt=rt,physical=physical,_sample=lambda:dict(snapshot=snapshot,depth=None))
        table=dict(id='table',axis_aligned_bounds={'center':{'x':1,'y':0},'size':{'x':1,'y':1}})
        plane=dict(destination_id='table',source_depth_stamp_ns=2,observed_xy_min=[.6,-.4],observed_xy_max=[1.4,.4])
        return WithdrawalObserver(backend,table,lambda:held,measure=lambda *a:plane,map_point=lambda s,p:p,rotation=lambda q:np.eye(3))
    def test_includes_hand_and_current_held_surface(self):
        result=self.build(dict(frame='hand_palm_link',vertices_palm_m=[[.2,0,0]]))()
        self.assertEqual(len(result['points_map']),3)
        self.assertEqual(result['points_map'][-1],[.7,0,.5])
        self.assertEqual(result['table_center'],[1,0]);self.assertEqual(result['table_half'],[.5,.5])
    def test_empty_withdrawal_does_not_include_old_target(self):
        observer=self.build(None);r=observer();self.assertEqual(len(r['points_map']),2)
        self.assertFalse(r['includes_held_surface'])
        with self.assertRaises(ValueError):observer()

    def test_no_surface_fit_even_when_depth_cannot_show_table(self):
        observer=self.build(dict(frame='hand_palm_link',vertices_palm_m=[[.2,0,0]]))
        def forbidden(*a):raise ValueError('dominant_table_plane_not_established')
        observer.measure=forbidden
        result=observer()
        self.assertFalse(result['plane_redetection_required'])
        self.assertEqual(result['furniture_bounds_basis'],'exported_axis_aligned_bounds')
        self.assertEqual(result['table_center'],[1,0])
