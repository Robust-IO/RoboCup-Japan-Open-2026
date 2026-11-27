import unittest
from types import SimpleNamespace
from threading import Event
from transfer_actions import place
from container_transfer import drop_and_check,ContainerPolicy
class FinishOnReleaseTests(unittest.TestCase):
 def forbidden(self,*a,**k):self.fail('post-release movement or observation must not run')
 def test_table_no_raise_retreat_fold(self):
  core=SimpleNamespace(release_open=lambda rt,check:check(),raise_after_release=self.forbidden,fold_after_withdrawal=self.forbidden)
  r=place(None,core,context={'target':'apple'},release_rotation=None,observe_hold=self.forbidden,
    measure_and_plan=self.forbidden,prepare_standoff=self.forbidden,approach=self.forbidden,
    verify_support=lambda p:{'ready':True},withdraw_empty=self.forbidden,bottom_observed=True,
    prepared_plan={},finish_on_release=True)
  self.assertTrue(r['release_sent']);self.assertFalse(r['cleanup_completed']);self.assertFalse(r['placement_verified'])
 def test_container_no_retreat_or_floor_scan(self):
  opening={'destination_id':'bin','polygon_map_m':[[-1,-1],[1,-1],[1,1],[-1,1]],'rim_height_m':.5}
  a=SimpleNamespace(observe_opening=lambda d:opening,bottom_below_palm_m=lambda:.1,approach=lambda p:None,
    current_gripper_and_bottom=lambda:([0,0,.8],.6),release_once=lambda:123,
    withdraw_empty=self.forbidden,capture_external_floor=self.forbidden)
  r=drop_and_check(a,'apple','bin',cancel=Event(),policy=ContainerPolicy(finish_on_release=True))
  self.assertEqual(r['basis'],'gripper_release_completed');self.assertFalse(r['placement_verified'])
