import unittest
from threading import Event
from container_transfer import OutsideFloorEvidence,release_ready,drop_and_check
OPENING=dict(destination_id='bin#living_room',polygon_map_m=[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]],rim_height_m=.6)
def row(stamp,**kw):
    return dict(valid_frame=True,floor_visible=True,evidence_ref=str(stamp),destination_id='bin#living_room',depth_stamp_ns=stamp,floor_targets=[],**kw)
class Adapter:
    def __init__(self):self.releases=0;self.moves=0;self.stamp=1_000_000_000;self.recovered=0;self.drop_on_floor=False
    def observe_opening(self,destination):return OPENING
    def bottom_below_palm_m(self):return .1
    def approach(self,plan):self.moves+=1
    def current_gripper_and_bottom(self):return [0,0,.75],.65
    def release_once(self):self.releases+=1;self.stamp+=1_000_000_000;return self.stamp
    def withdraw_empty(self):pass
    def capture_external_floor(self):
        self.stamp+=1_000_000_000;r=row(self.stamp)
        if self.drop_on_floor:r['floor_targets']=[{'target':'canned_juice','center_map_m':[.4,0,.05]}]
        return r
    def recover_floor_target(self,targets):self.recovered+=1;self.drop_on_floor=False;return True
class ContainerTests(unittest.TestCase):
    def test_release_accepts_any_inside_point_above_rim(self):
        self.assertTrue(release_ready([.15,0,.75],.65,OPENING))
        self.assertFalse(release_ready([.3,0,.75],.65,OPENING))
        self.assertFalse(release_ready([0,0,.5],.5,OPENING))
    def test_three_frames_do_not_claim_inside_proof(self):
        p=OutsideFloorEvidence('canned_juice','bin#living_room',1)
        for stamp in [2_000_000_000,2_200_000_000,2_400_000_000]:r=p.observe(row(stamp))
        self.assertEqual(r['state'],'clear');self.assertFalse(r['inside_container_proven'])
    def test_stale_invalid_or_no_floor_never_pass(self):
        p=OutsideFloorEvidence('canned_juice','bin#living_room',1)
        for _ in range(5):
            r=row(2_000_000_000);r['floor_visible']=False
            self.assertEqual(p.observe(r)['state'],'inconclusive')
        self.assertFalse(p.clear)
    def test_clear_path_one_release(self):
        a=Adapter();r=drop_and_check(a,'canned_juice','bin#living_room',cancel=Event())
        self.assertTrue(r['placement_verified']);self.assertEqual(a.releases,1);self.assertEqual(a.recovered,0)
    def test_positive_floor_detection_recovers_then_retries(self):
        a=Adapter();a.drop_on_floor=True;r=drop_and_check(a,'canned_juice','bin#living_room',cancel=Event())
        self.assertTrue(r['placement_verified']);self.assertEqual(a.recovered,1);self.assertEqual(a.releases,2)
    def test_cancel_sends_no_motion(self):
        e=Event();e.set();a=Adapter()
        with self.assertRaises(RuntimeError):drop_and_check(a,'canned_juice','bin#living_room',cancel=e)
        self.assertEqual(a.moves,0)
