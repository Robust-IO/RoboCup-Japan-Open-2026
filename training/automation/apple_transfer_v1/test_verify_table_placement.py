import copy
import math
import unittest
from verify_table_placement import verify
from test_tabletop_support import edge_plan, support_fixture


class PlacementTest(unittest.TestCase):
    def fixture(self):
        candidate=dict(inside_table_with_apple_margin=True,center_map=[10.86,-6.53,.53],estimated_bottom_gap_m=.003,confidence=.95,footprint_margin_m=[.22,.65],candidate=dict(center_m=[.705,.077,.53]))
        frames=[dict(capture=str(i),source_stamp_ns=(i+1)*1_000_000_000,joints=dict(arm_lift_joint=0,arm_flex_joint=0,wrist_flex_joint=-math.pi/2,hand_motor_joint=.65),apple_measurements=[copy.deepcopy(candidate)]) for i in range(3)]
        actions={k:dict(status='passed') for k in ('release','raise','retract')};actions['release']['events']=[dict(event='command',kind='hand',positions=[0,0,0,0,0,.65])]
        return dict(frames=frames),dict(center_map=[10.86,-6.53,.545]),actions

    def test_success(self):self.assertTrue(verify(*self.fixture())['placement_success_verified'])
    def test_legacy_success_does_not_claim_edge_safety(self):
        self.assertFalse(verify(*self.fixture())['table_edge_safety_verified'])
    def edge_fixture(self, y):
        audit, prior, actions = self.fixture()
        destination = 'square_low_table#living_room'
        _, model = support_fixture()
        for frame in audit['frames']:
            frame['apple_measurements'][0]['center_map'][:2] = [1.971, y]
            frame['apple_measurements'][0]['candidate']['radius_m'] = .032
        prior['center_map'][:2] = [1.971, y]
        return audit, prior, actions, destination, 'living_room', edge_plan(model, destination)
    def test_safe_stable_placement_passes_both_checks(self):
        a, p, c, d, r, plan = self.edge_fixture(3.76)
        result = verify(a, p, c, d, r, placement_plan=plan)
        self.assertTrue(result['placement_success_verified'])
        self.assertTrue(result['table_edge_safety_verified'])
    def test_stable_but_edge_unsafe_placement_is_not_success(self):
        a, p, c, d, r, plan = self.edge_fixture(3.644)
        result = verify(a, p, c, d, r, placement_plan=plan)
        self.assertFalse(result['placement_success_verified'])
        self.assertFalse(result['table_edge_safety_verified'])
    def test_single_image_insufficient(self):
        a,p,c=self.fixture();a['frames']=a['frames'][:1]
        self.assertFalse(verify(a,p,c)['placement_success_verified'])
    def test_floating_is_not_placed(self):
        a,p,c=self.fixture();a['frames'][0]['apple_measurements'][0]['estimated_bottom_gap_m']=.08
        self.assertFalse(verify(a,p,c)['placement_success_verified'])
    def test_moving_object_not_stable(self):
        a,p,c=self.fixture();a['frames'][0]['apple_measurements'][0]['center_map'][0]+=.03
        self.assertFalse(verify(a,p,c)['placement_success_verified'])
    def test_closed_hand_not_finished(self):
        a,p,c=self.fixture();a['frames'][0]['joints']['hand_motor_joint']=.4149
        self.assertFalse(verify(a,p,c)['placement_success_verified'])
    def test_duplicate_capture_or_stamp_is_not_multiframe(self):
        a,p,c=self.fixture();a['frames'][1]['source_stamp_ns']=a['frames'][0]['source_stamp_ns']
        self.assertFalse(verify(a,p,c)['placement_success_verified'])
        a,p,c=self.fixture();a['frames'][1]['capture']=a['frames'][0]['capture']
        self.assertFalse(verify(a,p,c)['placement_success_verified'])


if __name__=='__main__':unittest.main()
