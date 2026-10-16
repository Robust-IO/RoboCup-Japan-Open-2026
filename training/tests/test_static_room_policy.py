import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from static_room_policy import decide,navigation_failure

class StaticRoomTests(unittest.TestCase):
    def report(self,connected=False):
        return dict(map_bundle_sha256='a'*64,method='optimistic_8_connected_point_agent',initial_known_free=True,
                    rooms=[dict(room='room',status='point_connectivity_present' if connected else 'structural_disconnection_candidate',
                                cells=dict(known_free=10,known_connected=10 if connected else 0,optimistic_traversable=10,optimistic_connected=10 if connected else 0))])
    def test_explicit_disconnected(self): self.assertEqual(decide(self.report(),'room','a'*64,explicit_room=True,enable_static_rule=True)['protocol_response'],'Does_not_exist')
    def test_skip(self): self.assertIsNone(decide(self.report(),'room','a'*64,enable_static_rule=True)['protocol_response'])
    def test_connected(self): self.assertEqual(decide(self.report(True),'room','a'*64,explicit_room=True,enable_static_rule=True)['decision'],'retain_as_reachable')
    def test_default_disabled(self): self.assertIsNone(decide(self.report(),'room','a'*64,explicit_room=True)['protocol_response'])
    def test_stale(self): self.assertIsNone(decide(self.report(),'room','b'*64,explicit_room=True,enable_static_rule=True)['protocol_response'])
    def test_unknown(self):
        r=self.report();r['rooms'][0]['status']='unknown_space_requires_review'
        self.assertIsNone(decide(r,'room','a'*64,explicit_room=True,enable_static_rule=True)['protocol_response'])
    def test_failure_never_absence(self):
        for guest in [True,False]:
            r=navigation_failure(guest_blocking=guest); self.assertIsNone(r['protocol_response']); self.assertFalse(r['permanently_exclude_room'])
