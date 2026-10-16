import copy
from pathlib import Path
from types import SimpleNamespace as S
import unittest
import test_search_request_consumer as fixtures
from search_request_consumer import RequestMapGate


class GateTests(unittest.TestCase):
    def setUp(self):
        fixtures.RequestTests.setUp(self)
        self.active=self.c.receive('search_requested',self.row,1_100_000_000,1.)
        self.map=S(header=S(frame_id='map'),info=S(width=1,height=1,resolution=.05,
            origin=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),data=[0])
        self.gate=RequestMapGate(self.c)
        self.gate.install('1'*32,self.map,self.active['map_bundle_sha256'])
    def test_match(self):
        row=self.gate.receive_map(self.map,'pub',['pub'])
        self.assertTrue(row['live_map_verified']);self.assertFalse(row['actionable'])
    def test_change_revokes(self):
        self.gate.receive_map(self.map,'pub',['pub'])
        changed=copy.deepcopy(self.map);changed.data[0]=100
        self.assertEqual(self.gate.receive_map(changed,'pub',['pub'])['state'],'map_verification_revoked')
        self.assertIsNone(self.c.active)
    def test_cancel_during_decode(self):
        self.c.receive('search_cancelled',self.row,1_100_000_000,1.1)
        self.assertFalse(self.gate.install('1'*32,self.map,self.active['map_bundle_sha256']))
        self.assertIsNone(self.gate.receive_map(self.map,'pub',['pub']))
    def test_publisher_change(self):
        self.gate.receive_map(self.map,'pub',['pub'])
        self.assertIsNotNone(self.gate.check(['other']))
    def test_ambiguous_publisher(self):
        self.assertIsNotNone(self.gate.receive_map(self.map,'pub',['pub','other']))
    def test_local_file_change(self):
        (self.root/'maps/map.pgm').write_bytes(b'changed')
        self.assertEqual(self.gate.check(['pub'])['reason'],'local_map_or_environment_changed')
    def test_wrong_decode_digest(self):
        with self.assertRaises(ValueError):self.gate.install('1'*32,self.map,'b'*64)

if __name__=='__main__':unittest.main()
