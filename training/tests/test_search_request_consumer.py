import ast
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import search_request_consumer as module


class RequestTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);(self.root/'config').mkdir();(self.root/'maps').mkdir()
        self.pose=dict(x=1.,y=1.,yaw=0.)
        env=dict(internal_name='L',map='package://handyman_rebuild_ros2/maps/map.yaml',
            rooms=dict(kitchen=dict(region=[[0,0],[2,0],[2,2],[0,2]],search_points=[self.pose])))
        (self.root/'config/environments.yaml').write_text(yaml.safe_dump(dict(environment_files=dict(L='env.yaml'))))
        (self.root/'config/env.yaml').write_text(yaml.safe_dump(env))
        (self.root/'maps/map.yaml').write_text('image: map.pgm\n')
        (self.root/'maps/map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
        self.row=dict(schema='handyman-search-request-v1',task_id='1'*32,stamp_ns=1_000_000_000,
            environment='L',layout='L',map_uri=env['map'],frame_id='map',room='kitchen',target='apple',
            points=[dict(id='L/kitchen/0',pose=self.pose)],actionable=False,requires_map_verification=True)
        self.c=module.RequestConsumer(self.root)
    def receive(self,row=None,event='search_requested',now=1_100_000_000):
        return self.c.receive(event,row or self.row,now,1.)
    def test_prepare_cancel(self):
        result=self.receive();self.assertFalse(result['live_map_verified'])
        self.assertEqual(len(result['map_bundle_sha256']),64)
        self.assertEqual(self.receive(event='search_cancelled')['state'],'cancelled')
        with self.assertRaises(ValueError):self.receive()
    def test_cancel_before_request(self):
        self.receive(event='search_cancelled')
        with self.assertRaises(ValueError):self.receive()
    def test_stale(self):
        with self.assertRaises(ValueError):self.receive(now=4_000_000_000)
    def test_pose_mismatch(self):
        row=copy.deepcopy(self.row);row['points'][0]['pose']['x']=1.2
        with self.assertRaises(ValueError):self.receive(row)
    def test_map_mismatch(self):
        with self.assertRaises(ValueError):self.receive(dict(self.row,map_uri='wrong'))
    def test_expiry(self):
        self.receive();self.assertEqual(self.c.tick(31.)['state'],'expired')
        self.assertIsNone(self.c.active)
    def test_explicit_session_lifetime_replaces_short_lease(self):
        self.c=module.RequestConsumer(self.root,request_lifetime=180.)
        self.receive();self.assertIsNone(self.c.tick(65.))
        self.assertIsNotNone(self.c.active)
        self.assertEqual(self.c.tick(181.)['state'],'expired')
    def test_image_change_changes_digest(self):
        a=module.prepare(self.row,self.root)
        (self.root/'maps/map.pgm').write_bytes(b'changed')
        b=module.prepare(self.row,self.root)
        self.assertNotEqual(a['map_bundle_sha256'],b['map_bundle_sha256'])
    def test_path_escape(self):
        (self.root/'maps/map.yaml').write_text('image: ../../outside.pgm\n')
        with self.assertRaises(ValueError):self.receive()
    def test_no_control_apis(self):
        attrs={n.attr for n in ast.walk(ast.parse(Path(module.__file__).read_text())) if isinstance(n,ast.Attribute)}
        self.assertFalse(attrs & {'create_publisher','create_client','send_goal_async','cancel_goal_async'})

if __name__=='__main__':unittest.main()
