import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from active_search_policy import ring_views
from prepare_active_search import snapshot,validate_bundle,digest
from active_run_binding import ActiveRunBinding
from search_dispatch_journal import DispatchJournal


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.repo=Path(self.tmp.name)/'repo';self.out=Path(self.tmp.name)/'bundle'
        self.share=self.repo/'src/handyman_rebuild_ros2'
        cfg=self.share/'config';cfg.mkdir(parents=True)
        maps=self.share/'maps/A';maps.mkdir(parents=True)
        self.env=cfg/'a.yaml'
        self.env.write_text(yaml.safe_dump(dict(internal_name='A',map='package://handyman_rebuild_ros2/maps/A/map.yaml',
            rooms={'room':dict(region=[[0,0],[4,0],[4,4],[0,4]],search_points=[dict(x=2,y=2,yaw=0)])})))
        (cfg/'environments.yaml').write_text(yaml.safe_dump({'environment_files':{'A':'a.yaml'}}))
        self.map=maps/'map.yaml';self.map.write_text('image: map.pgm\n')
        self.image=maps/'map.pgm';self.image.write_bytes(b'P5\n1 1\n255\n\xff')
        self.params=self.repo/'src/handyman_ros2/param/nav2_params.yaml'
        self.params.parent.mkdir(parents=True);self.params.write_text('fixture')
        self.report=dict(footprint_config_sha256=digest(self.params.read_bytes()),reports=[dict(layout='A',
            environment_sha256=digest(self.env.read_bytes()),map_bundle_sha256=digest(self.map.read_bytes()+b'\0'+self.image.read_bytes()),
            rooms=[dict(room='room',status='proposed',views=ring_views(2,2,0))])])

    def make(self):return snapshot(self.repo,self.out,'A','room','canned_juice',self.report)

    def test_roundtrip_and_original_unchanged(self):
        original=self.env.read_bytes();m=self.make()
        self.assertEqual(validate_bundle(self.out/'manifest.json')['task_id'],m['task_id'])
        self.assertEqual(len(m['request']['points']),8);self.assertFalse(m['actionable'])
        self.assertEqual(original,self.env.read_bytes())

    def test_stale_map(self):
        self.image.write_bytes(b'changed')
        with self.assertRaises(ValueError):self.make()

    def test_stale_footprint(self):
        self.params.write_text('changed')
        with self.assertRaises(ValueError):self.make()

    def test_no_candidate(self):
        self.report['reports'][0]['rooms'][0]['status']='no_conservative_candidate'
        with self.assertRaises(ValueError):self.make()

    def test_no_overwrite(self):
        self.make()
        with self.assertRaises(FileExistsError):self.make()

    def test_tampered_snapshot(self):
        self.make();(self.out/'package/config/a.yaml').write_text('changed')
        with self.assertRaises(ValueError):validate_bundle(self.out/'manifest.json')

    def test_identity_mismatch(self):
        m=self.make();m['target']='apple';(self.out/'manifest.json').write_text(json.dumps(m))
        with self.assertRaises(ValueError):validate_bundle(self.out/'manifest.json')

    def test_unsupported_target(self):
        with self.assertRaises(ValueError):snapshot(self.repo,self.out,'A','room','untrained_item',self.report)

    def test_source_overlap(self):
        with self.assertRaises(ValueError):snapshot(self.repo,self.share/'bundle','A','room','apple',self.report)

    def binding(self):
        self.make()
        return ActiveRunBinding(self.out/'manifest.json',self.out/'package')

    def test_binding_one_request_only(self):
        b=self.binding();request=dict(b.bundle['request'],stamp_ns=100)
        b.accept(request)
        with self.assertRaises(ValueError):b.accept(request)

    def test_binding_wrong_task(self):
        b=self.binding();request=dict(b.bundle['request'],task_id='other')
        with self.assertRaises(ValueError):b.accept(request)
        self.assertFalse(b.used)

    def test_binding_changed_heading(self):
        b=self.binding();request=copy.deepcopy(b.bundle['request']);request['points'][0]['pose']['yaw']=.1
        with self.assertRaises(ValueError):b.accept(request)

    def test_binding_footprint_changed_after_init(self):
        b=self.binding();self.params.write_text('changed')
        with self.assertRaises(ValueError):b.check()

    def test_binding_manifest_changed_after_init(self):
        b=self.binding();path=self.out/'manifest.json';path.write_text(path.read_text()+' ')
        with self.assertRaises(ValueError):b.check()

    def test_binding_wrong_package(self):
        self.make()
        with self.assertRaises(ValueError):ActiveRunBinding(self.out/'manifest.json',self.share)

    def test_binding_result_identity(self):
        b=self.binding();m=b.bundle
        row=dict(task_id=m['task_id'],target=m['target'],terminal=True,cleanup_verified=True,
                 actionable=False,does_not_exist_authorized=False,
                 observer_context=dict(task_id=m['task_id'],map_sha256=m['map_bundle_sha256'],point_id='A/room/7'))
        self.assertTrue(b.result_matches(row))
        row['observer_context']['map_sha256']='other'
        self.assertFalse(b.result_matches(row))

    def test_persistent_claim_survives_reopen(self):
        b=self.binding();path=self.out/'dispatch.sqlite3'
        journal=DispatchJournal(path)
        try:b.accept(b.bundle['request'],journal)
        finally:journal.close()
        fresh=ActiveRunBinding(self.out/'manifest.json',self.out/'package')
        journal=DispatchJournal(path)
        try:
            with self.assertRaisesRegex(ValueError,'previously_consumed'):fresh.accept(fresh.bundle['request'],journal)
            self.assertFalse(fresh.used)
        finally:journal.close()

    def test_claim_failure_stops_accept(self):
        b=self.binding()
        class FailedJournal:
            def claim_active_request(self,*args):raise OSError('disk failure')
        with self.assertRaises(OSError):b.accept(b.bundle['request'],FailedJournal())
        self.assertFalse(b.used)


if __name__=='__main__':unittest.main()
