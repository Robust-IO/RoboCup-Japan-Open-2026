import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from latest_capture_index import latest,publish


class IndexTest(unittest.TestCase):
    def setUp(self):self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
    def tearDown(self):self.temp.cleanup()
    def frame(self,name,kind='observation.json'):
        path=self.root/name;path.mkdir();(path/kind).write_text('{}');return path/kind
    def test_latest_index_never_scans_history(self):
        expected=self.frame('000005');publish(self.root,'000005','epoch',True)
        with patch('latest_capture_index.os.scandir',side_effect=AssertionError('history_scan')):
            self.assertEqual(latest(self.root),expected)
    def test_partial_next_frame_does_not_replace_last_complete(self):
        expected=self.frame('000005');publish(self.root,'000005','epoch',True);self.frame('000006','snapshot.json')
        self.assertEqual(latest(self.root),expected)
    def test_path_traversal_rejected(self):
        (self.root/'latest-capture.json').write_text(json.dumps(dict(capture='../other',tracking_epoch='epoch',observation_complete=True)))
        with self.assertRaises(ValueError):latest(self.root)
    def test_raw_capture_not_an_identity_observation(self):
        expected=self.frame('000005','snapshot.json');publish(self.root,'000005','epoch',False)
        self.assertEqual(latest(self.root,'snapshot.json'),expected)
        with self.assertRaises(ValueError):latest(self.root)
    def test_legacy_recording_uses_latest_completed(self):
        self.frame('000001');expected=self.frame('000002');self.frame('000003','snapshot.json')
        self.assertEqual(latest(self.root),expected)


if __name__=='__main__':unittest.main()
