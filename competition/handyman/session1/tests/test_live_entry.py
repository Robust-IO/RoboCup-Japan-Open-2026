import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
class EntryTests(unittest.TestCase):
    def test_default_only_checks_files_without_ros(self):
        with tempfile.TemporaryDirectory() as root:
            p=Path(root);asset=p/'asset';asset.write_text('{}')
            cfg=dict(layout='LayoutA',task={'object':'dog_doll'},source_table_id='white_side_table#living_room',destination_id='square_low_table#living_room',**{k:str(asset) for k in ['environment','semantic_map','registry','model_python']})
            config=p/'config.json';config.write_text(json.dumps(cfg))
            entry=Path(__file__).resolve().parents[1]/'run_live_session1.py'
            result=subprocess.run([sys.executable,str(entry),'--config',str(config),'--hand',str(asset)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            report=json.loads(result.stdout);self.assertFalse(report['ros_initialized']);self.assertFalse(report['live_validation_passed'])
    def test_wrong_task_rejected_before_ros(self):
        with tempfile.TemporaryDirectory() as root:
            cfg=Path(root)/'config.json';cfg.write_text('{}')
            entry=Path(__file__).resolve().parents[1]/'run_live_session1.py'
            result=subprocess.run([sys.executable,str(entry),'--config',str(cfg)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0);self.assertIn('known_session_configuration_required',result.stderr)
