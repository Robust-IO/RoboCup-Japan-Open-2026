import tempfile,unittest,json
from pathlib import Path
from unittest.mock import patch
import build_package as b
class BuildTests(unittest.TestCase):
    def test_manifest_paths_cannot_escape(self):
        for path in ('/tmp/x','../x','a/../../x'):
            with self.assertRaises(ValueError):b.safe_relative(path)
    def test_existing_output_never_overwritten(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'keep').write_text('original')
            with self.assertRaises(ValueError):b.build(p)
            self.assertEqual((p/'keep').read_text(),'original')
    def test_source_only_renders_and_blocks_execution(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);here=root/'competition/handyman';here.mkdir(parents=True)
            source=here/'main.py';source.write_text("ROOT='/__HANDYMAN_PACKAGE__'\n")
            (here/'package_manifest.json').write_text(json.dumps({'path_token':'/__HANDYMAN_PACKAGE__','files':[{'source':'competition/handyman/main.py','destination':'main.py'}]}))
            out=root/'build/out'
            with patch.object(b,'REPO',root),patch.object(b,'HERE',here),patch.object(b.subprocess,'check_output',return_value='abc'):
                result=b.build(out)
            self.assertFalse(result['runtime_checked']);self.assertTrue((out/'PACKAGING_IN_PROGRESS').exists())
            self.assertIn(str(out),(out/'main.py').read_text());self.assertIn('/__HANDYMAN_PACKAGE__',source.read_text())
if __name__=='__main__':unittest.main()
