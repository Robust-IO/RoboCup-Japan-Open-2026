import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cyclone_test_env import environment


class EnvironmentTests(unittest.TestCase):
    def test_missing_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError): environment(directory,73,{})

    def test_isolated_process_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix=Path(directory)
            for relative in ('lib/librmw_cyclonedds_cpp.so','lib/x86_64-linux-gnu/libddsc.so.0',
                             'share/ament_index/resource_index/rmw_typesupport/rmw_cyclonedds_cpp'):
                path=prefix/relative;path.parent.mkdir(parents=True,exist_ok=True);path.touch()
            inherited={'ROS_DOMAIN_ID':'71','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp',
                       'LD_LIBRARY_PATH':'/old/lib','CYCLONEDDS_URI':'old','FASTRTPS_DEFAULT_PROFILES_FILE':'old'}
            before=dict(inherited)
            result=environment(prefix,73,inherited)
            self.assertEqual(inherited,before)
            self.assertEqual(result['ROS_DOMAIN_ID'],'73')
            self.assertEqual(result['RMW_IMPLEMENTATION'],'rmw_cyclonedds_cpp')
            self.assertTrue(result['LD_LIBRARY_PATH'].endswith(':/old/lib'))
            self.assertNotIn('CYCLONEDDS_URI',result)
            self.assertNotIn('FASTRTPS_DEFAULT_PROFILES_FILE',result)
            config=prefix/'explicit config.xml';config.write_text('<CycloneDDS/>')
            configured=environment(prefix,73,inherited,config)
            self.assertEqual(configured['CYCLONEDDS_URI'],config.as_uri())
            self.assertEqual(inherited,before)
            with self.assertRaises(ValueError):environment(prefix,73,inherited,prefix/'missing.xml')

    def test_invalid_domain(self):
        with self.assertRaises(ValueError):environment('/tmp',173,{})


if __name__=='__main__':unittest.main()
