import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from start_rgbd_diagnostics import listening_ports,diagnostic_environment

class LauncherTests(unittest.TestCase):
    def test_ipv4_ipv6(self):
        self.assertEqual(listening_ports('LISTEN 0 128 0.0.0.0:9090 0.0.0.0:*\nLISTEN 0 100 [::]:50001 [::]:*'),{9090,50001})
    def test_empty(self): self.assertEqual(listening_ports('State Recv-Q Send-Q Local Address:Port Peer Address:Port'),set())
    def test_ignore_connections(self): self.assertEqual(listening_ports('ESTAB 0 0 127.0.0.1:9090 127.0.0.1:20'),set())
    def test_legacy_domain(self): self.assertEqual(diagnostic_environment({})['ROS_DOMAIN_ID'],'71')
    def test_preserve_cyclone(self):
        original=dict(ROS_DOMAIN_ID='73',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',LD_LIBRARY_PATH='/test')
        actual=diagnostic_environment(original)
        self.assertEqual(actual['ROS_DOMAIN_ID'],'73')
        self.assertEqual(actual['RMW_IMPLEMENTATION'],'rmw_cyclonedds_cpp')
        self.assertEqual(actual['LD_LIBRARY_PATH'],'/test')
        self.assertNotIn('ROS_LOCALHOST_ONLY',original)
    def test_explicit_domain(self): self.assertEqual(diagnostic_environment({'ROS_DOMAIN_ID':'71'},73)['ROS_DOMAIN_ID'],'73')
    def test_invalid_domain(self):
        with self.assertRaises(ValueError):diagnostic_environment({},173)

if __name__=='__main__': unittest.main()
