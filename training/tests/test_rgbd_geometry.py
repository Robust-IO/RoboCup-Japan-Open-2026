import importlib.util
from pathlib import Path
import struct
import unittest
s=importlib.util.spec_from_file_location('geometry',Path(__file__).resolve().parents[1]/'scripts/rgbd_geometry.py')
m=importlib.util.module_from_spec(s)
s.loader.exec_module(m)

class GeometryTests(unittest.TestCase):
    def test_mm_even_below_100(self):
        self.assertAlmostEqual(m.depth_at(struct.pack('<H',50),1,1,2,'16UC1',False,0,0),.05)
    def test_bigendian_and_padding(self):
        self.assertEqual(m.depth_at(struct.pack('>HH',2000,0),1,1,4,'16UC1',True,0,0),2)
    def test_float_and_invalid(self):
        self.assertEqual(m.depth_at(struct.pack('<f',1.2),1,1,4,'32FC1',False,0,0),struct.unpack('<f',struct.pack('<f',1.2))[0])
        self.assertIsNone(m.depth_at(struct.pack('<f',float('nan')),1,1,4,'32FC1',False,0,0))
        self.assertIsNone(m.depth_at(bytes(2),1,1,2,'16UC1',False,0,0))
    def test_optical_axes(self):
        self.assertEqual(m.optical_point(420,340,2,[500,0,320,0,500,240,0,0,1]),(.4,.4,2))
    def test_bad_buffer(self):
        with self.assertRaises(ValueError):
            m.depth_at(bytes(1),1,1,2,'16UC1',False,0,0)

if __name__=='__main__':
    unittest.main()
