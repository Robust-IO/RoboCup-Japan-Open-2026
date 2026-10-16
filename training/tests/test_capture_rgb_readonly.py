import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import zlib
import struct

spec = importlib.util.spec_from_file_location('capture', Path(__file__).parents[1]/'scripts/capture_rgb_readonly.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class CaptureTests(unittest.TestCase):
    def test_stride_rgb_orientation(self):
        msg = SimpleNamespace(width=1, height=2, step=4, encoding='rgb8', data=bytes([255,0,0,99,0,0,255,99]))
        png = module.encode_png(msg)
        offset, payload = 8, b''
        while offset < len(png):
            size = struct.unpack('>I', png[offset:offset+4])[0]
            if png[offset+4:offset+8] == b'IDAT':
                payload += png[offset+8:offset+8+size]
            offset += 12+size
        self.assertEqual(zlib.decompress(payload), bytes([0,255,0,0,0,0,0,255]))

    def test_bgr_equals_rgb(self):
        rgb=SimpleNamespace(width=1,height=1,step=3,encoding='rgb8',data=bytes([1,2,3]))
        bgr=SimpleNamespace(width=1,height=1,step=3,encoding='bgr8',data=bytes([3,2,1]))
        self.assertEqual(module.encode_png(rgb),module.encode_png(bgr))

    def test_bad_payload(self):
        with self.assertRaises(ValueError):
            module.encode_png(SimpleNamespace(width=2,height=1,step=6,encoding='rgb8',data=b'1'))


if __name__ == '__main__':
    unittest.main()
