from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from capture_file_writer import write_capture_files


class WriterTests(unittest.TestCase):
    def test_all_original_pixels_depth_nan_and_stamps_complete_before_return(self):
        rgb=np.arange(900,dtype=np.uint8).reshape(15,20,3);hand=255-rgb
        depth=np.ones((15,20),np.float32);depth[0,0]=np.nan
        report=dict(joint_stamp_ns=123,streams=dict(depth=dict(stamp_ns=124)))
        with tempfile.TemporaryDirectory() as folder,ThreadPoolExecutor(max_workers=3) as pool:
            root=Path(folder);write_capture_files(root,rgb,hand,depth,report,pool)
            np.testing.assert_array_equal(cv2.imread(str(root/'rgb.png')),rgb)
            np.testing.assert_array_equal(cv2.imread(str(root/'hand.png')),hand)
            np.testing.assert_array_equal(np.load(root/'depth_m.npy',allow_pickle=False),depth)
            self.assertEqual(json.loads((root/'snapshot.json').read_text()),report)

    def test_failed_image_write_propagates_never_accepted_as_complete(self):
        with tempfile.TemporaryDirectory() as folder,ThreadPoolExecutor(max_workers=3) as pool:
            with patch('capture_file_writer.cv2.imwrite',return_value=False):
                with self.assertRaisesRegex(RuntimeError,'Image write failed'):
                    write_capture_files(Path(folder),np.zeros((2,2,3),np.uint8),np.zeros((2,2,3),np.uint8),np.ones((2,2),np.float32),{},pool)


if __name__=='__main__':unittest.main()
