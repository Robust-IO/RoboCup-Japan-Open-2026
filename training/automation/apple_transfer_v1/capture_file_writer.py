"""Lossless concurrent D-drive saves; return only after ALL files are complete."""
from concurrent.futures import wait
import json
import cv2
import numpy as np


def write_capture_files(output,rgb,hand,depth,report,executor):
    def image(name,pixels):
        if not cv2.imwrite(str(output/name),pixels,[cv2.IMWRITE_PNG_COMPRESSION,1]):
            raise RuntimeError('Image write failed:'+name)
    def metadata():
        np.save(output/'depth_m.npy',depth,allow_pickle=False)
        (output/'snapshot.json').write_text(json.dumps(report,indent=2))
    jobs=[executor.submit(image,'rgb.png',rgb),executor.submit(image,'hand.png',hand),executor.submit(metadata)]
    wait(jobs)
    for job in jobs:job.result()
