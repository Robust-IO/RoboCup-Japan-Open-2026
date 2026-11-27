"""Save only channels actually acquired, preserving existing filenames."""
import json
from concurrent.futures import wait
import cv2,numpy as np

def save(runtime,sample,name,measurement=None):
    output=runtime.root/'captures'/name;output.mkdir(parents=True,exist_ok=False);jobs=[]
    def image(filename,value):
        if not cv2.imwrite(str(output/filename),value,[cv2.IMWRITE_PNG_COMPRESSION,1]):raise RuntimeError('Image write failed:'+filename)
    if sample.get('rgb') is not None:jobs.append(runtime.pool.submit(image,'rgb.png',sample['rgb'][:,:,::-1].copy()))
    if sample.get('hand') is not None:jobs.append(runtime.pool.submit(image,'hand.png',sample['hand']))
    def metadata():
        if sample.get('depth') is not None:np.save(output/'depth_m.npy',sample['depth'],allow_pickle=False)
        (output/'snapshot.json').write_text(json.dumps(sample['snapshot'],indent=2))
        if measurement is not None:(output/'object-measurement.json').write_text(json.dumps(measurement,indent=2))
    jobs.append(runtime.pool.submit(metadata));wait(jobs)
    for job in jobs:job.result()
    return output
