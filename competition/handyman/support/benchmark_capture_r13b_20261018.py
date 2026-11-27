"""Compare unchanged recorded pixels through old/new workers; NO ROS/control."""
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import tarfile
import time
from concurrent.futures import ThreadPoolExecutor
import cv2
import numpy as np

base=Path('/__HANDYMAN_PACKAGE__/runtime');runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
out=base/'apple-capture-r13b-benchmark-20261018-01';out.mkdir(exist_ok=False)
candidate=out/'candidate';shutil.copytree(runtime,candidate,ignore=shutil.ignore_patterns('__pycache__'))
with tarfile.open(base/'apple-capture-r13b-benchmark-patch.tar.gz') as archive:
    members=archive.getmembers()
    if any('/' in m.name or not m.isfile() for m in members):raise ValueError('flat_files_required')
    archive.extractall(candidate)
with (out/'tests.log').open('x') as stream:
    tests=subprocess.run([sys.executable,'-m','unittest','discover','-p','test_*.py'],cwd=candidate,stdout=stream,stderr=subprocess.STDOUT)
if tests.returncode:raise ValueError('candidate_tests_failed')
sys.path.insert(0,str(runtime));from apple_worker_client import Worker
registry=base/'active_detection_model.json'
python='/__HANDYMAN_PACKAGE__/project/.pixi/envs/default/bin/python'
source=base/'apple-capture-budget-r13-source-readonly-20261018-01/source-observe'
captures=[source/f'{i:06d}' for i in (10,30,60,90,120)]
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for c in captures for p in c.iterdir() if p.is_file()}
old_script=(runtime/'apple_inference_worker.py').read_text()
workers={};measurements={}
try:
    for label,root in (('old',runtime),('new',candidate)):
        # Worker uses its own __file__ folder, so call the candidate client class
        # loaded directly instead of rebinding any production files.
        import importlib.util
        spec=importlib.util.spec_from_file_location('bench_client_'+label,root/'apple_worker_client.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        worker=module.Worker(python,str(registry),out/(label+'.stderr.log'),imgsz=960,confirm_apple_roi=True)
        workers[label]=worker;rows=[]
        for capture in captures:
            start=time.perf_counter();result=worker.infer(capture)
            rows.append(dict(capture=str(capture),roundtrip_s=time.perf_counter()-start,result=result))
        measurements[label]=rows
    keys=('status','reason','detections','candidate','support_evidence','tracking_proposals','near_grasp_roi_confirmation')
    for old,new in zip(measurements['old'],measurements['new']):
        for key in keys:
            a=old['result'].get(key);b=new['result'].get(key)
            if key=='near_grasp_roi_confirmation' and a is not None and b is not None:
                a={k:v for k,v in a.items() if k!='processing_s'};b={k:v for k,v in b.items() if k!='processing_s'}
            if a!=b:raise ValueError('original_result_mismatch:'+old['capture']+':'+key)
finally:
    for worker in workers.values():worker.close()

capture=captures[0];rgb=cv2.imread(str(capture/'rgb.png'));hand=cv2.imread(str(capture/'hand.png'))
depth=np.load(capture/'depth_m.npy');snapshot=(capture/'snapshot.json').read_text();saving=[]
def save_image(path,pixels):
    if not cv2.imwrite(str(path),pixels,[cv2.IMWRITE_PNG_COMPRESSION,1]):raise ValueError('image_write_failed')
def save_metadata(folder):
    np.save(folder/'depth_m.npy',depth);(folder/'snapshot.json').write_text(snapshot)
with ThreadPoolExecutor(max_workers=3) as executor:
    for mode in ('serial','parallel'):
        for i in range(7):
            folder=out/(mode+str(i));folder.mkdir();start=time.perf_counter()
            if mode=='serial':
                save_image(folder/'rgb.png',rgb);save_image(folder/'hand.png',hand);save_metadata(folder)
            else:
                futures=[executor.submit(save_image,folder/'rgb.png',rgb),executor.submit(save_image,folder/'hand.png',hand),executor.submit(save_metadata,folder)]
                for future in futures:future.result()
            saving.append(dict(mode=mode,seconds=time.perf_counter()-start))
            if (not np.array_equal(cv2.imread(str(folder/'rgb.png')),rgb)
                or not np.array_equal(cv2.imread(str(folder/'hand.png')),hand)
                or not np.array_equal(np.load(folder/'depth_m.npy'),depth,equal_nan=True)
                or (folder/'snapshot.json').read_text()!=snapshot):raise ValueError('saved_pixels_or_snapshot_changed')
if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=digest for p,digest in hashes.items()):raise ValueError('original_input_changed')
report=dict(status='recorded_frame_equivalence_and_latency_measured',robot_commands_sent=False,
    original_inputs_unchanged=True,results_exactly_equal=True,tests_passed=True,
    worker_medians_s={k:statistics.median(r['roundtrip_s'] for r in rows) for k,rows in measurements.items()},
    save_medians_s={m:statistics.median(r['seconds'] for r in saving if r['mode']==m) for m in ('serial','parallel')},
    measurements=measurements,saving=saving)
(out/'result.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('measurements','saving')}),flush=True)
