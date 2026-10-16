"""Four-way middleware comparison using official C++ examples, domain 73.

No Unity connection, services, navigation actions or robot command publishers.
Exit zero means all cases ran, not that all communication directions passed.
Read summary.json for individual outcomes.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cyclone-manifest',type=Path,required=True)
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Requires isolated domain 73, localhost only')
    manifest=json.loads(a.cyclone_manifest.read_text())
    if set(manifest)!={'RMW_IMPLEMENTATION','LD_LIBRARY_PATH','AMENT_PREFIX_PATH'} or manifest['RMW_IMPLEMENTATION']!='rmw_cyclonedds_cpp':
        raise ValueError('invalid_test_manifest')
    base=dict(os.environ)
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE',
                'RMW_FASTRTPS_USE_QOS_FROM_XML','CYCLONEDDS_URI','ROS_DISCOVERY_SERVER'):
        base.pop(key,None)
    envs={'fast':dict(base,RMW_IMPLEMENTATION='rmw_fastrtps_cpp'),
          'cyclone':dict(base,**manifest)}
    out=Path(tempfile.mkdtemp(prefix='search-mixed-dds-'))
    print('OUTPUT',out,flush=True)
    summary=[]
    for sender,receiver in [('fast','fast'),('fast','cyclone'),('cyclone','fast'),('cyclone','cyclone')]:
        case=sender+'-to-'+receiver
        folder=out/case;folder.mkdir()
        procs=[];streams=[]
        try:
            for name,impl in [('listener',receiver),('talker',sender)]:
                stream=(folder/(name+'.log')).open('w');streams.append(stream)
                procs.append(subprocess.Popen(['/opt/ros/humble/lib/demo_nodes_cpp/'+name],
                    env=envs[impl],stdout=stream,stderr=stream))
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                if any(proc.poll() is not None for proc in procs): break
                time.sleep(.1)
        finally:
            for proc in procs:
                if proc.poll() is None: proc.terminate()
            for proc in procs:
                try: proc.wait(timeout=3)
                except subprocess.TimeoutExpired: proc.kill();proc.wait()
            for stream in streams:stream.close()
        sent=re.findall(r"Publishing: 'Hello World: (\d+)'",(folder/'talker.log').read_text())
        received=re.findall(r'I heard: \[Hello World: (\d+)\]',(folder/'listener.log').read_text())
        started=all(proc.returncode in (0,-15) for proc in procs) and bool(sent)
        row=dict(case=case,started=started,sent=len(sent),received=len(received),
            matched_ids=len(set(sent)&set(received)),communication_passed=started and bool(set(sent)&set(received)),
            robot_control=False)
        summary.append(row)
        (out/'summary.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps(row),flush=True)


if __name__=='__main__':main()
