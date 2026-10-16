"""Isolated startup smoke tests; neither case creates navigation clients."""
import json
import os
from pathlib import Path
import subprocess
import tempfile

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='73' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    binary='/tmp/handyman-search-nav-install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2/handyman_search_worker'
    out=Path(tempfile.mkdtemp(prefix='search-worker-startup-'))
    rows=[]
    for name,args,code,expected in (
        ('disabled',[],2,'no navigation clients created'),
        ('invalid_index',['--ros-args','-p','execution.enabled:=true'],3,'invalid point index/lifetime')):
        result=subprocess.run([binary]+args,capture_output=True,text=True,timeout=10)
        log=result.stdout+result.stderr
        (out/(name+'.log')).write_text(log)
        assert result.returncode==code and expected in log,(name,result.returncode,log)
        rows.append(dict(case=name,returncode=result.returncode,passed=True))
    (out/'summary.json').write_text(json.dumps(dict(passed=True,cases=rows),indent=2))
    print('OUTPUT',out);print(json.dumps(rows))

if __name__=='__main__':main()
