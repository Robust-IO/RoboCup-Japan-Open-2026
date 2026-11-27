"""Read-only TCP counters alongside the model-free RGBD probe. No payload capture."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import time


def parse_ss(output):
    rows=[];current=None
    for line in output.splitlines():
        fields=line.split()
        if fields and fields[0]=='ESTAB' and len(fields)>=5:
            current=None
            if fields[3].rsplit(':',1)[-1] not in ('9090','50001'):continue
            owner=re.search(r'pid=(\d+),fd=(\d+)',line)
            current=dict(local=fields[3],peer=fields[4],recv_queue=int(fields[1]),send_queue=int(fields[2]),
                pid=int(owner[1]) if owner else None,fd=int(owner[2]) if owner else None)
            rows.append(current)
        elif line[:1].isspace() and current is not None:
            for key,value in re.findall(r'\b(bytes_received|bytes_sent|segs_in|data_segs_in|lastrcv):([0-9]+)',line):
                current[key]=int(value)
    return rows


def changes(before,after,seconds):
    def key(row):return (row['local'],row['peer'],row['pid'],row['fd'])
    old={key(row):row for row in before};result=[]
    for row in after:
        first=old.get(key(row));item=dict(row)
        a=first.get('bytes_received') if first else None;b=row.get('bytes_received')
        if a is None or b is None:item['delta_status']='new_connection_or_missing_counter'
        elif b<a:item['delta_status']='counter_reset'
        else:item.update(delta_status='comparable',received_bytes_delta=b-a,bytes_per_second=(b-a)/seconds)
        result.append(item)
    return result


def snmp():
    lines=Path('/proc/net/snmp').read_text().splitlines();result={}
    for i in range(0,len(lines)-1,2):
        keys=lines[i].split();values=lines[i+1].split()
        if keys[0] in ('Ip:','Udp:'):
            result[keys[0][:-1]]=dict(zip(keys[1:],map(int,values[1:])))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=int,default=8);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not 1<=a.seconds<=20:p.error('seconds must be 1..20')
    a.output.mkdir(parents=True,exist_ok=False)
    samples=[];initial=snmp();proc=None;log=(a.output/'probe.log').open('w')
    def sample():
        samples.append(dict(monotonic=time.monotonic(),wall_ns=time.time_ns(),
            connections=parse_ss(subprocess.check_output(['ss','-tinp'],text=True,timeout=3))))
    try:
        sample()
        proc=subprocess.Popen([sys.executable,str(Path(__file__).with_name('probe_rgbd_streams.py')),
            '--seconds',str(a.seconds),'--output',str(a.output/'streams.json')],stdout=log,stderr=log)
        deadline=time.monotonic()+a.seconds+10
        while proc.poll() is None:
            if time.monotonic()>deadline:raise RuntimeError('metadata probe timeout')
            time.sleep(.5);sample()
        sample()
        elapsed=samples[-1]['monotonic']-samples[0]['monotonic']
        final=snmp()
        report=dict(read_only=True,model_loaded=False,elapsed_s=elapsed,probe_exit=proc.returncode,
            connections=changes(samples[0]['connections'],samples[-1]['connections'],elapsed),
            caveats=['TCP endpoints are not automatically mapped to ROS topics.',
                     'TCP bytes received do not prove BSON decoding or ROS publishing.',
                     'Kernel counters cover the whole WSL, not only these connections.',
                     'TCP interval includes probe startup/shutdown, slightly wider than ROS sampling.'],
            kernel_deltas={group:{key:final[group][key]-value for key,value in values.items()
                if key in ('ReasmReqds','ReasmOKs','ReasmFails','InErrors','RcvbufErrors','InDatagrams')}
                for group,values in initial.items()})
        (a.output/'tcp_samples.json').write_text(json.dumps(samples,indent=2))
        (a.output/'summary.json').write_text(json.dumps(report,indent=2))
        if proc.returncode:raise RuntimeError('metadata probe failed; see '+str(a.output))
        metadata=json.loads((a.output/'streams.json').read_text())
        print(json.dumps(dict(output=str(a.output),counts={k:v['count'] for k,v in metadata['streams'].items()},
            tcp_received_bytes=sum(r.get('received_bytes_delta',0) for r in report['connections']),
            comparable_connections=sum(r['delta_status']=='comparable' for r in report['connections']),
            kernel_deltas=report['kernel_deltas'])),flush=True)
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close()


if __name__=='__main__':main()
