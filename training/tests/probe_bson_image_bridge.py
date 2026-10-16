"""Valid synthetic BSON images through a dedicated bridge, domain 73/port 51001.

No production sockets, Unity files, coordinator, or robot command topics.
Tests two simultaneous image channels with byte-for-byte validation.
"""
from collections import defaultdict
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time


def bson(document):
    fields=[]
    for key,value in document.items():
        name=key.encode()+b'\0'
        if isinstance(value,str):
            value=value.encode();fields.append(b'\x02'+name+struct.pack('<i',len(value)+1)+value+b'\0')
        elif type(value) is int:fields.append(b'\x10'+name+struct.pack('<i',value))
        elif isinstance(value,bytes):fields.append(b'\x05'+name+struct.pack('<i',len(value))+b'\0'+value)
        elif isinstance(value,dict):fields.append(b'\x03'+name+bson(value))
        else:raise TypeError(type(value))
    body=b''.join(fields)+b'\0'
    return struct.pack('<i',len(body)+4)+body


def packet(kind,sequence,warmup=False):
    w,h=(64,64) if warmup else (640,480)
    bpp=3 if kind=='rgb' else 2
    return bson(dict(op='publish',topic='/handyman_test/'+kind,type='sensor_msgs/msg/Image',
        msg=dict(header=dict(frame_id=('warmup' if warmup else kind)+':'+str(sequence)),
            width=w,height=h,encoding='rgb8' if kind=='rgb' else '16UC1',
            is_bigendian=0,step=w*bpp,data=bytes([sequence%256])*(w*h*bpp))))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge-executable',type=Path)
    parser.add_argument('--audit',action='store_true')
    args=parser.parse_args()
    if args.bridge_executable and not args.bridge_executable.is_file():parser.error('missing executable')
    if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp':
        raise RuntimeError('Requires isolated Cyclone domain 73')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from start_rgbd_diagnostics import listening_ports
    with socket.socket() as guard:
        guard.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);guard.bind(('127.0.0.1',51001))
    out=Path(tempfile.mkdtemp(prefix='bson-image-bridge-'));print('OUTPUT',out,flush=True)
    rclpy.init();node=Node('bson_image_bridge_receiver')
    received=defaultdict(set);warm=set();errors=[];subs=[];sockets={};proc=None
    report=dict(synthetic=True,domain=73,port=51001,passed=False)
    def callback(msg,kind,qos):
        key=kind+'/'+qos
        if msg.header.frame_id.startswith('warmup:'):
            warm.add(key);return
        try:
            label,index=msg.header.frame_id.split(':');index=int(index)
            bpp=3 if kind=='rgb' else 2
            valid=(label==kind and 0<=index<20 and msg.width==640 and msg.height==480 and
                msg.step==640*bpp and msg.encoding==('rgb8' if kind=='rgb' else '16UC1') and
                msg.is_bigendian==0 and bytes(msg.data)==bytes([index])*(640*480*bpp))
            if valid:received[key].add(index)
            else:errors.append(dict(channel=key,frame=msg.header.frame_id,error='payload_mismatch'))
        except (ValueError,TypeError):errors.append(dict(channel=key,error='invalid_frame'))
    def spin_for(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if proc.poll() is not None:raise RuntimeError('test bridge exited')
            rclpy.spin_once(node,timeout_sec=.01)
    log=(out/'bridge.log').open('w')
    try:
        for kind in ('rgb','depth'):
            for label,qos in [('best_effort',qos_profile_sensor_data),('reliable',10)]:
                subs.append(node.create_subscription(Image,'/handyman_test/'+kind,
                    lambda msg,k=kind,q=label:callback(msg,k,q),qos))
        command=[str(args.bridge_executable),'51001'] if args.bridge_executable else ['ros2','run','sigverse_ros_bridge','sigverse_ros_bridge','51001']
        proc=subprocess.Popen(command,env=dict(os.environ,HANDYMAN_IMAGE_AUDIT='1' if args.audit else '0'),
            stdout=log,stderr=log,start_new_session=True)
        deadline=time.monotonic()+10
        while 51001 not in listening_ports(subprocess.check_output(['ss','-ltn'],text=True)):
            if time.monotonic()>deadline:raise RuntimeError('test bridge listen timeout')
            spin_for(.1)
        for kind in ('rgb','depth'):
            sockets[kind]=socket.create_connection(('127.0.0.1',51001),timeout=2)
            spin_for(.1)  # Separate accepts; no simultaneous connection burst.
        deadline=time.monotonic()+8;sequence=0
        while len(warm)<4 and time.monotonic()<deadline:
            for kind,sock in sockets.items():sock.sendall(packet(kind,sequence,True))
            sequence+=1;spin_for(.1)
        report['warmup_received_channels']=sorted(warm)
        if len(warm)<4:raise RuntimeError('not all channels received warmup')
        print('All four subscriptions received warmup',flush=True)
        for sequence in range(20):
            for kind,sock in sockets.items():sock.sendall(packet(kind,sequence))
            spin_for(.12)
        spin_for(2)
        report.update(sent_per_channel=20,received_unique={k:len(v) for k,v in received.items()},
            missing={k:sorted(set(range(20))-received[k]) for k in warm},errors=errors)
        report['passed']=not errors and all(len(received[k])==20 for k in warm)
    finally:
        for sock in sockets.values():sock.close()
        if proc is not None:
            try:os.killpg(proc.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                proc.wait(timeout=3)
        log.close();node.destroy_node();rclpy.try_shutdown()
        lines=[line for line in (out/'bridge.log').read_text().splitlines() if line.startswith('HANDYMAN_IMAGE_AUDIT ')]
        report['audit_enabled']=args.audit
        report['audit_lines']=len(lines)
        report['audit_expectation_passed']=(all(any('phase=after_publish' in line and '/handyman_test/'+kind+'"' in line for line in lines) for kind in ('rgb','depth')) if args.audit else not lines)
        report['passed']=report['passed'] and report['audit_expectation_passed']
        report['test_port_released']=51001 not in listening_ports(subprocess.check_output(['ss','-ltn'],text=True))
        (out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    if not report['passed'] or not report['test_port_released']:raise RuntimeError('integration not passed; see report')


if __name__=='__main__':
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    main()
