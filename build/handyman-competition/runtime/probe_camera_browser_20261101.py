"""Read two image messages from the EXISTING bridge. Only subscribe/unsubscribe."""
import base64,json,os,socket,struct,time
from pathlib import Path

topics=['/hsrb/head_rgbd_sensor/rgb/image_raw','/hsrb/hand_camera/image_raw']
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/camera-browser-viewer-20261101')
root.mkdir(exist_ok=True)
report=dict(robot_commands_sent=False,ros_services_started_or_stopped=False,frames=[])
sock=None;buffer=bytearray()
def take(count):
    while len(buffer)<count:
        block=sock.recv(max(4096,min(65536,count-len(buffer))))
        if not block:raise ConnectionError('bridge_closed')
        buffer.extend(block)
    result=bytes(buffer[:count]);del buffer[:count];return result
def send(value,opcode=1):
    raw=json.dumps(value).encode() if opcode==1 else value
    mask=os.urandom(4);size=len(raw)
    header=bytes([128|opcode,128|size]) if size<126 else bytes([128|opcode,128|126])+struct.pack('!H',size)
    sock.sendall(header+mask+bytes(v^mask[i%4] for i,v in enumerate(raw)))
try:
    sock=socket.create_connection(('127.0.0.1',9090),timeout=3)
    key=base64.b64encode(os.urandom(16)).decode()
    request='GET / HTTP/1.1\r\nHost: 127.0.0.1:9090\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: '+key+'\r\nSec-WebSocket-Version: 13\r\nOrigin: null\r\n\r\n'
    sock.sendall(request.encode())
    while b'\r\n\r\n' not in buffer:
        block=sock.recv(4096)
        if not block:raise ConnectionError('handshake_closed')
        buffer.extend(block)
    headers,rest=bytes(buffer).split(b'\r\n\r\n',1);buffer[:]=rest
    if b' 101 ' not in headers.split(b'\r\n')[0]:raise ValueError('websocket_handshake_rejected')
    for index,topic in enumerate(topics):send(dict(op='subscribe',id='readonly-camera-check-'+str(index),topic=topic,throttle_rate=500,queue_length=1,compression='none'))
    deadline=time.monotonic()+10;seen=set();parts=bytearray()
    while time.monotonic()<deadline and len(seen)<2:
        first,second=take(2);opcode=first&15;length=second&127
        if length==126:length=struct.unpack('!H',take(2))[0]
        elif length==127:length=struct.unpack('!Q',take(8))[0]
        if length>8_000_000:raise ValueError('oversized_bridge_frame')
        mask=take(4) if second&128 else None
        raw=take(length)
        if mask:raw=bytes(v^mask[i%4] for i,v in enumerate(raw))
        if opcode==9:send(raw,10);continue
        if opcode==8:raise ConnectionError('bridge_sent_close')
        if opcode not in (0,1):continue
        if opcode==1:parts.clear()
        parts.extend(raw)
        if not first&128:continue
        event=json.loads(parts);parts.clear()
        if event.get('op')=='status':report.setdefault('bridge_status',[]).append(event)
        if event.get('op')!='publish' or event.get('topic') not in topics:continue
        topic=event['topic']
        if topic in seen:continue
        msg=event['msg'];data=msg['data']
        count=len(base64.b64decode(data)) if isinstance(data,str) else len(data)
        row=dict(topic=topic,width=msg['width'],height=msg['height'],encoding=msg['encoding'],
                 step=msg['step'],data_bytes=count,data_format='base64' if isinstance(data,str) else 'array')
        if count<msg['step']*msg['height']:raise ValueError('incomplete_image_payload')
        report['frames'].append(row);seen.add(topic)
    report['status']='both_live_camera_streams_verified' if len(seen)==2 else 'camera_messages_incomplete'
except Exception as exc:report.update(status='camera_bridge_read_failed',reason=str(exc))
finally:
    if sock:
        try:
            for index,topic in enumerate(topics):send(dict(op='unsubscribe',id='readonly-camera-check-'+str(index),topic=topic))
            send(b'',8)
        except Exception:pass
        sock.close()
    (root/'bridge-check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
