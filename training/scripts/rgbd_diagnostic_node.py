"""Live RGBD diagnostic node, isolated from legacy vision/control topics.

Audited geometry may be explicitly loaded: still no actionable pose publisher.
GPU inference runs in a separate Pixi process, leaving ROS in system Python.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import subprocess
import sys
import time
from capture_rgb_readonly import encode_png
from probe_rgbd_readonly import stamp,image_meta
from rgbd_localization import DiagnosticLocalization, load_audit, camera_matches, AUDIT_SHA256
from rgbd_tf_wait import ExactStampWait
from rgbd_shutdown import cleanup_steps, stop_worker

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--pixi',required=True)
    parser.add_argument('--weights',required=True)
    parser.add_argument('--seconds',type=float,default=30)
    parser.add_argument('--session-budget-seconds',type=float)
    parser.add_argument('--target',default='canned_juice')
    parser.add_argument('--camera-audit',type=Path,help='Exact reviewed report; diagnostics only')
    args=parser.parse_args()
    session_deadline=None
    if args.session_budget_seconds is not None:
        from search_timing import session_budget
        try:session_deadline=time.monotonic()+session_budget(args.session_budget_seconds)
        except ValueError as exc:parser.error(str(exc))
    audited=load_audit(args.camera_audit)
    if not 0<args.seconds<=120: parser.error('seconds must be in (0,120]')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import Image,CameraInfo
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener
    rclpy.init(); node=Node('handyman_rgbd_diagnostics')
    pub=node.create_publisher(String,'/handyman/vision/diagnostics',10)
    buffer=Buffer(); listener=TransformListener(buffer,node)
    queues={'rgb':deque(maxlen=10),'depth':deque(maxlen=10)}; infos={}; subscriptions=[]
    for kind,prefix in [('rgb','rgb'),('depth','depth_registered')]:
        topic='/hsrb/head_rgbd_sensor/'+prefix
        subscriptions.append(node.create_subscription(Image,topic+'/image_raw',lambda m,k=kind:queues[k].append(m),qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(CameraInfo,topic+'/camera_info',lambda m,k=kind:infos.update({k:m}),qos_profile_sensor_data))
    gate=DiagnosticLocalization(audited); pending=None; tf_wait=None; last=(0,0); seq=0; last_report=time.monotonic()
    def emit(value):
        value.update(actionable=False,geometry_verified=audited,target=args.target,
                     verification_scope='reviewed_snapshot_and_live_metadata' if audited else 'none',
                     audit_sha256=AUDIT_SHA256 if audited else None,position_semantics='visible_surface_not_grasp_pose')
        message=String(); message.data=json.dumps(value,allow_nan=False)
        with (args.output/'diagnostics.jsonl').open('a') as stream: stream.write(message.data+'\n')
        print(message.data,flush=True)
        if rclpy.ok():
            try: pub.publish(message)
            except Exception:
                if rclpy.ok(): raise
    log=(args.output/'worker.log').open('w')
    worker=None
    stop_reason='exception'
    try:
        worker_command=[args.pixi,'run','python',str(Path(__file__).with_name('rgbd_inference_worker.py')),'--spool',str(args.output),'--weights',args.weights]
        if session_deadline is not None:
            worker_command+=['--deadline-monotonic',str(session_deadline)]
        worker=subprocess.Popen(worker_command,cwd=args.repo,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        startup=min(time.monotonic()+120,session_deadline) if session_deadline is not None else time.monotonic()+120
        while rclpy.ok() and not (args.output/'ready.json').exists():
            rclpy.spin_once(node,timeout_sec=.05)
            if worker.poll() is not None or time.monotonic()>startup: raise RuntimeError('worker_startup_failed; see worker.log')
        print('READY live diagnostics, no control publishers',flush=True)
        deadline=session_deadline if session_deadline is not None else time.monotonic()+args.seconds
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if worker.poll() is not None:
                stop_reason='worker_exited'
                raise RuntimeError('worker_exited')
            if pending:
                folder,a,b,sent=pending
                if (folder/'result.json').exists():
                    result=json.loads((folder/'result.json').read_text())
                    selected=[d for d in result.get('detections',[]) if d['name']==args.target]
                    if tf_wait is None:
                        tf_wait=ExactStampWait(stamp(b),time.monotonic())
                    def lookup_exact(stamp_ns):
                        transform=buffer.lookup_transform('odom',b.header.frame_id,Time(nanoseconds=stamp_ns))
                        t,q=transform.transform.translation,transform.transform.rotation
                        return {'translation':[t.x,t.y,t.z],'rotation_xyzw':[q.x,q.y,q.z,q.w]}
                    age=(node.get_clock().now().nanoseconds-min(stamp(a),stamp(b)))/1e9
                    tf_status,tf,tf_error=tf_wait.poll(lookup_exact,time.monotonic(),fresh=0<=age<=2)
                    if tf_status=='waiting':
                        continue  # Next iteration spins ROS, allowing delayed TF to arrive.
                    quality=gate.evaluate(result,args.target,stamp(a),stamp(b),node.get_clock().now().nanoseconds,tf,tf_status)
                    emit(dict(sample=seq,quality=quality,age_s=age,time_delta_ms=abs(stamp(a)-stamp(b))/1e6,
                              rgb_stamp_ns=stamp(a),depth_stamp_ns=stamp(b),tf_at_depth_stamp=tf,tf_error=tf_error,
                              tf_wait_status=tf_status,tf_wait_ms=(time.monotonic()-tf_wait.started)*1000,inference=result))
                    pending=None; tf_wait=None; last_report=time.monotonic()
                elif time.monotonic()-sent>5:
                    stop_reason='inference_timeout'
                    emit({'quality':gate.reject('inference_timeout')}); break
            if not pending and all(queues.values()) and len(infos)==2:
                pairs=[(a,b) for a in queues['rgb'] for b in queues['depth'] if stamp(a)>last[0] and stamp(b)>last[1] and abs(stamp(a)-stamp(b))<=50_000_000]
                if pairs:
                    a,b=max(pairs,key=lambda pair:min(stamp(pair[0]),stamp(pair[1])))
                    last=(stamp(a),stamp(b)); ci=infos['rgb']; di=infos['depth']
                    if not camera_matches(a,b,ci,di):
                        emit({'quality':gate.reject('camera_calibration_mismatch')}); continue
                    seq+=1; folder=args.output/f'{seq:06d}'; folder.mkdir()
                    (folder/'rgb.png').write_bytes(encode_png(a)); (folder/'depth.bin').write_bytes(bytes(b.data))
                    temp=folder/'request.tmp'; temp.write_text(json.dumps({'rgb':image_meta(a),'depth':image_meta(b),'k':list(ci.k)})); temp.rename(folder/'request.json')
                    pending=(folder,a,b,time.monotonic())
            if time.monotonic()-last_report>2:
                emit({'quality':gate.reject('waiting_for_fresh_result')}); last_report=time.monotonic()
        else:
            stop_reason='deadline_reached' if time.monotonic()>=deadline else 'ros_shutdown'
    except (KeyboardInterrupt,ExternalShutdownException):
        stop_reason='interrupted_or_ros_shutdown'
    finally:
        errors=cleanup_steps([
            ('stop_record',lambda:emit({'quality':gate.reject('diagnostic_stopped'),'stop_reason':stop_reason,
                                      'monotonic_s':time.monotonic(),'session_deadline_monotonic':session_deadline})),
            ('stop_signal',lambda:(args.output/'stop').touch()),
            ('worker',lambda:stop_worker(worker)),
            ('worker_log',log.close),
            ('node',node.destroy_node),
            ('ros',lambda:rclpy.try_shutdown()),
        ])
        if errors:
            print('Cleanup errors: '+json.dumps(errors),file=sys.stderr)
            raise RuntimeError('Incomplete diagnostic cleanup; see stderr')

if __name__=='__main__': main()
