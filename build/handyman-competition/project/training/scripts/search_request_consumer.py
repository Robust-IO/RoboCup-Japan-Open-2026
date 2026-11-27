"""Read-only request preparation. File consistency is not proof of Nav2's live map."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
import yaml
from build_search_plan import build


def beneath(root,relative):
    if not isinstance(relative,str) or not relative:
        raise ValueError('invalid_resource_path')
    path=(root/relative).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError('resource_outside_package_or_missing')
    return path


def prepare(row,share):
    config=share/'config'
    catalog=yaml.safe_load((config/'environments.yaml').read_text())
    path=beneath(config,catalog['environment_files'][row['environment']])
    content=path.read_bytes();env=yaml.safe_load(content)
    if row['layout']!=env['internal_name'] or row['map_uri']!=env['map'] or row['frame_id']!='map':
        raise ValueError('environment_or_map_mismatch')
    plan=build(env)
    room=next((r for r in plan['rooms'] if r['room']==row['room']),None)
    if room is None or not room['valid']:
        raise ValueError('invalid_room_search_plan')
    points=[dict(id=p['id'],pose=p['pose']) for p in room['points']]
    incoming=row['points']
    if not isinstance(incoming,list) or len(incoming)!=len(points) or any(
        a['id']!=b['id'] or any(not math.isfinite(a['pose'][k]) or
            abs(a['pose'][k]-b['pose'][k])>1e-9 for k in ('x','y','yaw'))
        for a,b in zip(incoming,points)):
        raise ValueError('search_points_mismatch')
    if not isinstance(row['target'],str) or not row['target']:
        raise ValueError('invalid_target')
    prefix='package://handyman_rebuild_ros2/'
    if not env['map'].startswith(prefix):raise ValueError('unexpected_map_package')
    map_path=beneath(share,env['map'][len(prefix):])
    meta_bytes=map_path.read_bytes();meta=yaml.safe_load(meta_bytes)
    image_path=beneath(share,str((map_path.parent/meta['image']).relative_to(share)))
    image_bytes=image_path.read_bytes()
    # Detect edits during preparation; caller must revalidate immediately before execution.
    if content!=path.read_bytes() or meta_bytes!=map_path.read_bytes() or image_bytes!=image_path.read_bytes():
        raise ValueError('map_files_changed_during_validation')
    return dict(task_id=row['task_id'],target=row['target'],room=row['room'],points=points,
        environment_sha256=hashlib.sha256(content).hexdigest(),
        environment_path=str(path),
        map_bundle_sha256=hashlib.sha256(meta_bytes+b'\0'+image_bytes).hexdigest(),
        map_path=str(map_path),state='prepared_readonly',actionable=False,
        live_map_verified=False,requires_live_map_verification=True)


class RequestConsumer:
    def __init__(self,share,request_lifetime=30.):
        if not math.isfinite(request_lifetime) or not 0<request_lifetime<=600:
            raise ValueError('invalid_request_lifetime')
        self.request_lifetime=request_lifetime
        self.share=Path(share).resolve();self.active=None;self.retired=set();self.expires=None

    def receive(self,event,row,sensor_now,now):
        if not isinstance(row,dict) or row.get('schema')!='handyman-search-request-v1':
            raise ValueError('invalid_request_schema')
        key=row.get('task_id')
        if not isinstance(key,str) or uuid.UUID(key).hex!=key or key=='0'*32:
            raise ValueError('invalid_task_id')
        if event=='search_cancelled':
            if self.active and self.active['task_id']==key:
                self.active=None;self.expires=None;self.retired.add(key)
                return dict(state='cancelled',task_id=key,actionable=False)
            # Remember cancellation received before its request to prevent resurrection.
            if len(self.retired)>=1000:raise ValueError('retired_request_limit')
            self.retired.add(key)
            return dict(state='cancel_recorded',task_id=key,actionable=False)
        if event!='search_requested':raise ValueError('unknown_request_event')
        stamp=row.get('stamp_ns')
        if type(stamp) is not int or not 0<=sensor_now-stamp<=2_000_000_000:
            raise ValueError('stale_request')
        if row.get('actionable') is not False or row.get('requires_map_verification') is not True:
            raise ValueError('unexpected_request_authority')
        if key in self.retired or len(self.retired)>=1000:raise ValueError('retired_request')
        if self.active is not None:raise ValueError('request_already_active')
        result=prepare(row,self.share)
        self.active=result;self.expires=now+self.request_lifetime
        return dict(result)

    def tick(self,now):
        if self.active and now>=self.expires:
            key=self.active['task_id'];self.retired.add(key);self.active=None;self.expires=None
            return dict(state='expired',task_id=key,actionable=False)
        return None


class RequestMapGate:
    """No navigation permission. A mismatch retires this request rather than auto-resuming."""
    def __init__(self,consumer):
        self.consumer=consumer;self.expected=None;self.task_id=None;self.publisher=None

    def invalidate(self,reason):
        active=self.consumer.active
        self.expected=None;self.task_id=None;self.publisher=None
        if active is None:return None
        key=active['task_id'];self.consumer.retired.add(key)
        self.consumer.active=None;self.consumer.expires=None
        return dict(state='map_verification_revoked',task_id=key,reason=reason,
                    live_map_verified=False,actionable=False)

    def install(self,task_id,expected,digest):
        active=self.consumer.active
        if active is None or active['task_id']!=task_id:return False
        if digest!=active['map_bundle_sha256']:raise ValueError('decoded_map_digest_changed')
        self.expected=expected;self.task_id=task_id;self.publisher=None
        return True

    def receive_map(self,msg,publisher,known_publishers):
        from verify_live_map import compare
        active=self.consumer.active
        if active is None or active['task_id']!=self.task_id or self.expected is None:return None
        if len(known_publishers)!=1 or publisher not in known_publishers:
            return self.invalidate('map_publisher_missing_or_ambiguous')
        if self.publisher is not None and publisher!=self.publisher:
            return self.invalidate('map_publisher_changed')
        result=compare(self.expected,msg)
        if not result['matches']:return self.invalidate(result['reason'])
        self.publisher=publisher
        if active.get('live_map_verified'):return None
        active.update(state='map_content_verified',live_map_verified=True,
            requires_live_map_verification=False,costmap_verified=False,actionable=False)
        return dict(active)

    def check(self,known_publishers):
        from verify_live_map import bundle
        active=self.consumer.active
        if active is None:return None
        if self.publisher is not None and known_publishers!=[self.publisher]:
            return self.invalidate('map_publisher_changed_or_missing')
        try:
            if (bundle(Path(active['map_path']))!=active['map_bundle_sha256'] or
                hashlib.sha256(Path(active['environment_path']).read_bytes()).hexdigest()!=active['environment_sha256']):
                return self.invalidate('local_map_or_environment_changed')
        except (OSError,ValueError,KeyError,TypeError,yaml.YAMLError):
            return self.invalidate('local_map_files_unavailable')
        return None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package-share',type=Path)
    p.add_argument('--topic',default='/handyman/search/request')
    p.add_argument('--decoder',type=Path,help='Enable received-map verification using Nav2 file decoder')
    p.add_argument('--map-topic',default='/map')
    p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not 0<a.seconds<=120:p.error('seconds must be in (0,120]')
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from handyman_msgs.msg import HandymanMsg
    from ament_index_python.packages import get_package_share_directory
    consumer=RequestConsumer(a.package_share or get_package_share_directory('handyman_rebuild_ros2'))
    gate=RequestMapGate(consumer)
    pool=ThreadPoolExecutor(max_workers=1) if a.decoder else None
    job=None;job_task=None;latest_map=None
    with a.output.open('x') as stream:
        rclpy.init();node=rclpy.create_node('handyman_search_request_readonly')
        def emit(row):
            row.update(actionable=False,read_only=True)
            stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
        def receive(msg):
            try:
                emit(consumer.receive(msg.message,yaml.safe_load(msg.detail),node.get_clock().now().nanoseconds,time.monotonic()))
            except (ValueError,TypeError,KeyError,AttributeError,OSError,yaml.YAMLError) as exc:
                emit(dict(state='rejected',reason=str(exc)))
        def publishers():
            return sorted(bytes(p.endpoint_gid).hex() for p in node.get_publishers_info_by_topic(a.map_topic))
        def receive_map(msg):
            nonlocal latest_map
            known=publishers()
            # Humble rclpy supplies no MessageInfo here. This is a graph snapshot,
            # not authenticated per-message publisher provenance.
            latest_map=(msg,known[0] if len(known)==1 else None)
            row=gate.receive_map(*latest_map,known)
            if row:emit(row)
        try:
            sub=node.create_subscription(HandymanMsg,a.topic,receive,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            if a.decoder:
                from nav_msgs.msg import OccupancyGrid
                map_sub=node.create_subscription(OccupancyGrid,a.map_topic,receive_map,
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            end=time.monotonic()+a.seconds
            next_check=0.
            while rclpy.ok() and time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.05)
                row=consumer.tick(time.monotonic())
                if row:emit(row)
                if pool:
                    if job is not None and job.done():
                        try:
                            expected,digest=job.result()
                            if gate.install(job_task,expected,digest) and latest_map:
                                row=gate.receive_map(*latest_map,publishers())
                                if row:emit(row)
                        except Exception as exc:
                            if consumer.active and consumer.active['task_id']==job_task:
                                row=gate.invalidate('map_decode_failed: '+str(exc))
                                if row:emit(row)
                        job=None
                    if consumer.active and gate.task_id!=consumer.active['task_id'] and job is None:
                        from verify_live_map import decode
                        job_task=consumer.active['task_id']
                        job=pool.submit(decode,Path(consumer.active['map_path']),a.decoder.resolve())
                    if time.monotonic()>=next_check:
                        next_check=time.monotonic()+.5
                        row=gate.check(publishers())
                        if row:emit(row)
        except (KeyboardInterrupt,ExternalShutdownException):pass
        finally:
            consumer.active=None
            emit(dict(state='consumer_stopped'))
            node.destroy_node();rclpy.try_shutdown()
            if pool:pool.shutdown(wait=False,cancel_futures=True)

if __name__=='__main__':main()
