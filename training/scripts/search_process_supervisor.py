"""ROS adapter for one explicitly owned worker. No launch, restart or kill API.

The caller supplies a SearchProcessWatchdog for its child. It must retain this
adapter and spin a single-threaded ROS node until task retirement is resolved.
Cannot certify task drain or robot standstill. An optional isolated-test exact
goal canceller may be supplied; it receives only matched fault cancellations.
"""
import yaml
from handyman_msgs.msg import HandymanMsg
from rclpy.qos import QoSProfile, DurabilityPolicy


class SearchProcessSupervisor:
    def __init__(self, node, watchdog, *, request_topic, status_topic, goal_canceller=None,
                 lease_test=False,retire_topic=None,retirement_check=None,lease_topics=None):
        self.watchdog = watchdog
        self.goal_canceller = goal_canceller
        self.drain=None;self.retirement_blocked=False
        self.retirement_check=retirement_check
        self.retire_publisher=None
        if retire_topic is not None:
            if not callable(retirement_check):raise ValueError('retirement check required')
            self.retire_publisher=node.create_publisher(HandymanMsg,retire_topic,10)
            self.status_subscription=node.create_subscription(HandymanMsg,status_topic,self.on_status,10)
        self.publisher = node.create_publisher(HandymanMsg, status_topic, 10)
        self.subscription = node.create_subscription(
            HandymanMsg, request_topic, self.on_request,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = node.create_timer(.1, self.tick)
        if lease_test or lease_topics:
            import os
            if lease_test and (os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):
                raise ValueError('lease adapter is test domain only')
            lease_topics=lease_topics or ('/handyman_test/lease_request','/handyman_test/lease_reply_raw')
            self.lease_publisher=node.create_publisher(HandymanMsg,lease_topics[1],10)
            self.lease_subscription=node.create_subscription(
                HandymanMsg,lease_topics[0],self.on_lease,10)
        self.node=node

    def close(self):
        self.node.destroy_timer(self.timer)
        for name in ('subscription','status_subscription','lease_subscription'):
            item=getattr(self,name,None)
            if item is not None:self.node.destroy_subscription(item)
        for name in ('publisher','retire_publisher','lease_publisher'):
            item=getattr(self,name,None)
            if item is not None:self.node.destroy_publisher(item)

    def on_lease(self, message):
        if message.message!='supervisor_lease_request':return
        try:row=yaml.safe_load(message.detail)
        except yaml.YAMLError:return
        if (not isinstance(row,dict) or row.get('schema')!='handyman-supervisor-lease-v1' or
            row.get('task_id')!=self.watchdog.task_id or type(row.get('challenge')) is not int or
            not 0<row['challenge']<2**64 or self.watchdog.sample() is not None or
            self.watchdog.cancel_id is not None):return
        reply=HandymanMsg();reply.message='supervisor_lease_reply';reply.detail=message.detail
        self.lease_publisher.publish(reply)

    def on_status(self, message):
        try:row=yaml.safe_load(message.detail)
        except yaml.YAMLError:return
        if not isinstance(row,dict) or row.get('task_id')!=self.watchdog.task_id:return
        if message.message=='search_worker_fault' or row.get('state')=='cancel_failed':
            self.retirement_blocked=True
        elif message.message=='search_cancel_status' and row.get('state')=='cancel_drained':
            self.drain=row

    def on_request(self, message):
        if message.message != 'search_cancelled':
            return
        try:
            row = yaml.safe_load(message.detail)
        except yaml.YAMLError:
            return
        if self.watchdog.cancellation(row):
            if self.goal_canceller is not None and self.watchdog.sample() is not None:
                self.goal_canceller.request_cancel(row)
            self.tick()

    def tick(self):
        if self.retire_publisher is not None and not self.retirement_blocked and self.drain is not None:
            try:can_retire=self.retirement_check()
            except Exception:can_retire=False
            if can_retire and self.watchdog.prepare_retirement(self.drain) and not self.watchdog.retired:
                msg=HandymanMsg();msg.message='search_retire'
                msg.detail=yaml.safe_dump(dict(schema='handyman-search-retire-v1',
                    task_id=self.watchdog.task_id,cancel_id=self.watchdog.cancel_id))
                self.retire_publisher.publish(msg)
        notice = self.watchdog.fault_notice()
        if notice is None:
            return
        report = self.watchdog.failure_report()
        # Cancellation may arrive before the process fault. Reconcile when the
        # fault is observed too; the canceller deduplicates the same cancel_id.
        if report is not None and self.goal_canceller is not None:
            self.goal_canceller.request_cancel(dict(
                schema='handyman-search-request-v1', task_id=report['task_id'],
                cancel_id=report['cancel_id']))
        message = HandymanMsg()
        message.message = 'search_cancel_status' if report is not None else 'search_worker_fault'
        message.detail = yaml.safe_dump(report if report is not None else notice)
        self.publisher.publish(message)
