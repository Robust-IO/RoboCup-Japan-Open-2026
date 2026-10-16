"""Bridge existing RequestMapGate results to challenge-bound execution evidence.

Caller owns map subscription/decoder and request lifecycle. This does not turn
the read-only request into production navigation authorization.
"""
import yaml


class SearchMapResponder:
    def __init__(self,node,consumer,gate,publishers,*,request_topic,reply_topic):
        from handyman_msgs.msg import HandymanMsg
        self.node,self.consumer,self.gate,self.publishers=node,consumer,gate,publishers
        self.message_type=HandymanMsg
        self.publisher=node.create_publisher(HandymanMsg,reply_topic,10)
        self.subscription=node.create_subscription(HandymanMsg,request_topic,self.receive,10)

    def receive(self,msg):
        if msg.message!='map_check_request':return
        try:
            row=yaml.safe_load(msg.detail)
            if (not isinstance(row,dict) or row.get('schema')!='handyman-map-check-v1' or
                type(row.get('challenge')) is not int or not 0<row['challenge']<2**64):return
            # Recheck files and current publisher graph for every response.
            import time
            self.consumer.tick(time.monotonic())
            self.gate.check(self.publishers())
            active=self.consumer.active
            if active is None:
                if row.get('task_id') not in self.consumer.retired:return
                state='revoked'
            else:
                if (row.get('task_id')!=active['task_id'] or
                    row.get('map_sha256')!=active['map_bundle_sha256'] or
                    row.get('point_id') not in [p['id'] for p in active['points']]):return
                state='verified' if active.get('live_map_verified') else 'pending'
            reply=self.message_type();reply.message='map_check_reply'
            reply.detail=yaml.safe_dump(dict(row,state=state));self.publisher.publish(reply)
        except (ValueError,TypeError,KeyError,OSError,yaml.YAMLError):
            # No reply: client's previous proof expires. Never fabricate success.
            return
