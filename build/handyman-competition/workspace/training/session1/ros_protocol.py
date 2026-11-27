"""Real Handyman ROS protocol binding. Construct only in the owning ROS process.

Importing this module does not initialize ROS. Call attach only after installing
all real stage handlers. Wire messages have no session ID: a Unity restart must
retire the old Application and create a new one, not reuse its operation state.
"""
from concurrent.futures import Future

TO_ROBOT='/handyman/message/to_robot'
TO_MODERATOR='/handyman/message/to_moderator'
INCOMING={'Environment','Are_you_ready?','Instruction','Corrected_instruction',
          'Task_succeeded','Task_failed','Mission_complete'}
OUTGOING={'I_am_ready','Room_reached','Object_grasped','Task_finished','Does_not_exist','Give_up'}

class RosProtocol:
    def __init__(self,node,session_id,*,message_type=None):
        if message_type is None:
            from handyman_msgs.msg import HandymanMsg as message_type
        self.node=node;self.session_id=session_id;self.message_type=message_type
        self.application=None;self.subscription=None;self.timer=None;self.closed=False
        self.publisher=node.create_publisher(message_type,TO_MODERATOR,10)

    def attach(self,application):
        if self.closed or self.application is not None:raise ValueError('protocol_already_bound_or_closed')
        if application.session.session_id!=self.session_id:raise ValueError('protocol_session_mismatch')
        self.application=application
        self.subscription=self.node.create_subscription(self.message_type,TO_ROBOT,self.receive,10)
        self.timer=self.node.create_timer(.02,application.tick)

    def receive(self,message):
        if self.closed or self.application is None:return
        if message.message not in INCOMING:
            self.node.get_logger().warning('Ignoring unknown Moderator message: '+str(message.message))
            return
        self.node.get_logger().info('Moderator: '+message.message+' '+message.detail)
        if message.message=='Mission_complete' or (message.message=='Task_failed' and message.detail=='Time_is_up'):
            self.application.official_simulation_end=dict(message=message.message,detail=message.detail,session_id=self.session_id)
        self.application.receive(message.message,message.detail,session_id=self.session_id)

    def publish(self,effect):
        future=Future()
        try:
            if (self.closed or self.application is None or effect.get('session_id')!=self.session_id
                    or effect.get('run_id')!=self.application.session.run_id):
                raise ValueError('inactive_protocol_run')
            if effect.get('message') not in OUTGOING:raise ValueError('unsupported_session1_outgoing_message')
            message=self.message_type();message.message=effect['message'];message.detail=effect.get('detail','')
            self.publisher.publish(message)
            self.node.get_logger().info('Robot: '+message.message+' '+message.detail)
            # This confirms publish() returned, not Unity receipt/scoring.
            future.set_result(dict(status='sent',moderator_acknowledged=False))
        except Exception as exc:future.set_exception(exc)
        return future

    def close(self):
        if self.application is not None and not self.application.executor.retired:
            raise RuntimeError('retire_motion_before_closing_protocol')
        self.closed=True
        if self.timer is not None:self.node.destroy_timer(self.timer);self.timer=None
        if self.subscription is not None:self.node.destroy_subscription(self.subscription);self.subscription=None
        if self.publisher is not None:self.node.destroy_publisher(self.publisher);self.publisher=None


def bind_application(node,session_id,handlers,request_stop,*,message_type=None,profile=None):
    """All stage handlers and real stop confirmation must be supplied explicitly."""
    from application import Application
    protocol=RosProtocol(node,session_id,message_type=message_type)
    try:
        app=Application(session_id,handlers,protocol.publish,request_stop,profile=profile)
        protocol.attach(app)
    except Exception:
        if protocol.application is not None:protocol.application.close()
        protocol.close()
        raise
    return app,protocol
