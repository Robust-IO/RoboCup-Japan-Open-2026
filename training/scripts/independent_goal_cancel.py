"""Test-domain-only, exact-UUID cancellation owned outside the worker process.

One validated FINAL search-point binding. Never claims task drain or standstill.
No cancel-all, new goal, restart, or automatic coordinator unlock operation.
"""
import os
import time
import yaml
from search_process_watchdog import valid_id


class ExactCancelEvidence:
    def __init__(self, goal_id, timeout=2., *, clock=time.monotonic):
        if not valid_id(goal_id):
            raise ValueError('nonzero canonical goal UUID required')
        self.goal_id = goal_id
        self.clock = clock
        self.deadline = clock()+timeout
        self.accepted = False
        self.cancelled = False
        self.state = 'waiting'

    def poll(self):
        if self.state == 'waiting' and self.clock() >= self.deadline:
            self.state = 'timeout'
        return self.state

    def acknowledgement(self, returncode, ids):
        if self.poll() != 'waiting': return
        if returncode != 0 or self.goal_id not in ids:
            self.state = 'rejected_or_uuid_mismatch'
        else:
            self.accepted = True
            self.finish()

    def terminal(self, status):
        if self.poll() != 'waiting': return
        if status != 5:  # action_msgs/GoalStatus.STATUS_CANCELED
            self.state = 'not_cancelled_terminal'
        else:
            self.cancelled = True
            self.finish()

    def finish(self):
        if self.accepted and self.cancelled: self.state = 'goal_cancel_confirmed'


class IndependentGoalCanceller:
    def __init__(self, node, expected, *, binding_topic,
                 action_name='/handyman_test/navigate_to_pose',enable_live=False):
        if not enable_live and (os.environ.get('ROS_DOMAIN_ID') != '73' or
                os.environ.get('ROS_LOCALHOST_ONLY') != '1' or
                action_name != '/handyman_test/navigate_to_pose'):
            raise ValueError('isolated domain 73 / localhost / test Nav2 only')
        from action_msgs.srv import CancelGoal
        from nav2_msgs.action import NavigateToPose
        from handyman_msgs.msg import HandymanMsg
        self.node, self.expected = node, expected
        self.cancel_type = CancelGoal
        self.result_type = NavigateToPose.Impl.GetResultService
        self.cancel_client = node.create_client(CancelGoal, action_name+'/_action/cancel_goal')
        self.result_client = node.create_client(self.result_type, action_name+'/_action/get_result')
        self.goal_id = None
        self.cancel_id = None
        self.evidence = None
        self.sent = False
        self.finished = False
        self.events = []
        # Existing acceptance publisher is reliable/volatile. Start this monitor
        # before worker dispatch; it cannot reconstruct bindings missed on startup.
        self.subscription = (node.create_subscription(HandymanMsg, binding_topic, self.on_binding, 10)
            if binding_topic is not None else None)
        self.timer = node.create_timer(.05, self.tick)

    def emit(self, state):
        self.events.append(dict(state=state, task_id=self.expected.task_id,
            goal_id=self.goal_id, cancel_id=self.cancel_id, task_drained=False))

    def close(self):
        self.node.destroy_timer(self.timer)
        if self.subscription is not None:self.node.destroy_subscription(self.subscription)
        self.node.destroy_client(self.cancel_client);self.node.destroy_client(self.result_client)

    def on_binding(self, msg):
        if msg.message != 'search_goal_accepted' or self.cancel_id is not None: return
        from search_arrival_ros import validate_binding
        try:
            binding = validate_binding(yaml.safe_load(msg.detail), self.expected,
                self.node.get_clock().now().nanoseconds)
            if self.goal_id is None:
                self.goal_id = binding['goal_id']
                self.emit('binding_recorded')
            elif self.goal_id != binding['goal_id']:
                self.emit('additional_binding_not_covered')
        except (ValueError, TypeError, AttributeError, yaml.YAMLError):
            self.emit('binding_rejected')

    def request_cancel(self, row):
        if (not isinstance(row, dict) or row.get('schema') != 'handyman-search-request-v1' or
                row.get('task_id') != self.expected.task_id or not valid_id(row.get('cancel_id'))):
            return False
        if self.cancel_id is not None: return self.cancel_id == row['cancel_id']
        self.cancel_id = row['cancel_id']
        if self.goal_id is None:
            self.finished = True
            self.emit('no_verified_goal')
            return False
        self.evidence = ExactCancelEvidence(self.goal_id)
        self.emit('exact_cancel_requested')
        self.tick()
        return True

    def tick(self):
        if self.finished or self.evidence is None: return
        state = self.evidence.poll()
        if state != 'waiting':
            self.finished = True
            self.emit(state)
            return
        if self.sent or not (self.cancel_client.service_is_ready() and self.result_client.service_is_ready()):
            return
        self.sent = True
        cancel = self.cancel_type.Request()
        cancel.goal_info.goal_id.uuid = list(bytes.fromhex(self.goal_id))
        # Zero stamp + nonzero UUID means this exact goal only.
        assert cancel.goal_info.stamp.sec == 0 and cancel.goal_info.stamp.nanosec == 0
        result = self.result_type.Request()
        result.goal_id.uuid = list(bytes.fromhex(self.goal_id))
        try:
            self.cancel_client.call_async(cancel).add_done_callback(self.on_ack)
            self.result_client.call_async(result).add_done_callback(self.on_result)
        except Exception:
            self.evidence.state = 'service_error'

    def on_ack(self, future):
        try:
            response = future.result()
            self.evidence.acknowledgement(response.return_code,
                [bytes(g.goal_id.uuid).hex() for g in response.goals_canceling])
        except Exception:
            if self.evidence.poll() == 'waiting': self.evidence.state = 'service_error'
        self.tick()

    def on_result(self, future):
        try:
            self.evidence.terminal(future.result().status)
        except Exception:
            if self.evidence.poll() == 'waiting': self.evidence.state = 'service_error'
        self.tick()
