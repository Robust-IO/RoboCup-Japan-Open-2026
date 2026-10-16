"""Accepted-goal registry and bounded per-UUID cancellation. Never task drain.

Acceptance registration is not durable dispatch intent: unknown/missed goals
remain outside coverage. ROS adapter is restricted to the isolated test domain.
"""
import math
import time
import yaml
from search_process_watchdog import valid_id


class OwnedGoalRegistry:
    def __init__(self, task_id, point_id, map_sha256, allowed_poses):
        if not valid_id(task_id): raise ValueError('invalid_task')
        self.task_id, self.point_id, self.map_sha256 = task_id, point_id, map_sha256
        self.allowed = allowed_poses
        self.entries = {}
        self.generations = {}

    def register(self, row, now_ns):
        if not isinstance(row, dict) or row.get('schema') != 'handyman-owned-goal-v1':
            raise ValueError('invalid_schema')
        if (row.get('task_id'), row.get('point_id'), row.get('map_sha256')) != (
                self.task_id, self.point_id, self.map_sha256): raise ValueError('wrong_owner')
        if type(row.get('stamp_ns')) is not int or not 0 <= now_ns-row['stamp_ns'] <= 2_000_000_000:
            raise ValueError('stale_registration')
        key, generation, role, pose = row.get('goal_id'), row.get('generation'), row.get('role'), row.get('pose')
        if not valid_id(key) or type(generation) is not int or generation <= 0:
            raise ValueError('invalid_goal_identity')
        if (row.get('frame_id') != 'map' or role not in self.allowed or not isinstance(pose, dict) or
                set(pose) != {'x','y','yaw'} or not all(type(v) in (int,float) and math.isfinite(v) for v in pose.values()) or
                pose not in self.allowed[role]): raise ValueError('pose_not_in_plan_allowlist')
        entry = dict(goal_id=key, generation=generation, role=role, pose=dict(pose))
        if key in self.entries:
            if self.entries[key] != entry: raise ValueError('conflicting_uuid')
            return False
        if generation in self.generations or len(self.entries) >= 128:
            raise ValueError('generation_conflict_or_limit')
        self.entries[key] = entry
        self.generations[generation] = key
        return True


class RegisteredGoalsCanceller:
    def __init__(self, node, registry, *, topic='/handyman_test/owned_goals',
                 ack_topic='/handyman_test/owned_goal_ack',action_name='/handyman_test/navigate_to_pose',enable_live=False,journal=None):
        import os
        if not enable_live and (os.environ.get('ROS_DOMAIN_ID') != '73' or os.environ.get('ROS_LOCALHOST_ONLY') != '1' or action_name!='/handyman_test/navigate_to_pose'):
            raise ValueError('test domain only')
        from handyman_msgs.msg import HandymanMsg
        from nav2_msgs.action import NavigateToPose
        self.node, self.registry = node, registry
        from search_dispatch_journal import DispatchJournal
        self.owns_journal=journal is None
        if journal is None:
            if enable_live:raise ValueError('persistent journal required for live intent acknowledgement')
            import tempfile
            from pathlib import Path
            journal=DispatchJournal(Path(tempfile.mkdtemp(prefix='test-intent-journal-'))/'intents.sqlite3')
        self.journal=journal;self.intent_keys=set();self.failed=False
        self.action_name,self.enable_live=action_name,enable_live
        self.message_type=HandymanMsg
        self.ack_publisher=node.create_publisher(HandymanMsg,ack_topic,100)
        self.result_type = NavigateToPose.Impl.GetResultService
        self.result_client = node.create_client(self.result_type, action_name+'/_action/get_result')
        self.results = {}; self.jobs = {}; self.events = []; self.cancel = None
        self.queried = set(); self.deadline = None
        self.subscription = node.create_subscription(HandymanMsg, topic, self.on_registration, 100)
        self.timer = node.create_timer(.05, self.tick)

    def on_registration(self, msg):
        if msg.message not in ('owned_goal_intent','owned_goal_accepted','owned_goal_rejected'): return
        try:
            if msg.message=='owned_goal_intent' and (self.cancel is not None or self.failed):return
            if self.deadline is not None and time.monotonic() >= self.deadline:
                self.events.append(dict(state='registration_after_cancel_window', task_drained=False)); return
            row = yaml.safe_load(msg.detail)
            self.registry.register(row, self.node.get_clock().now().nanoseconds)
            key=row['goal_id']
            if msg.message=='owned_goal_intent':
                self.journal.record(row,self.action_name)
                self.intent_keys.add(key)
            elif msg.message=='owned_goal_rejected':
                if key not in self.intent_keys:raise ValueError('rejection_without_intent')
                self.journal.resolve(key,0,rejected=True);self.results[key]=0
        except (ValueError, TypeError, yaml.YAMLError) as exc:
            self.events.append(dict(state='registration_rejected', reason=str(exc), task_drained=False)); return
        except Exception as exc:
            self.failed=True
            self.events.append(dict(state='intent_persistence_failed',reason=str(exc),task_drained=False));return
        # ACK only after validation and in-memory insertion (or exact duplicate).
        # Echo original immutable payload; this is not a durable-storage receipt.
        ack=self.message_type();ack.message={'owned_goal_intent':'owned_goal_intent_recorded',
            'owned_goal_rejected':'owned_goal_rejected_recorded','owned_goal_accepted':'owned_goal_registered'}[msg.message]
        ack.detail=yaml.safe_dump(dict(schema='handyman-owned-goal-ack-v1',task_id=self.registry.task_id,
            goal_id=row['goal_id'],registration=msg.detail))
        self.ack_publisher.publish(ack)
        self.tick()

    def request_cancel(self, row):
        if (not isinstance(row, dict) or row.get('schema') != 'handyman-search-request-v1' or
                row.get('task_id') != self.registry.task_id or not valid_id(row.get('cancel_id'))): return False
        if self.cancel is not None: return self.cancel == row
        self.cancel = dict(row); self.deadline = time.monotonic()+3.
        self.tick(); return True

    def tick(self):
        from independent_goal_cancel import IndependentGoalCanceller
        from types import SimpleNamespace
        for key in list(self.registry.entries):
            if key not in self.queried and self.result_client.service_is_ready():
                self.queried.add(key)
                query = self.result_type.Request();query.goal_id.uuid=list(bytes.fromhex(key))
                def received(future, key=key):
                    try:
                        status=future.result().status
                        if status in (4,5,6):
                            if key in self.intent_keys:self.journal.resolve(key,status)
                            self.results[key]=status
                        else:self.queried.discard(key)
                    except Exception:
                        self.queried.discard(key)
                        self.events.append(dict(state='result_query_failed',goal_id=key,task_drained=False))
                try: self.result_client.call_async(query).add_done_callback(received)
                except Exception: self.queried.discard(key)
            if self.cancel is not None and key not in self.jobs:
                if key in self.results:
                    self.jobs[key]=None
                    self.events.append(dict(state='already_terminal',goal_id=key,status=self.results[key],task_drained=False))
                elif time.monotonic() < self.deadline:
                    if key in self.intent_keys:
                        from search_dispatch_journal import IntentGoalCanceller
                        child=IntentGoalCanceller(self.node,self.journal,dict(goal_id=key,action_name=self.action_name))
                        self.jobs[key]=child
                    else:
                        child=IndependentGoalCanceller(self.node,SimpleNamespace(task_id=self.registry.task_id),binding_topic=None,
                            action_name=self.action_name,enable_live=self.enable_live)
                        child.goal_id=key
                        self.jobs[key]=child
                        child.request_cancel(self.cancel)

    def diagnostics(self):
        return dict(registrations=list(self.registry.entries.values()), results=dict(self.results),
            cancellations={key:job.events for key,job in self.jobs.items() if job is not None},
            events=list(self.events), task_drained=False)

    def close(self):
        self.node.destroy_timer(self.timer);self.node.destroy_subscription(self.subscription)
        self.node.destroy_publisher(self.ack_publisher);self.node.destroy_client(self.result_client)
        for child in self.jobs.values():
            if child is not None:child.close()
        if self.owns_journal:self.journal.close()
