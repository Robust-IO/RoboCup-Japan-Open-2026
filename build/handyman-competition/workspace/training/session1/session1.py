"""Session1 task orchestration. Pure Python; effects are requests, not ROS calls.

One instance belongs to one live session. Runtime adapters must return current
physical evidence for requests. Offline replay never authorizes competition IO.
"""
from dataclasses import dataclass, asdict
import uuid

INSTRUCTION = 'Go to the living room, grasp the dog_doll and put it on the square_low_table.'


@dataclass(frozen=True)
class Task:
    layout: str = 'LayoutA'
    target: str = 'dog_doll'
    room: str = 'living_room'
    destination: str = 'square_low_table#living_room'
    destination_room: str = 'living_room'
    source_table: str | None = None


class Session1:
    """Effect consumer owns motion and protocol publication, including cancellation.

    No target coordinates are taken from Unity task answers. Empty search results
    do not automatically become Does_not_exist. Holding uncertainty stops progress.
    """
    STEPS = ('navigate_room', 'search', 'grasp', 'tuck',
             'navigate_destination', 'place', 'verify_placement')
    TERMINAL = ('succeeded', 'failed', 'stopped', 'cancelled')

    def __init__(self, session_id, *, run_id=None, source_table=None, profile=None):
        if not isinstance(session_id, str) or not session_id.strip():
            raise ValueError('session_id_required')
        self.session_id = session_id
        self.run_id = run_id or uuid.uuid4().hex
        if profile is None and source_table not in (None, 'white_side_table#living_room'):
            raise ValueError('unsupported_session1_source_table')
        if profile is not None:
            source_table=profile['source']['furniture_id']
            self.task=Task(layout=profile['layout'],target=profile['target'],room=profile['source']['room'],
                           destination=profile['destination']['furniture_id'],destination_room=profile['destination']['room'],source_table=source_table)
            room=self.task.room.replace('_',' ');destination=self.task.destination.split('#')[0]
            prep='in' if profile['destination']['mode']=='container_drop' else 'on'
            self.instruction=profile.get('expected_instruction') or f'Go to the {room}, grasp the {self.task.target} and put it {prep} the {destination}.'
        else:
            self.task = Task(source_table=source_table)
            self.instruction=INSTRUCTION
        self.steps = (('navigate_room', 'navigate_source', 'observe_target',
                       'grasp', 'tuck', 'navigate_destination', 'place', 'verify_placement')
                      if source_table else self.STEPS)
        self.state = 'waiting_ready'
        self.environment = None
        self.ready_requested = self.preflight_ok = False
        self.pending = None
        self.sequence = 0
        self.effects = []
        self.reason = None
        self.evidence = {}

    def emit(self, kind, **payload):
        self.effects.append(dict(kind=kind, session_id=self.session_id,
                                 run_id=self.run_id, **payload))

    def drain(self):
        effects, self.effects = self.effects, []
        return effects

    def stop(self, reason, state='stopped'):
        if self.state in self.TERMINAL:
            return
        pending = self.pending
        self.pending = None
        self.state, self.reason = state, reason
        # Discard queued actions/publications; never allow a cancelled task to
        # publish Task_finished or start an operation that has not been sent.
        self.effects.clear()
        self.emit('stop', operation_id=pending, reason=reason,
                  preserve_grip=True)

    def give_up(self, reason):
        # Called only after the operation port has retired all motion.
        if self.state in self.TERMINAL or self.state == 'waiting_give_up':return
        self.pending=None;self.reason=reason;self.effects.clear()
        self.state='waiting_give_up'
        self.emit('protocol',message='Give_up',detail='')

    def _ready(self):
        if (self.state == 'waiting_ready' and self.ready_requested
                and self.preflight_ok and self.environment == self.task.layout):
            self.state = 'waiting_instruction'
            self.emit('protocol', message='I_am_ready', detail='')

    def preflight(self, *, ready):
        # Adapter checks current localization/sensors/control ownership. This
        # function does not inspect ROS and a saved checkpoint is not preflight.
        self.preflight_ok = ready is True
        self._ready()

    def moderator(self, message, detail='', *, session_id):
        if session_id != self.session_id or self.state in self.TERMINAL:
            return
        if message in ('Task_failed', 'Mission_complete'):
            self.stop(detail or message, 'failed' if message == 'Task_failed' else 'cancelled')
            return
        if message == 'Environment':
            self.environment = detail
            if detail != self.task.layout:
                self.stop('unexpected_layout')
            else:
                self._ready()
        elif message == 'Are_you_ready?':
            self.ready_requested = True
            self._ready()
            if self.state == 'waiting_instruction' and not self.effects:
                self.emit('protocol', message='I_am_ready', detail='')
        elif message in ('Instruction', 'Corrected_instruction'):
            # Fixed Session1 profile, deliberately no general language parser.
            normalized = lambda value: ' '.join(value.lower().split()).rstrip('.')
            if message == 'Corrected_instruction' or normalized(detail) != normalized(self.instruction):
                self.stop('instruction_not_session1')
            elif self.state == 'waiting_instruction':
                self._request('navigate_room')
            # Repeated identical instruction never repeats navigation/grasp.
        elif message == 'Task_succeeded':
            if self.state == 'waiting_moderator':
                self.state = 'succeeded'
            else:
                self.stop('success_before_verified_placement')

    def _request(self, stage):
        self.sequence += 1
        self.pending = f'{self.run_id}/{self.sequence}'
        self.state = stage
        self.emit('operation', operation_id=self.pending, stage=stage,
                  task=asdict(self.task), evidence=dict(self.evidence),
                  strategy=('measured_irregular_body' if stage == 'grasp' else None))

    def result(self, operation_id, result, *, session_id, run_id):
        if (session_id != self.session_id or run_id != self.run_id
                or operation_id != self.pending or self.pending is None):
            return
        stage = self.state
        if result.get('status') != 'ok':
            self.stop(result.get('reason') or f'{stage}_incomplete')
            return
        required = {
            'navigate_room': result.get('room') == self.task.room and result.get('arrived') is True,
            'navigate_source': (result.get('source_table') == self.task.source_table
                                and result.get('room') == self.task.room
                                and result.get('arrived') is True),
            'observe_target': (result.get('target') == self.task.target
                               and result.get('room') == self.task.room
                               and result.get('source_table') == self.task.source_table
                               and result.get('confirmed') is True
                               and bool(result.get('evidence_ref'))),
            'search': (result.get('target') == self.task.target and result.get('room') == self.task.room
                       and result.get('confirmed') is True and bool(result.get('evidence_ref'))),
            'grasp': result.get('target') == self.task.target and result.get('grasp_verified') is True,
            'tuck': result.get('holding_verified') is True and result.get('carry_ready') is True,
            'navigate_destination': result.get('destination') == self.task.destination and result.get('arrived') is True,
            'place': result.get('release_sent') is True,
            'verify_placement': (result.get('target') == self.task.target
                                 and result.get('destination') == self.task.destination
                                 and (result.get('placement_verified') is True or
                                      (result.get('completion_actions_confirmed') is True
                                       and result.get('release_sent') is True
                                       and (result.get('basis')=='gripper_release_completed' or
                                            (result.get('cleanup_completed') is True and
                                             result.get('basis')=='release_withdraw_and_fold_completed'))))),
        }
        if not required.get(stage, False):
            self.stop(f'{stage}_evidence_missing_or_mismatched')
            return
        self.pending = None
        self.evidence[stage] = dict(result)
        event = {'navigate_room': 'Room_reached', 'grasp': 'Object_grasped',
                 'verify_placement': 'Task_finished'}.get(stage)
        if event:
            self.emit('protocol', message=event, detail='')
        if stage == 'verify_placement':
            self.state = 'waiting_moderator'
        else:
            self._request(self.steps[self.steps.index(stage) + 1])

    def snapshot(self):
        return dict(session_id=self.session_id, run_id=self.run_id, state=self.state,
                    task=asdict(self.task), pending=self.pending, reason=self.reason,
                    evidence=self.evidence)
