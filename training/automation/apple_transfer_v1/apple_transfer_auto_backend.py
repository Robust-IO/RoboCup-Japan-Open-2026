"""Live adapter for the reviewed LayoutA apple transfer, using existing trials.

Every child action is journaled before launch. No action is automatically replayed;
cleanup stops owned observation/child processes, never releases or returns the arm.
Finite receiver-side trajectories may finish after interruption (not cancellable).
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import tempfile

import numpy as np
from PIL import Image
import yaml

from apple_approach_lease import measured_target
from apple_transfer_auto_plan import (fresh_snapshot, map_point, map_pose, plan_grasp,
                                     plan_placement, placement_step)
from apple_transfer_automation import write_json
from hand_camera_ray import rotation
from inverse_table_place_plan import FLEX, OFFSET, WRIST
from tracked_apple_surface import measure as measure_held
from verify_apple_lift_sequence import measured, palm_in_base
from latest_capture_index import latest as latest_capture


def read(path):
    return json.loads(Path(path).read_text())


def stop_group(process):
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


class LiveBackend:
    mode = 'live'

    def __init__(self, config, input_audit=None):
        self.config = config
        self.input_audit = input_audit
        self.code = Path(__file__).parent
        sys.path.insert(0, str(self.code/'scripts'))
        self.environment = yaml.safe_load(Path(config['environment']).read_text())
        semantic = read(config['semantic_map'])
        self.tables = {x['id']: x for x in semantic['destinations']}
        self.observer = None
        self.observer_log = None
        self.active_child = None
        self.raw_observer = False
        self.serial = 0
        self.inference_service = None
        self.inference_log = None
        self.inference_socket = None
        self.inference_directory = None

    @property
    def efficient(self):
        return self.config['policy'].get('efficient_non_navigation', False)

    def start_model_service(self):
        if not self.efficient:
            return
        if os.environ.get('HANDYMAN_RESIDENT_MODEL')=='1':
            from resident_inference import connect
            worker=connect(self.config['model_python'],self.config['registry'],imgsz=960,
                confirm_apple_roi=self.config['policy'].get('near_grasp_roi_confirmation',False))
            self.inference_socket=worker.endpoint
            self.record(dict(event='resident_inference_attached',**worker.ready,robot_commands_sent=False))
            worker.close();return
        self.inference_directory = tempfile.TemporaryDirectory(prefix='hm-apple-model-')
        self.inference_socket = Path(self.inference_directory.name)/'worker.sock'
        args = [sys.executable, str(self.code/'apple_shared_inference.py'),
                '--socket', str(self.inference_socket), '--run-root', str(self.output),
                '--python', self.config['model_python'], '--registry', self.config['registry']]
        if self.config['policy'].get('near_grasp_roi_confirmation', False):
            args.append('--confirm-apple-roi')
        self.inference_log = (self.output/'shared-model.log').open('x')
        self.inference_service = subprocess.Popen(args, stdout=self.inference_log,
            stderr=subprocess.STDOUT, start_new_session=True)
        self.record(dict(event='inference_service_start', pid=self.inference_service.pid,
                         robot_commands_sent=False, warmup_overlaps_navigation=True))

    def bind(self, output, task_id, record):
        self.output, self.task_id, self.record = output, task_id, record

    def evidence(self, name, value):
        path = self.output / (name + '.json')
        if path.exists():
            raise ValueError('evidence_already_exists:' + name)
        write_json(path, value)
        return dict(verified=True, evidence=str(path))

    def command(self, arguments, result_path, expected_key=None, expected_value=None, timeout=None):
        self.serial += 1
        command = [sys.executable, *map(str, arguments)]
        self.record(dict(event='command_intent', stage=self.stage, sequence=self.serial, command=command))
        with (self.output / f'command-{self.serial:03d}.log').open('x') as log:
            self.active_child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = self.active_child.wait(timeout=timeout)
            except BaseException:
                stop_group(self.active_child)
                self.active_child = None
                raise
        try:
            result = read(result_path) if result_path is not None else {}
            if code != 0 or expected_key is not None and result.get(expected_key) != expected_value:
                raise RuntimeError('child_action_failed:' + str(result_path) + ':' + str(result.get('reason', result.get('error', result.get('status', code)))))
        except BaseException:
            stop_group(self.active_child)
            raise
        finally:
            self.active_child = None
        self.record(dict(event='command_confirmed', stage=self.stage, sequence=self.serial, evidence=str(result_path)))
        return result

    def capture(self, name):
        path = self.output / name
        args = [self.code/'capture_grasp_scene.py', path, '--hand-geometry', '--map-geometry',
                '--tracking-epoch', self.task_id+'-'+name, '--duration', '7',
                '--interval', '.6' if self.efficient else '1']
        if self.efficient:
            final = name == 'placed-final'
            args += ['--stop-after-frames', '3' if final else '1',
                     '--min-frame-span', '1.2' if final else '0']
        self.command(args,
                     path/'manifest.json', timeout=25)
        frames = sorted(path.glob('*/snapshot.json'))
        if not frames:
            raise RuntimeError('no_current_capture:' + name)
        return frames[-1].parent

    def start_observer(self, name, infer):
        if self.observer is not None:
            raise RuntimeError('observer_already_owned')
        self.recording = self.output / name
        self.epoch = self.task_id + '-' + name
        self.raw_observer = not infer
        command = [sys.executable, str(self.code/'capture_grasp_scene.py'), str(self.recording),
                   '--hand-geometry', '--map-geometry', '--tracking-epoch', self.epoch,
                   '--duration', str(self.config['policy']['observer_duration_s']), '--interval', '.1', '--compact-progress']
        if infer:
            command += ['--inference-python', self.config['model_python'], '--registry', self.config['registry'],
                        '--inference-imgsz', '960', '--environment-yaml', self.config['environment'],
                        '--expected-layout', 'LayoutA', '--target-room', 'bedroom', '--identity-tracking-task', self.task_id]
            if getattr(self, 'inference_socket', None) is not None:
                if self.inference_service.poll() is not None:
                    raise RuntimeError('owned_model_service_exited')
                command += ['--inference-socket', str(self.inference_socket)]
            if self.config['policy'].get('near_grasp_roi_confirmation',False):
                command += ['--confirm-apple-roi']
            if self.config['policy'].get('near_grasp_geometry_handoff',False):
                command += ['--near-apple-geometry','--near-apple-gap-m',
                            str(self.config['policy']['near_grasp_geometry_gap_m'])]
        self.record(dict(event='observation_start', stage=self.stage, command=command, robot_commands_sent=False))
        self.observer_log = (self.output/(name+'.log')).open('x')
        self.observer = subprocess.Popen(command, stdout=self.observer_log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop_observer(self):
        if self.observer is not None:
            stop_group(self.observer)
            self.observer = None
        if self.observer_log is not None:
            self.observer_log.close()
            self.observer_log = None

    def latest(self, timeout=6):
        end = time.monotonic() + timeout
        reason = 'no_observation'
        while time.monotonic() < end:
            if self.observer is None or self.observer.poll() is not None:
                raise RuntimeError('owned_observer_not_running:' + reason)
            try:
                paths = [latest_capture(self.recording, 'snapshot.json' if self.raw_observer else 'observation.json')]
            except FileNotFoundError:
                paths = []
            if paths:
                try:
                    capture = paths[-1].parent
                    snapshot = read(capture/'snapshot.json')
                    evidence, observation = None, None
                    if not self.raw_observer:
                        observation = read(capture/'observation.json')
                    # Disk reads must precede the decision; never renew a lease
                    # merely by checking the old snapshot before reading results.
                    fresh_snapshot(snapshot, time.time_ns(), self.epoch)
                    if not self.raw_observer:
                        tf = snapshot['tracking_transforms']['odom_from_base_at_depth']
                        r = rotation(tf['quaternion'])
                        odom = [*tf['translation'][:2], math.atan2(r[1, 0], r[0, 0])]
                        evidence = measured_target(observation, snapshot, now_ns=time.time_ns(), epoch=self.epoch,
                                                   task_id=self.task_id, current_odom=odom, expected_room='bedroom')
                    return capture, snapshot, observation, evidence
                except (ValueError, KeyError, FileNotFoundError) as exc:
                    reason = str(exc)
            time.sleep(.02)
        raise RuntimeError('fresh_observation_wait_expired:' + reason)

    def route(self, source, destination):
        if source == destination:
            return []
        selected = next((r for r in self.environment['routes'] if r['from'] == source and r['to'] == destination), None)
        if selected is None:
            raise ValueError('configured_room_route_missing')
        return [[p['x'], p['y'], p['yaw']] for p in selected['waypoints']]

    def room_at(self, pose):
        from scripts.search_room_gate import inside, boundary_distance
        owners = [name for name, room in self.environment['rooms'].items()
                  if inside(*pose[:2], room['region']) and boundary_distance(*pose[:2], room['region']) > .15]
        if len(owners) != 1:
            raise ValueError('base_room_not_confirmed')
        return owners[0]

    def navigate(self, source, destination, goal, opening, name, *, remaining_waypoints=None):
        result_path = self.output / (name+'.json')
        args = [self.code/'navigate_carry_trial.py', '--run', '--goal', *goal,
                '--holding-opening', opening, '--verify-current-precision',
                '--motion-profile', self.config.get('navigation_motion_profile',self.config.get('motion_profile','baseline')), '--output', result_path]
        if self.config.get('navigation_joint_feedback_max_age_s',.7)!=.7:
            args+=['--joint-feedback-max-age-s',self.config['navigation_joint_feedback_max_age_s']]
        if self.config.get('navigation_feedback_recovery',False):args.append('--recover-feedback-interruption')
        for waypoint in (self.route(source, destination) if remaining_waypoints is None else remaining_waypoints):
            args += ['--waypoint', *waypoint]
        result = self.command(args, result_path, 'status', 'arrived')  # no navigation time limit
        if (result.get('nav_reset') is not True or result.get('goal_precision_changed') is not False
                or result.get('speed_parameters_restored') is not True or result.get('grip_commands_sent') is not False):
            raise ValueError('navigation_completion_or_ownership_not_verified')
        return dict(verified=True, evidence=str(result_path))

    def arm(self, name, pose, extra, timeout=60):
        path = self.output / name
        if self.efficient:
            extra = [*extra, '--efficient-transfer']
        result = self.command([self.code/'pregrasp_raised_trial.py', '--run', '--segmented-high',
                               '--simulation-contact-trial', '--reviewed-map-pose', *pose,
                               '--motion-profile', self.config.get('motion_profile','baseline'),
                               *extra, '--output', path], path/'result.json', 'status', 'passed', timeout)
        return result, str(path/'result.json')

    def adjust(self, name, pose, delta, arm, opening, *, single_placement_approach=False):
        path = self.output/(name+'.json')
        args = [self.code/'reviewed_carry_adjustment.py', '--run', '--simulation-ignore-collision',
                      '--reviewed-map-pose', *pose, '--delta', *delta, '--expected-arm', *arm,
                      '--motion-profile', self.config.get('motion_profile','baseline'),
                      '--holding-opening', opening, '--output', path]
        if self.config.get('joint_feedback_max_age_s',.4)!=.4:
            args+=['--joint-feedback-max-age-s',self.config['joint_feedback_max_age_s']]
        if single_placement_approach:args.append('--single-placement-approach')
        if self.efficient:args.append('--efficient-transfer')
        # The placement child derives its finite observation window from the
        # actual distance/speed. A fixed25s parent becomes a hidden metre cap.
        self.command(args, path, 'status', 'endpoint_verified', None if single_placement_approach else 25)
        return str(path)

    def held_measurement(self, capture, snapshot):
        expected = palm_in_base(snapshot) + self.held['center_minus_palm_m']
        value = measure_held(np.asarray(Image.open(capture/'rgb.png').convert('RGB')),
                             np.load(capture/'depth_m.npy', allow_pickle=False), snapshot, expected)
        value['center_map'] = map_point(snapshot, np.array(value['center_base'])).tolist()
        value['source_stamp_ns'] = snapshot['streams']['depth']['stamp_ns']
        return value

    def perform(self, stage):
        self.stage = stage
        return getattr(self, stage)()

    def preflight(self):
        path = self.output/'preflight.json'
        args = [self.code/'inspect_transfer_ready.py', '--output', path]
        if self.efficient:args.append('--efficient-transfer')
        self.ready = self.command(args, path, 'ready', True, 25)
        if self.input_audit is not None:
            self.evidence('input-files-audit', self.input_audit)
        self.start_room = self.room_at(self.ready['base_map_pose'])
        if self.start_room not in ('living_room', 'bedroom'):
            raise ValueError('start_outside_reviewed_source_route')
        self.initial_opening = .65 if self.ready['initial_hand_open'] else 0.
        self.start_model_service()
        return dict(verified=True, evidence=str(path))

    def navigate_source(self):
        return self.navigate(self.start_room, 'bedroom', self.config['source_staging_goal'], self.initial_opening, 'source-navigation')

    def head_view(self):
        path = self.output/'head-view'
        self.command([self.code/'scripts/head_view_trial.py', '--run', '--confirm-head-motion', '--pan', '0',
                      '--tilt', '-.5', '--output', path], path/'summary.json', 'passed', True, 25)
        return dict(verified=True, evidence=str(path/'summary.json'))

    def observe_source(self):
        self.start_observer('source-observe', True)
        capture, snapshot, observation, evidence = self.latest(110)
        self.grasp_plan = plan_grasp(observation, snapshot, self.tables[self.config['source_table_id']], evidence, self.config)
        self.grasp_plan['capture'] = str(capture)
        return self.evidence('measured-grasp-plan', self.grasp_plan)

    def prepare_grasp(self):
        _, snapshot, _, _ = self.latest()
        flags = ['--white-table-prepare-lift', self.grasp_plan['contact_start_lift_m'], '--bedroom-side-table-grasp']
        if self.ready['initial_hand_open']:
            flags += ['--prepare-hand-already-open']
        _, evidence = self.arm('grasp-prepare', map_pose(snapshot), flags)
        return dict(verified=True, evidence=evidence)

    def grasp_and_lift(self):
        path = self.output/'grasp'
        args = [self.code/'run_bedroom_apple_regrasp.py', '--run', '--output', path,
                               '--model-python', self.config['model_python'], '--registry', self.config['registry'],
                               '--environment', self.config['environment'], '--observer-recording', self.recording,
                               '--observer-epoch', self.epoch, '--task-id', self.task_id,
                               '--motion-profile', self.config.get('motion_profile','baseline'),
                               '--contact-start-lift', self.grasp_plan['contact_start_lift_m']]
        if self.config['policy'].get('post_lift_rgbd_follow_verification',False):
            args.append('--verify-preclose-rgbd-follow')
        if self.efficient:args.append('--efficient-transfer')
        # Children own bounded sensor/geometry/action waits. A second 140s wall
        # clock used to preempt their valid sequence and obscure its true cause.
        result = self.command(args,path/'result.json', 'grasp_success_verified', True)
        return dict(verified=True, evidence=str(path/'result.json'))

    def verify_grasp(self):
        from run_apple_close_lift_trial import lift_allowed
        trial = read(self.output/'grasp/close-lift/result.json')
        independent=self.config['policy'].get('post_lift_rgbd_follow_verification',False)
        if independent:
            from audit_preclose_identity_lift_follow import audit
            if trial.get('effective_grasp_success_verified') is not True:
                raise ValueError('independent_lift_evidence_not_verified')
            verification=audit(self.output/'grasp/close-lift',expected_recording=self.recording,
                               expected_epoch=self.epoch,expected_task_id=self.task_id)
            self.evidence('backend-independent-lift-verification',verification)
        else:
            verification = read(self.output/'grasp/close-lift/visual-lift-verification.json')
        if (not lift_allowed(trial['close']) or verification.get('grasp_success_verified') is not True
                or trial['lift'].get('status') != 'passed'):
            raise ValueError('contact_and_multiframe_lift_not_verified')
        capture = Path(verification['rows'][-1]['capture'])
        if independent:
            snapshot=read(capture/'snapshot.json')
            candidate=verification['rows'][-1]['center_measurement']
        else:
            _, snapshot, candidate = measured(capture)
        center = np.array(candidate['center_m'])
        palm = palm_in_base(snapshot)
        self.held = dict(center_minus_palm_m=(center-palm).tolist(),
                         bottom_below_palm_m=float(palm[2]-center[2]+candidate['radius_m']),
                         radius_m=candidate['radius_m'], holding_opening_rad=trial['close']['closure_evidence']['observed_opening_rad'],
                         verified_capture=str(capture), new_identity_verified=False)
        self.holding = self.held['holding_opening_rad']
        return self.evidence('verified-held-geometry', self.held)

    def retract_holding(self):
        # Use raw capture after stopping the inference observer: a held target has
        # correctly left its support and must not be required to pass support gates.
        self.stop_observer()
        capture = self.capture('before-carry-retract')
        _, evidence = self.arm('carry-retract', map_pose(read(capture/'snapshot.json')),
                               ['--measured-contact-start-lift', self.grasp_plan['contact_start_lift_m'],
                                '--bedroom-side-table-grasp', '--contact-finish', 'carry', '--holding-opening', self.holding])
        return dict(verified=True, evidence=evidence)

    def navigate_destination(self):
        return self.navigate('bedroom', 'living_room', self.config['destination_staging_goal'], self.holding, 'destination-navigation')

    def observe_surface(self):
        from measure_destination_surface import measure
        self.surface_capture = self.capture('destination-outside')
        self.surface_snapshot = read(self.surface_capture/'snapshot.json')
        self.surface_depth = np.load(self.surface_capture/'depth_m.npy', allow_pickle=False)
        self.plane = measure(self.surface_depth, self.surface_snapshot, self.tables[self.config['destination_id']])
        return self.evidence('destination-surface', self.plane)

    def plan_placement(self):
        self.place_plan = plan_placement(self.surface_depth, self.surface_snapshot, self.tables[self.config['destination_id']],
                                         self.plane, self.held, self.config, self.task_id)
        # Horizontal prepared apple remains outside the actual observed front.
        from pregrasp_geometry import expected_horizontal_center
        predicted = expected_horizontal_center(self.place_plan['palm_height_m'], self.held['center_minus_palm_m'])
        mapped = map_point(self.surface_snapshot, predicted)
        table_front = min(self.plane['observed_xy_min'][1],
                          self.place_plan['footprint_center'][1]-self.place_plan['footprint_half_size'][1])
        if mapped[1] + max(self.held['radius_m'], .08) >= table_front - .03:
            raise ValueError('horizontal_preparation_not_outside_table')
        self.place_plan['predicted_apple_before_approach_map_m'] = mapped.tolist()
        self.place_plan_path = self.output/'placement-plan.json'
        return self.evidence('placement-plan', self.place_plan)

    def prepare_placement(self):
        _, evidence = self.arm('place-prepare', map_pose(self.surface_snapshot),
                               ['--inverse-place-stage', 'prepare', '--place-palm-height', self.place_plan['palm_height_m'],
                                '--holding-opening', self.holding])
        self.start_observer('placement-observe', False)
        return dict(verified=True, evidence=evidence)

    def approach_placement(self):
        distance=0.;steps=[]
        single=self.config['policy'].get('placement_approach_mode','segmented')=='single'
        observations=2 if single else 4
        for index in range(observations):
            capture, snapshot, _, _ = self.latest(20 if index==0 else 6)
            measurement = self.held_measurement(capture, snapshot)
            step = placement_step(measurement,snapshot,self.place_plan,single_move=single)
            if step['aligned']:
                return self.evidence('placement-approach-sequence',dict(steps=steps,total_translation_m=distance,
                    final_measurement=measurement,endpoint_verified=True,automatic_retry=False,
                    approach_mode='single' if single else 'segmented'))
            if index==observations-1:
                raise ValueError('single_placement_endpoint_not_aligned' if single else 'placement_segment_budget_exhausted')
            delta=step['delta_m'];distance+=math.hypot(*delta[:2])
            if not single and distance>.6:raise ValueError('placement_total_distance_exceeded')
            self.evidence(f'prepared-held-geometry-{index:02d}',dict(measurement,planned_body_delta_m=delta))
            evidence=self.adjust(f'placement-approach-{index:02d}',map_pose(snapshot),delta,
                [self.place_plan['palm_height_m']-OFFSET,FLEX,0.,WRIST,0.],self.holding,
                **({'single_placement_approach':True} if single else {}))
            steps.append(dict(evidence=evidence,delta_m=delta))

    def check_release(self):
        from table_release_gate import validate
        capture, snapshot, _, _ = self.latest()
        names = ('arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint', 'hand_motor_joint')
        value = validate(capture, self.place_plan, now_ns=time.time_ns(), epoch=self.epoch, task_id=self.task_id,
                         current_map_pose=map_pose(snapshot), current_joints=[snapshot['joints'][k] for k in names])
        return self.evidence('pre-release-check', value)

    def release(self):
        _, snapshot, _, _ = self.latest()
        result, evidence = self.arm('place-release', map_pose(snapshot),
                                    ['--inverse-place-stage', 'release', '--place-palm-height', self.place_plan['palm_height_m'],
                                     '--holding-opening', self.holding, '--release-recording', self.recording,
                                     '--release-plan', self.place_plan_path, '--release-epoch', self.epoch,
                                     '--release-task-id', self.task_id])
        if 'release_geometry_evidence' not in result:
            raise ValueError('actual_release_publish_boundary_not_verified')
        self.prior_release = result['release_geometry_evidence']
        return dict(verified=True, evidence=evidence)

    def withdraw(self):
        try:
            return self._withdraw_actions()
        except Exception as exc:
            # No more motion after a failed cleanup action. Only read-only
            # placement verification may follow; release is never replayed.
            self.stop_observer()
            result=self.evidence('withdrawal-incomplete',dict(cleanup_required=True,
                reason=str(exc),error_type=type(exc).__name__,robot_ready_for_next_task=False,automatic_retry=False,
                next_action='read_only_object_placement_verification'))
            return dict(result,verified=False,deferred_to_object_verification=True,
                        cleanup_required=True,reason=str(exc))

    def _withdraw_actions(self):
        _, snapshot, _, _ = self.latest()
        self.arm('place-raise', map_pose(snapshot),
                 ['--inverse-place-stage', 'raise', '--place-palm-height', self.place_plan['palm_height_m'], '--holding-opening', self.holding])
        _, snapshot, _, _ = self.latest()
        arm_names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint')
        self.adjust('placement-retreat', map_pose(snapshot), [-self.config['policy']['retreat_m'], 0., 0.],
                    [snapshot['joints'][k] for k in arm_names], snapshot['joints']['hand_motor_joint'])
        _, snapshot, _, _ = self.latest()
        _, evidence = self.arm('place-retract', map_pose(snapshot),
                               ['--inverse-place-stage', 'retract', '--place-palm-height', self.place_plan['palm_height_m'], '--holding-opening', self.holding])
        self.stop_observer()
        return dict(verified=True, evidence=evidence)

    def verify_placement(self):
        from verify_table_placement import verify
        capture = self.capture('placed-final')
        frames = sorted(capture.parent.glob('*/snapshot.json'))
        if len(frames) < 3:
            raise ValueError('three_final_observations_required')
        selected = [frames[0].parent, frames[len(frames)//2].parent, frames[-1].parent]
        path = self.output/'placed-final-audit.json'
        args = [self.code/'inspect_table_apple.py', *selected, '--registry', self.config['registry'],
                      '--python', self.config['model_python'], '--semantic-map', self.config['semantic_map'],
                      '--destination-id', self.config['destination_id'], '--plane', self.output/'destination-surface.json',
                      '--output', path]
        if getattr(self, 'inference_socket', None) is not None:
            args += ['--inference-socket', self.inference_socket]
        self.command(args, path, timeout=120)
        actions = {k: read(self.output/f'place-{k}/result.json') if (self.output/f'place-{k}/result.json').exists()
                   else dict(status='not_completed') for k in ('release', 'raise', 'retract')}
        result = verify(read(path), self.prior_release, actions, self.config['destination_id'], self.config['destination_room'],
                        placement_plan=self.place_plan)
        evidence = self.evidence('placement-verification', result)
        if result.get('placement_success_verified') is not True:
            raise ValueError('final_placement_inconclusive')
        return dict(evidence,robot_ready_for_next_task=result['robot_ready_for_next_task'])

    def close(self):
        stop_group(self.active_child)
        self.active_child = None
        self.stop_observer()
        stop_group(getattr(self, 'inference_service', None))
        self.inference_service = None
        if getattr(self, 'inference_log', None) is not None:
            self.inference_log.close()
            self.inference_log = None
        if getattr(self, 'inference_directory', None) is not None:
            self.inference_directory.cleanup()
            self.inference_directory = None
