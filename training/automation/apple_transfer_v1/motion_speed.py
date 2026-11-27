"""Pure speed/duration policy. Does not import ROS or publish motion.

Unity HSRCommon limits in the supplied project: base .3m/s, base1.1rad/s,
arm lift .15m/s, arm rotation1rad/s. Use lower reviewed software envelopes.
Gripper contact timing, arrival tolerances and observation gates are unchanged.
"""
import json
import math
from pathlib import Path


PROFILE_PATH = Path(__file__).parent / 'config/motion_speed_profiles.json'
NAV_SPEED_NAMES = ('FollowPath.max_vel_x', 'FollowPath.max_speed_xy', 'FollowPath.max_vel_theta')


def load_profile(name, path=PROFILE_PATH):
    document = json.loads(Path(path).read_text())
    if document.get('version') != 1 or name not in ('baseline', 'balanced_v1', 'carry_fast_v1'):
        raise ValueError('unsupported_motion_profile')
    value = document['profiles'][name]
    if name == 'baseline':
        if value != dict(mode='legacy'):
            raise ValueError('baseline_profile_must_preserve_legacy_timing')
        return value
    limits = {
        'navigation': dict(empty_linear_mps=(.2, .28), empty_angular_rps=(.2, .4),
            holding_linear_mps=(.2, .24), holding_angular_rps=(.2, .3),
            short_linear_mps=(.2, .2), short_angular_rps=(.2, .2), short_segment_m=(1.6, 1.6)),
        'arm': dict(empty_lift_mps=(.04, .10), empty_angular_rps=(.25, .65),
            holding_lift_mps=(.04, .10), holding_angular_rps=(.25, .5),
            minimum_duration_s=(1.2, 1.2), trajectory_allowance_s=(.15, .15)),
        'finite_base': dict(empty_linear_mps=(.06, .12), holding_linear_mps=(.06, .10),
            near_linear_mps=(.04, .06), near_distance_m=(.08, .08), angular_rps=(.15, .3),
            minimum_duration_s=(1., 1.), trajectory_allowance_s=(.15, .15)),
    }
    if name=='carry_fast_v1':
        limits['navigation'].update(holding_linear_mps=(.28,.28), empty_angular_rps=(.8,.8),
            holding_angular_rps=(.8,.8), short_angular_rps=(.8,.8))
    if value.get('mode') != 'measured_duration' or set(value) != {'mode', *limits}:
        raise ValueError('invalid_motion_profile')
    for section, fields in limits.items():
        if set(value[section]) != set(fields):
            raise ValueError('invalid_motion_profile_fields')
        for key, (low, high) in fields.items():
            number = value[section][key]
            if type(number) not in (int, float) or not math.isfinite(number) or not low <= number <= high:
                raise ValueError('motion_speed_outside_reviewed_envelope:' + section + '.' + key)
    return value


def _round_up(seconds):
    return math.ceil(seconds * 20 - 1e-10) / 20


def duration_fields(seconds):
    # ROS Duration.sec is signed32-bit. Do not turn60s into a hidden distance
    # limit for an explicitly unlimited, speed-derived placement trajectory.
    if not math.isfinite(seconds) or not 0 < seconds < 2**31:
        raise ValueError('invalid_trajectory_duration')
    nanoseconds = round(seconds * 1e9)
    fields=divmod(nanoseconds, 1_000_000_000)
    if fields[0]>=2**31:raise ValueError('invalid_trajectory_duration')
    return fields


def arm_duration(profile, kind, before, goal, *, holding=False, segmented=True):
    if (kind not in ('arm', 'hand') or len(before) != 6 or len(goal) != 6
            or not all(math.isfinite(x) for x in (*before, *goal))):
        raise ValueError('invalid_arm_duration_query')
    legacy = 6. if segmented else 4.
    # Preserve actual gripper opening/closure/contact behaviour for this revision.
    if profile['mode'] == 'legacy' or kind == 'hand':
        return legacy
    p = profile['arm']
    prefix = 'holding' if holding else 'empty'
    travel = max(abs(goal[0] - before[0]) / p[prefix + '_lift_mps'],
                 *(abs(goal[i] - before[i]) / p[prefix + '_angular_rps'] for i in range(1, 5)))
    return _round_up(max(p['minimum_duration_s'], travel + p['trajectory_allowance_s']))


def finite_duration(profile, delta, *, holding=False, max_translation_m=.4):
    if max_translation_m not in (.4,.6,None):
        raise ValueError('unsupported_finite_translation_limit')
    if len(delta) != 3 or not all(math.isfinite(x) for x in delta):
        raise ValueError('invalid_finite_duration_query')
    distance = math.hypot(*delta[:2])
    if not math.isfinite(distance) or (max_translation_m is not None and distance > max_translation_m) or abs(delta[2]) > .35:
        raise ValueError('outside_existing_finite_adjustment_range')
    if profile['mode'] == 'legacy' and max_translation_m is not None:
        return 4.
    # An unlimited placement cannot inherit a fixed4s legacy duration; use the
    # same reviewed finite-base speeds, not an arbitrarily faster trajectory.
    p = profile['finite_base'] if profile['mode'] != 'legacy' else load_profile('balanced_v1')['finite_base']
    speed = p['holding_linear_mps'] if holding else p['empty_linear_mps']
    if distance <= p['near_distance_m']:
        speed = min(speed, p['near_linear_mps'])
    travel = max(distance / speed, abs(delta[2]) / p['angular_rps'])
    seconds=max(p['minimum_duration_s'], travel + p['trajectory_allowance_s'])
    if not math.isfinite(seconds) or seconds>=2**31:raise ValueError('unrepresentable_trajectory_duration')
    return _round_up(seconds)


def navigation_caps(profile, distance, *, holding=False):
    if not math.isfinite(distance) or distance < 0:
        raise ValueError('invalid_navigation_distance')
    if profile['mode'] == 'legacy':
        return None
    p = profile['navigation']
    prefix = 'short' if distance <= p['short_segment_m'] else ('holding' if holding else 'empty')
    linear, angular = p[prefix + '_linear_mps'], p[prefix + '_angular_rps']
    return [linear, linear, angular]


class NavigationSpeedLease:
    """Injectable read/atomic-write adapter; restore only our three speed limits.

Never writes goal tolerance, costmap, accelerations, routes or lifecycle state.
Call restore even after failure; a failed restore must prevent success reporting.
"""
    def __init__(self, read, atomic_write):
        self.read, self.write = read, atomic_write
        self.original = None
        self.changed = False
        self.restored = False

    def apply(self, values):
        if values is None:
            return
        if len(values) != 3 or not all(math.isfinite(x) and x > 0 for x in values):
            raise ValueError('invalid_navigation_speed_caps')
        if self.original is None:
            self.original = list(self.read())
            if len(self.original) != 3 or not all(math.isfinite(x) and x > 0 for x in self.original):
                raise ValueError('original_navigation_speeds_not_verified')
        current = self._read()
        if current != list(values):
            # Atomic write prevents partial application. Mark before calling so
            # an ambiguous timeout also triggers restoration, never a blind retry.
            self.changed = True
            self.write(values)
        if any(abs(x - y) > 1e-8 for x, y in zip(self._read(), values)):
            raise ValueError('navigation_speed_readback_failed')

    def _read(self):
        current = list(self.read())
        if len(current) != 3 or not all(math.isfinite(x) and x > 0 for x in current):
            raise ValueError('navigation_speed_readback_failed')
        return current

    def restore(self):
        if self.original is None:
            self.restored = True
            return
        if self.changed:
            self.write(self.original)
        current = self._read()
        self.restored = len(current) == 3 and all(abs(x - y) <= 1e-8 for x, y in zip(current, self.original))
        if not self.restored:
            raise ValueError('navigation_speed_restore_failed')
