"""Offline arrival gate. Caller supplies synchronized map pose and odometry.

No ROS commands. A matching FINAL search-point goal success is required, not
room-entry success. Sensor timestamps and monotonic time are separate clocks.
"""
import math
from search_vision_adapter import SearchVisionAdapter


class SearchArrival:
    def __init__(self, scan, goal_id):
        request = scan.request()
        if scan.phase != 'navigate' or not goal_id:
            raise ValueError('navigation_not_ready')
        self.scan = scan
        self.goal_id = goal_id
        self.view_id = request['view_id']
        self.goal = request['pose']
        self.success_stamp = None
        self.last_stamp = 0
        self.last_now = None
        self.since = None
        self.anchor = None
        self.adapter = None

    def result(self, goal_id, succeeded, stamp_ns):
        if goal_id != self.goal_id or self.scan.phase != 'navigate':
            return False
        # An accepted outcome is immutable; delayed callbacks cannot restart it.
        if self.success_stamp is not None:
            return False
        if succeeded is not True or type(stamp_ns) is not int or stamp_ns <= 0:
            self.reset()
            return False  # Failure is not evidence of an absent object.
        self.success_stamp = stamp_ns
        return True

    def reset(self):
        self.since = None
        self.anchor = None

    def sample(self, *, pose, linear_speed, angular_speed, stamp_ns,
               sensor_now_ns, now, frame_id='map'):
        def reject(reason):
            self.reset()
            return dict(self.scan.status(), arrival_reason=reason)
        if not math.isfinite(now):
            return reject('invalid_time')
        self.scan.tick(now)
        if self.scan.phase != 'navigate' or self.scan.request().get('view_id') != self.view_id:
            return reject('view_not_navigating')
        if self.success_stamp is None:
            return reject('waiting_for_matching_goal_success')
        try:
            x, y, yaw = (pose[k] for k in ('x', 'y', 'yaw'))
            if frame_id != 'map' or not all(math.isfinite(v) for v in (x,y,yaw,linear_speed,angular_speed)):
                return reject('invalid_pose_or_speed')
            if not all(type(v) is int for v in (stamp_ns,sensor_now_ns)):
                return reject('invalid_stamp')
            if stamp_ns <= max(self.last_stamp,self.success_stamp) or not 0 <= sensor_now_ns-stamp_ns <= 500_000_000:
                return reject('stale_or_duplicate_pose')
            previous_stamp = self.last_stamp
            self.last_stamp = stamp_ns
            if self.last_now is not None and (now <= self.last_now or now-self.last_now > .5 or stamp_ns-previous_stamp > 500_000_000):
                self.reset()
            self.last_now = now
            yaw_error = abs(math.atan2(math.sin(yaw-self.goal['yaw']),math.cos(yaw-self.goal['yaw'])))
            if math.hypot(x-self.goal['x'],y-self.goal['y']) > .15 or yaw_error > math.radians(10):
                return reject('outside_search_pose_tolerance')
            if abs(linear_speed) > .02 or abs(angular_speed) > .03:
                return reject('robot_still_moving')
            if self.anchor is not None:
                ax,ay,ayaw = self.anchor
                if math.hypot(x-ax,y-ay) > .02 or abs(math.atan2(math.sin(yaw-ayaw),math.cos(yaw-ayaw))) > .03:
                    self.reset()
            if self.since is None:
                self.since = (now,stamp_ns)
                self.anchor = (x,y,yaw)
            if now-self.since[0] < 1. or stamp_ns-self.since[1] < 1_000_000_000:
                return dict(self.scan.status(),arrival_reason='settling')
            self.scan.arrived(self.view_id,True,True,now)
            self.adapter = SearchVisionAdapter(self.scan,self.view_id,stamp_ns)
            return dict(self.scan.status(),arrival_reason='arrived_and_settled',arrival_stamp_ns=stamp_ns)
        except (KeyError,TypeError,ValueError,OverflowError):
            return reject('malformed_pose')
