"""Offline active-search ledger. Proposals only; never authorizes robot motion.

Adapters must verify live arrival, sensor freshness and geometry. A completed
ring is not room coverage. No absence or grasp authorization is emitted here.
"""
import math
from dataclasses import dataclass, field


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def ring_views(x, y, yaw, fov_deg=59.99, overlap_deg=15.):
    if not all(finite(v) for v in (x, y, yaw, fov_deg, overlap_deg)):
        raise ValueError('nonfinite_view_configuration')
    if not 0 < overlap_deg < fov_deg < 180:
        raise ValueError('invalid_overlap')
    count = math.ceil(360 / (fov_deg - overlap_deg))
    # With a nominal 60-degree camera, eight 45-degree steps have ~15-degree
    # overlap; use the exact measured FOV to compute the actual overlap.
    if abs(fov_deg - 60) < .1 and overlap_deg == 15.:
        count = 8
    if count > 72:
        raise ValueError('excessive_view_count')
    return [dict(id=f'ring/{i}', x=x, y=y,
                 yaw=math.atan2(math.sin(yaw + i * 2 * math.pi / count),
                                math.cos(yaw + i * 2 * math.pi / count)),
                 overlap_deg=fov_deg - 360 / count,
                 safety_verified=False) for i in range(count)]


@dataclass
class Candidate:
    id: str
    position: tuple | None
    support: str
    scores: dict = field(default_factory=dict)
    views: set = field(default_factory=set)
    state: str = 'pending'
    attempted_views: set = field(default_factory=set)

    @property
    def score(self):
        values = sorted(self.scores.values())
        return values[len(values)//2] if values else 0.


class ActiveSearch:
    """One task/map/room ledger, fed only by validated adapter observations.

    merge_radius is a conservative association gate, not a probability model.
    Nearby instances in the same frame and ambiguous associations stay separate.
    """
    def __init__(self, task_id, map_id, room, target, views, regions,
                 deadline, low=.15, high=.8, merge_radius=.12):
        if not all(isinstance(v, str) and v for v in (task_id, map_id, room, target)):
            raise ValueError('missing_identity')
        if not all(finite(v) for v in (deadline, low, high, merge_radius)):
            raise ValueError('invalid_parameters')
        if not 0 <= low < high <= 1 or merge_radius <= 0:
            raise ValueError('invalid_thresholds')
        if not views or len(set(views)) != len(views):
            raise ValueError('invalid_views')
        if len(set(regions)) != len(regions):
            raise ValueError('duplicate_regions')
        self.identity = (task_id, map_id, room)
        self.target, self.deadline = target, deadline
        self.low, self.high, self.merge_radius = low, high, merge_radius
        self.views = {v: 'pending' for v in views}
        self.regions = {r: 'unseen' for r in regions}
        self.candidates = {}
        self.seen = set()
        self.last_frame = {}
        self.state = 'searching'

    def observe(self, identity, view, stamp, now, healthy, detections):
        if self.state != 'searching':
            return []
        if not finite(now):
            raise ValueError('invalid_time')
        if now >= self.deadline:
            self.state = 'incomplete'
            return []
        if tuple(identity) != self.identity or view not in self.views:
            return []
        if healthy is not True or type(stamp) is not int or stamp <= 0:
            return []
        if stamp <= self.last_frame.get(view, 0):
            return []
        self.last_frame[view] = stamp
        touched = []
        for d in detections:
            score = d.get('score')
            if d.get('label') != self.target or not finite(score) or not self.low <= score <= 1:
                continue
            instance = d.get('instance_id')
            if not isinstance(instance, str) or not instance:
                continue
            key = (stamp, instance)
            if key in self.seen:
                continue
            self.seen.add(key)
            p = d.get('position')
            if p is not None:
                if not isinstance(p, (list, tuple)) or len(p) != 3 or not all(finite(v) for v in p):
                    continue
                p = tuple(p)
            support = d.get('support', '')
            matches = [c for c in self.candidates.values()
                       if p is not None and c.position is not None
                       and c.support == support and stamp not in c.scores
                       and math.dist(c.position, p) <= self.merge_radius]
            # Without geometry only associate within this uninterrupted view.
            if p is None:
                matches = [c for c in self.candidates.values()
                           if c.position is None and c.id == f'bearing/{view}/{instance}']
            if len(matches) == 1:
                c = matches[0]
            else:
                cid = f'object/{len(self.candidates)}' if p else f'bearing/{view}/{instance}'
                c = Candidate(cid, p, support)
                self.candidates[cid] = c
            c.scores[stamp] = score
            c.views.add(view)
            touched.append(c.id)
        return touched

    def finish_view(self, view, usable):
        if view not in self.views:
            raise ValueError('unknown_view')
        if self.state == 'searching':
            self.views[view] = 'observed' if usable is True else 'invalid'

    def mark_region(self, region, evidence):
        if region not in self.regions:
            raise ValueError('unknown_region')
        if evidence not in ('unseen', 'too_far', 'occluded', 'invalid', 'reviewed'):
            raise ValueError('invalid_region_evidence')
        if self.state == 'searching':
            self.regions[region] = evidence

    def verify(self, candidate_id, viewpoint, outcome):
        if outcome not in ('confirmed', 'rejected', 'occluded', 'depth_invalid', 'navigation_failed'):
            raise ValueError('invalid_verification_outcome')
        c = self.candidates[candidate_id]
        if outcome == 'confirmed' and c.position is None:
            raise ValueError('confirmation_requires_geometry')
        if self.state != 'searching' or c.state in ('confirmed', 'rejected'):
            return False
        if viewpoint in c.attempted_views:
            return False
        c.attempted_views.add(viewpoint)
        if outcome in ('confirmed', 'rejected'):
            c.state = outcome
            if outcome == 'confirmed':
                self.state = 'found'
        else:
            c.state = 'deferred'
        return True

    def next_action(self, now):
        if not finite(now):
            raise ValueError('invalid_time')
        if now >= self.deadline and self.state == 'searching':
            self.state = 'incomplete'
        base = dict(actionable=False, does_not_exist_authorized=False)
        if self.state != 'searching':
            return dict(base, action=self.state)
        pending = sorted((c for c in self.candidates.values()
                          if c.state == 'pending' and len(c.scores) >= 3),
                         key=lambda c: (-c.score, c.id))
        if pending and pending[0].score >= self.high:
            c = pending[0]
            return dict(base, action='verify' if c.position else 'resolve_depth', candidate=c.id)
        for view, state in self.views.items():
            if state == 'pending':
                return dict(base, action='observe', view=view)
        if pending:
            c = pending[0]
            return dict(base, action='verify' if c.position else 'resolve_depth', candidate=c.id)
        missing = [r for r, state in self.regions.items() if state != 'reviewed']
        unresolved = [c.id for c in self.candidates.values() if c.state not in ('rejected', 'confirmed')]
        invalid = [v for v, s in self.views.items() if s == 'invalid']
        return dict(base, action='supplement' if missing or unresolved or invalid else 'review_completion',
                    regions=missing, candidates=unresolved,
                    invalid_views=invalid)

    def cancel(self):
        self.state = 'cancelled'
