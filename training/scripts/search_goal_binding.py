"""Offline final-search-goal binding contract; not a navigation sender."""
import math
import uuid


class SearchGoalBinding:
    def __init__(self, task_id, point_id, pose, map_sha256):
        if not task_id or not point_id:
            raise ValueError('missing_identity')
        if not isinstance(map_sha256,str) or len(map_sha256)!=64 or any(c not in '0123456789abcdef' for c in map_sha256):
            raise ValueError('invalid_map_digest')
        if set(pose)!=set(('x','y','yaw')) or not all(type(v) in (int,float) and math.isfinite(v) for v in pose.values()):
            raise ValueError('invalid_pose')
        self.task_id,self.point_id=task_id,point_id
        self.pose=dict(pose)
        self.map_sha256=map_sha256
        self.generation=0
        self.goal_id=None
        self.used=set()

    def begin_attempt(self):
        self.generation+=1
        self.goal_id=None
        return self.generation

    def accept(self, *, generation, task_id, point_id, pose, map_sha256,
               goal_id, role, frame_id='map'):
        if generation!=self.generation or generation<=0:
            raise ValueError('stale_attempt')
        if role!='final_search_point' or frame_id!='map':
            raise ValueError('not_final_search_goal')
        if (task_id,point_id,map_sha256)!=(self.task_id,self.point_id,self.map_sha256) or pose!=self.pose:
            raise ValueError('binding_mismatch')
        key=uuid.UUID(goal_id).hex
        if key=='0'*32 or key in self.used or self.goal_id is not None:
            raise ValueError('duplicate_or_invalid_goal')
        self.goal_id=key
        self.used.add(key)
        return dict(goal_id=key,pose=dict(self.pose),task_id=self.task_id,
                    point_id=self.point_id,map_sha256=self.map_sha256,
                    generation=self.generation,actionable=False)

    def matches(self, goal_id, generation):
        return self.goal_id is not None and generation==self.generation and goal_id==self.goal_id

    def cancel(self):
        self.generation+=1
        self.goal_id=None
