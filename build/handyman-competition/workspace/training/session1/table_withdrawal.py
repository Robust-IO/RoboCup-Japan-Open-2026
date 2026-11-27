"""Measured table withdrawal adapters. Calls existing finite-motion backend only.

Projected clearance is a tabletop check, not certification of the entire arm's
swept volume or obstacles behind the base. Main runtime owns route supervision.
"""
import numpy as np
from itertools import count
from task_recovery import check_cancel,motion_cancel,retry_read


def retreat_distance(points_map,table_center,table_half,heading,*,clearance=.03):
    points=np.asarray(points_map,float);c=np.asarray(table_center,float)
    half=np.asarray(table_half,float);f=np.asarray(heading,float)
    if (points.ndim!=2 or points.shape[1]!=3 or not len(points) or c.shape!=(2,)
            or half.shape!=(2,) or f.shape!=(2,) or not np.isfinite(points).all()
            or not np.isfinite([*c,*half,*f,clearance]).all() or (half<=0).any()
            or abs(np.linalg.norm(f)-1)>1e-5 or clearance<0):
        raise ValueError('invalid_table_withdrawal_geometry')
    front=float(c@f-half@abs(f));tip=float((points[:,:2]@f).max())
    return max(0.,tip-front+clearance)


class TableWithdrawal:
    def __init__(self,runtime,motion_backend,observe_geometry,*,label='table-withdraw'):
        """observe_geometry -> current map surface/hand points and measured table.

        Include hand/held envelope in points_map; a target center alone is not
        sufficient. The observation callback must check frame freshness.
        """
        self.runtime=runtime;self.backend=motion_backend;self.observe=observe_geometry;self.label=label;self.invocations=0

    def __call__(self):
        self.invocations+=1
        for attempt in count():
            check_cancel(motion_cancel(self.backend))
            row=retry_read(self.observe,cancel=motion_cancel(self.backend))
            distance=retreat_distance(row['points_map'],row['table_center'],row['table_half'],row['heading'])
            if distance<=.001:
                return dict(clear_to_fold=True,evidence_ref=row['evidence_ref'],
                            clearance_basis='current_hand_held_projection_against_furniture_bounds',corrections=attempt)
            state=self.runtime.state();self.backend.stage='withdraw'
            self.backend.adjust(f'{self.label}-{self.invocations}-{attempt}',state['poses']['map'],[-distance,0.,0.],
                                state['joints'][:5],state['joints'][5])
        raise AssertionError('unreachable')


class PlacementStandoff:
    def __init__(self,observer,motion_backend,*,clearance=None,predict=None,map_point=None):
        if clearance is None:
            from pepper_place import extension_clearance as clearance
        if predict is None:
            from pregrasp_geometry import expected_horizontal_center as predict
        if map_point is None:
            from apple_transfer_auto_plan import map_point
        self.observer=observer;self.backend=motion_backend
        self.clearance=clearance;self.predict=predict;self.map_point=map_point

    def __call__(self,plan):
        for attempt in count():
            check_cancel(motion_cancel(self.backend))
            row=retry_read(lambda:self.observer.observe('extension-clearance'),cancel=motion_cancel(self.backend))
            snapshot=row['item']['snapshot']
            predicted=self.map_point(snapshot,self.predict(plan['palm_height_m'],plan['held_center_minus_palm_m']))
            check=self.clearance(predicted,snapshot,row['plane'],plan)
            if check['ready']:return plan
            distance=max(0.,check['minimum_clearance_m']-check['clearance_m']+.02)
            if not np.isfinite(distance) or distance<=0:raise ValueError('invalid_measured_standoff')
            state=self.observer.runtime.state();self.backend.stage='withdraw'
            self.backend.adjust(f'session1-standoff-{attempt}',state['poses']['map'],[-distance,0.,0.],
                                state['joints'][:5],state['joints'][5])
            plan=self.observer.measure_and_plan()
        raise AssertionError('unreachable')
