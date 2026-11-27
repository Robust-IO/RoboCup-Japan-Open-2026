"""Offline geometry adapters. Caller supplies current RGBD/TF, never Unity truth."""
import math
from statistics import median


def points_valid(points):
    if not points or any(len(p)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in p) for p in points):
        raise ValueError('finite_measured_points_required')


def inside(x,y,polygon):
    if len(polygon)<3: raise ValueError('room_polygon_required')
    hit=False
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]: hit=not hit
    return hit


def observation_result(*,target,source_table,points_map,room_polygon,source_support_verified,evidence_ref,expected_target='dog_doll',expected_source='white_side_table#living_room',room='living_room'):
    """Points must be segmented target cloud; support is measured association."""
    points_valid(points_map)
    if target!=expected_target or source_table!=expected_source:
        return dict(status='incomplete',reason='wrong_target_or_source')
    if source_support_verified is not True or not evidence_ref:
        return dict(status='incomplete',reason='source_observation_missing')
    center=[median([p[i] for p in points_map]) for i in range(3)]
    if not inside(center[0],center[1],room_polygon):
        return dict(status='incomplete',reason='target_outside_'+room)
    return dict(status='ok',target=target,source_table=source_table,room=room,
                confirmed=True,evidence_ref=str(evidence_ref),target_center_map=center)


def rotation_valid(r):
    points_valid(r)
    if len(r)!=3: raise ValueError('rotation_must_be_3x3')
    for i in range(3):
        for j in range(3):
            if abs(sum(r[i][k]*r[j][k] for k in range(3))-(1 if i==j else 0))>1e-5:
                raise ValueError('rotation_not_orthonormal')
    a,b,c=r
    det=a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])
    if abs(det-1)>1e-5: raise ValueError('rotation_not_right_handed')


def held_geometry(*,points_map,palm_position_map,palm_rotation_map,release_rotation_map,hold_verified,evidence_ref):
    """Rotations map palm vectors to map axes. Result z is map vertical.

    Visible points may omit true bottom; do not authorize release from this alone.
    """
    points_valid(points_map);points_valid([palm_position_map])
    rotation_valid(palm_rotation_map);rotation_valid(release_rotation_map)
    if hold_verified is not True or not evidence_ref: raise ValueError('current_verified_hold_required')
    vertices=[]
    for p in points_map:
        d=[p[i]-palm_position_map[i] for i in range(3)]
        local=[sum(palm_rotation_map[j][i]*d[j] for j in range(3)) for i in range(3)]
        vertices.append([sum(release_rotation_map[i][j]*local[j] for j in range(3)) for i in range(3)])
    return dict(vertices_release_map_relative=vertices,bottom_below_palm_m=-min(v[2] for v in vertices),
                evidence_ref=str(evidence_ref),complete_surface_verified=False,release_authorized=False)


def placement_height_plan(core,geometry,*,measured_table_height,current_lift,bottom_observed,release_gap=.015):
    """Reuse existing table_transfer_core without commanding robot motion."""
    if bottom_observed is not True: raise ValueError('held_bottom_observation_required')
    height=core.release_height(measured_table_height,geometry['bottom_below_palm_m'],release_gap)
    return dict(**core.lift_targets(height,current_lift),release_authorized=False)
