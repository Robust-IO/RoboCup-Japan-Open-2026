"""Measured tabletop -> physical lift travel; no fixed furniture-height ceiling."""
import math
OFFSET=.2771978151685961
LIFT_MIN_M=0.
LIFT_MAX_M=.69
PREFERRED_CLEARANCE_M=.08

def lift_targets(palm_height,current_lift=0.):
    if not all(type(v) in (int,float) and math.isfinite(v) for v in (palm_height,current_lift)):
        raise ValueError('finite_measured_height_required')
    lift=palm_height-OFFSET
    if not LIFT_MIN_M-1e-9<=lift<=LIFT_MAX_M+1e-9 or not LIFT_MIN_M<=current_lift<=LIFT_MAX_M:
        raise ValueError('required_height_outside_physical_flat_arm_range')
    lift=min(LIFT_MAX_M,max(LIFT_MIN_M,lift))
    high=min(LIFT_MAX_M,max(current_lift,lift+PREFERRED_CLEARANCE_M))
    return dict(release_lift_m=lift,extension_lift_m=high,palm_height_m=palm_height,
        available_cleanup_raise_m=min(PREFERRED_CLEARANCE_M,LIFT_MAX_M-lift),
        physical_lift_range_m=[LIFT_MIN_M,LIFT_MAX_M],height_source='measured_table_plus_held_offset_and_release_gap')

def release_height(table_height,bottom_below_palm,gap=.015):
    if (not all(type(v) in (int,float) and math.isfinite(v) for v in (table_height,bottom_below_palm,gap))
            or table_height<0 or not 0<bottom_below_palm<.2 or not .005<=gap<=.03):
        raise ValueError('invalid_measured_release_geometry')
    height=table_height+bottom_below_palm+gap
    lift_targets(height)
    return height

def extension_retreat(clearance):
    values=[clearance.get('minimum_clearance_m'),clearance.get('clearance_m')]
    if not all(type(v) in (int,float) and math.isfinite(v) for v in values) or values[0]<0:
        raise ValueError('finite_measured_extension_clearance_required')
    return max(0.,values[0]-values[1]+.02) if not clearance['ready'] else 0.
