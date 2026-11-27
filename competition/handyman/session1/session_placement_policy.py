"""Session-local support relaxation; never mutate the shared policy cache."""
from copy import deepcopy

def relaxed_support_policy(policy):
    policy=deepcopy(policy)
    policy['placement'].update(outer_edge_gap_m=.01,position_uncertainty_m=.005,planning_reserve_m=0.,center_based_selection=True,center_inset_m=.005,object_disk_edge_gap_m=.005,
                               local_patch_radius_m=.035,minimum_local_surface_points=30)
    return policy
