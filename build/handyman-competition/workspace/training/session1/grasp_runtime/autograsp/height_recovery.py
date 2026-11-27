"""Bounded upward compensation for a measured table-clearance shortfall."""
import math

def recovery_raise(exc):
    if str(exc) != 'open_hand_path_too_low_for_table':return None
    clearance=getattr(exc,'clearance_m',float('nan'))
    required=getattr(exc,'required_m',float('nan'))
    if not (math.isfinite(clearance) and math.isfinite(required)):return None
    dz=required-clearance+.002
    return dz if 0<dz<=.02 else None
