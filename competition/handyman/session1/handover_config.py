"""Validate the fixed sample delivery configuration without ROS or motion."""
import hashlib,math
from pathlib import Path

def validate_handover(config,*,check_sources=False):
    h=config.get('handover',{})
    if h.get('schema')!='handyman_fixed_sample_handover_v1' or h.get('destination')!='Moderator':raise ValueError('prepare_session3_handover_required')
    def vector(value):return isinstance(value,list) and len(value)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in value)
    if not vector(h.get('delivery_point_map_m')):raise ValueError('invalid_delivery_point')
    if not 0<h.get('estimated_center_tolerance_m',0)<h.get('official_distance_m',0)==.3:raise ValueError('invalid_delivery_margin')
    if check_sources:
        if not h.get('sources'):raise ValueError('delivery_source_provenance_required')
        for source in h['sources']:
            if hashlib.sha256(Path(source['path']).read_bytes()).hexdigest()!=source['sha256']:raise ValueError('delivery_source_changed_reexport_required')
    return h
