"""Shared class-specific overrides for fixed and dynamically resolved tasks."""
from pathlib import Path
import json
from functools import lru_cache
FLAGS=('skip_tiny_surface_extent','unlimited_registration_shift','skip_contact_band_depth','trial_before_rotation_enabled','skip_actual_pad_overlap')
@lru_cache(maxsize=1)
def catalog():
    return json.loads((Path(__file__).resolve().parents[3]/'config/object-grasp-profiles.json').read_text())
def profile_for(target):
    c=catalog();return dict(c['default'],**c['objects'].get(target,{}))
def apply(backend,target):
    profile=profile_for(target)
    for flag in FLAGS:setattr(backend,flag,profile.get(flag,False) is True)
    changed=getattr(backend,'object_grasp_profile_target',None)!=target
    backend.object_grasp_profile_target=target
    if changed and callable(getattr(backend,'record',None)):
        backend.record(dict(event='object_grasp_profile_applied',target=target,profile=profile,binding_basis='target_class_not_session_number'))
    return profile
