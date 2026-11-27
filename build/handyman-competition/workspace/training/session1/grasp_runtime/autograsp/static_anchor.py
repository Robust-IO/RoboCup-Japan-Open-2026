"""Pre-contact static target assumption; never manufacture a depth measurement."""
from .occlusion_anchor import MISSING

REASONS=MISSING|{'surface_moved_beyond_registration_window'}

def allowed(backend,reason,lift_delta=None):
    return (lift_delta is None and reason in REASONS
            and getattr(backend,'static_precontact_anchor',False)
            and getattr(backend,'preextension_anchor_confirmed',False)
            and backend.holding_state()=='empty')

def retain(backend,reason):
    item,m=backend._observe_locked()
    m=dict(m,static_anchor_authorized=True,depth_measured=False,
           position_source='confirmed_map_anchor_static_before_contact')
    backend.record(dict(event='static_precontact_anchor_retained',reason=reason,
                        measurement=m,object_stationary_assumed=True))
    return item,m
