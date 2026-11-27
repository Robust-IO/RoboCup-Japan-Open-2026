"""Failure alone does not authorize switching a narrow object's grasp strategy."""
def too_wide_for_side(decision,limit=.095):
    rows=decision.get('side_candidate_width_checks',[])
    if rows:
        widths=[r.get('evaluated_contact_width_m') or r.get('local_span_width_m') for r in rows]
        return bool(widths) and all(w is not None and w>limit for w in widths)
    return decision.get('observed_minimum_horizontal_width_m',0)>limit
