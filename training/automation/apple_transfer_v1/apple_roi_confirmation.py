"""Bounded same-frame identity evidence, NEVER replacement RGBD geometry.

Only one already measured near-grasp apple can request one320px crop/input640.
No tiled search, old anchor, confidence threshold reduction or motion authority.
"""
import math

SCHEMA = 'near_grasp_roi_v1'
CROP_SIZE = 320
INPUT_SIZE = 640
CONFIDENCE = .8


def _box(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 4
            or not all(type(v) in (float, int) and math.isfinite(v) for v in value)
            or value[2] <= value[0] or value[3] <= value[1]):
        raise ValueError('invalid_roi_box')
    return list(value)


def _confidence(value):
    if type(value) not in (float, int) or not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError('invalid_roi_confidence')
    return value


def iou(left, right):
    a, b = _box(left), _box(right)
    intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    area = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
    return intersection / area


def proposal_plan(proposals, snapshot, full_detections=()):
    """Return either a tightly bound crop or an explicit reason not to run one."""
    def decline(reason):return dict(eligible=False, reason=reason)
    try:
        if not isinstance(proposals, list) or len(proposals) != 1 or 'rejected_reason' in proposals[0]:
            return decline('unique_measured_proposal_required')
        proposal=proposals[0];candidate=proposal['candidate'];detection=proposal['detection']
        score=_confidence(detection['confidence']);box=_box(detection['box'])
        if detection['name'] != 'apple' or score < .1:return decline('existing_apple_detection_required')
        if score >= CONFIDENCE:return decline('full_frame_confirmation_already_available')
        rgb, depth = snapshot['streams']['rgb'], snapshot['streams']['depth']
        if (rgb['width'], rgb['height']) != (640, 480):return decline('reviewed_camera_shape_required')
        if (type(depth['stamp_ns']) is not int or type(rgb['stamp_ns']) is not int
                or min(depth['stamp_ns'], rgb['stamp_ns']) <= 0
                or abs(depth['stamp_ns']-rgb['stamp_ns']) > 30_000_000
                or candidate['stamp_ns'] != depth['stamp_ns']):
            return decline('same_frame_rgbd_required')
        point=candidate['center_m'];radius=candidate['radius_m'];residual=candidate['fit_residual_p95_m']
        if (len(point) != 3 or not all(type(v) in (float,int) and math.isfinite(v) for v in [*point,radius,residual])
                or candidate.get('frame_id') != 'base_link' or not .45 <= point[0] <= 1.05
                or abs(point[1]) > .35 or not .35 <= point[2] <= .8
                or not .015 <= radius <= .065 or not 0 <= residual <= .012):
            return decline('measured_near_grasp_geometry_required')
        # The crop verifies class identity only; it neither proves support nor
        # authorizes motion. Initial source/table planning still checks support.
        if box[0] <= 1 or box[1] <= 1 or box[2] >= 639 or box[3] >= 479:
            return decline('unclipped_source_box_required')
        for item in full_detections:
            if (item['name'] != 'apple' and _confidence(item['confidence']) >= CONFIDENCE
                    and iou(item['box'],box) >= .25):
                return decline('full_frame_conflicting_class')
        x=int(max(0,min(640-CROP_SIZE,(box[0]+box[2]-CROP_SIZE)/2)))
        y=int(max(0,min(480-CROP_SIZE,(box[1]+box[3]-CROP_SIZE)/2)))
        roi=[x,y,x+CROP_SIZE,y+CROP_SIZE]
        if not (x+8 <= box[0] and y+8 <= box[1] and box[2] <= roi[2]-8 and box[3] <= roi[3]-8):
            return decline('source_box_needs_crop_context')
        return dict(eligible=True,reason=None,roi=roi,input_size=INPUT_SIZE,
                    source_box=box,depth_stamp_ns=depth['stamp_ns'],rgb_stamp_ns=rgb['stamp_ns'],
                    full_frame_confidence=score,confidence_threshold=CONFIDENCE)
    except (KeyError,TypeError,ValueError,IndexError):
        return decline('invalid_roi_source')


def assess(plan, detections):
    """Crop detections are mapped back to original image pixels before entry."""
    report=dict(schema=SCHEMA,plan=plan,detections=detections,status='skipped',
                reason=plan.get('reason'),confidence=None,original_geometry_preserved=True,
                motion_authorized=False)
    if not plan.get('eligible'):
        if detections:raise ValueError('detections_without_eligible_roi')
        return report
    box=plan['source_box'];roi=plan['roi']
    if not isinstance(detections,list):raise ValueError('invalid_roi_detections')
    for item in detections:
        b=_box(item['box']);_confidence(item['confidence'])
        if not isinstance(item['name'],str) or not item['name']:raise ValueError('invalid_roi_class')
        if any(b[i] < roi[i]-1e-3 for i in (0,1)) or any(b[i] > roi[i]+1e-3 for i in (2,3)):
            raise ValueError('roi_detection_outside_crop')
    apples=[item for item in detections if item['name']=='apple' and item['confidence']>=.1]
    if any(item['name']!='apple' and item['confidence']>=CONFIDENCE and iou(item['box'],box)>=.25 for item in detections):
        return dict(report,status='conflict',reason='roi_conflicting_class')
    if len(apples)>1:return dict(report,status='conflict',reason='roi_ambiguous_apple')
    if not apples:return dict(report,status='not_confirmed',reason='roi_no_apple')
    item=apples[0];b=item['box']
    if (iou(b,box)<.5 or abs((b[0]+b[2]-box[0]-box[2])/2)>(box[2]-box[0])*.25
            or abs((b[1]+b[3]-box[1]-box[3])/2)>(box[3]-box[1])*.25):
        return dict(report,status='conflict',reason='roi_target_mismatch')
    if item['confidence']<CONFIDENCE:return dict(report,status='not_confirmed',reason='roi_below_identity_threshold')
    return dict(report,status='confirmed',reason=None,confidence=item['confidence'])


def identity_evidence(result, snapshot):
    """Recompute binding/ambiguity at the observer; never trust a status flag."""
    proof=result.get('near_grasp_roi_confirmation')
    if proof is None:return None
    if not isinstance(proof,dict):raise ValueError('invalid_roi_proof')
    plan=proposal_plan(result.get('tracking_proposals'),snapshot,result.get('detections',[]))
    if proof.get('plan')!=plan:raise ValueError('roi_source_binding_mismatch')
    model=result.get('model_sha256')
    if (not isinstance(model,str) or len(model)!=64 or any(c not in '0123456789abcdef' for c in model)
            or proof.get('model_sha256')!=model):raise ValueError('roi_model_binding_mismatch')
    try:checked=assess(plan,proof['detections'])
    except (KeyError,TypeError,ValueError):raise ValueError('invalid_roi_proof') from None
    if any(proof.get(key)!=checked[key] for key in ('schema','status','reason','confidence','original_geometry_preserved','motion_authorized')):
        raise ValueError('roi_proof_verdict_mismatch')
    if checked['status']=='conflict' or plan.get('reason')=='full_frame_conflicting_class':
        raise ValueError(checked['reason'])
    if checked['status']!='confirmed':return None
    return dict(confidence=checked['confidence'],confidence_basis='same_frame_near_grasp_roi',
                full_frame_confidence=plan['full_frame_confidence'],proof=proof)
