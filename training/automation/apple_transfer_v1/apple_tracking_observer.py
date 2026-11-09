"""Read-only adapter from measured RGBD proposals to identity continuity.

Does not alter the detector's candidate gate or authorize grasp/motion. Timestamp
quality remains that of the upstream bridge, not proof of acquisition alignment.
"""
from apple_identity_track import AppleIdentityTrack
from apple_room_evidence import room_check
from target_anchor_tracking import point_transform


class TrackingObserver:
    def __init__(self, task_id, epoch, environment, room, *, near_grasp_roi=False,
                 near_geometry=False, near_geometry_gap_m=.15):
        if type(near_grasp_roi) is not bool:raise ValueError('invalid_roi_option')
        if type(near_geometry) is not bool:raise ValueError('invalid_near_geometry_option')
        self.track = AppleIdentityTrack(task_id, epoch)
        self.task_id, self.environment, self.room = task_id, environment, room
        self.near_grasp_roi = near_grasp_roi
        self.geometry = None
        if near_geometry:
            from near_apple_geometry import NearAppleGeometry
            self.geometry = NearAppleGeometry(task_id, epoch, near_geometry_gap_m)

    def geometry_ready(self, snapshot):
        return self.geometry is not None and self.geometry.ready(snapshot)

    def observe_geometry(self, rgb, depth, snapshot, now_ns):
        """Parent-owned fast path: skip inference entirely after a valid handoff."""
        if not self.geometry_ready(snapshot):return None
        import time
        start=time.monotonic();model=self.geometry.anchor.get('model_sha256')
        try:
            evidence=self.geometry.measure(rgb,depth,snapshot,now_ns)
            room=room_check(evidence['current_geometry_candidate'],snapshot,self.environment,self.room)
            if room.get('room_confirmed') is not True:raise ValueError('near_geometry_room_unverified')
            evidence['room_evidence']=[room]
            status='geometry_only';reason=None
        except (KeyError,ValueError,TypeError) as exc:
            self.track.reset();self.geometry.reset()
            evidence=dict(status='reobserve',reason=str(exc),identity_confirmed=False,
                          motion_authorized=False,grasp_center_verified=False)
            status='rejected';reason=str(exc)
        return dict(status=status,reason=reason,detections=[],tracking_proposals=[],
            detector_ran=False,inference_skipped_for_near_geometry=True,model_sha256=model,
            worker_processing_s=0.,geometry_processing_s=time.monotonic()-start,
            identity_tracking=evidence,identity_tracking_task_id=self.task_id,motion_authorized=False)

    def observe(self, result, snapshot, now_ns, *, capture=None):
        try:
            if result.get('timing', {}).get('freshness_passed') is not True:
                raise ValueError('upstream_freshness_failed')
            stamp = snapshot['streams']['depth']['stamp_ns']
            rgb_stamp = snapshot['streams']['rgb']['stamp_ns']
            if not 0 <= (now_ns-min(rgb_stamp, stamp))/1e9 <= .4:
                raise ValueError('rgbd_result_expired')
            if abs(rgb_stamp-stamp)>50_000_000:
                raise ValueError('rgbd_unsynchronised')
            proposals = result.get('tracking_proposals')
            if not isinstance(proposals, list):
                raise ValueError('measured_proposals_missing')
            roi_identity = None
            if self.near_grasp_roi:
                from apple_roi_confirmation import identity_evidence
                roi_identity = identity_evidence(result, snapshot)
            measurements = []
            room_evidence = []
            for proposal in proposals:
                # An unmeasurable apple-like detection may be a second target.
                # Do not silently discard it and claim unique association.
                if 'rejected_reason' in proposal:
                    raise ValueError('unresolved_geometry_proposal')
                candidate, detection = proposal['candidate'], proposal['detection']
                if candidate['stamp_ns'] != stamp:
                    raise ValueError('candidate_stamp_mismatch')
                room = room_check(candidate, snapshot, self.environment, self.room)
                room_evidence.append(room)
                if not room['room_confirmed']:
                    raise ValueError('proposal_room_unverified')
                point = point_transform(candidate['center_m'],
                    snapshot['tracking_transforms']['odom_from_base_at_depth'],
                    stamp=stamp, source='base_link', target='odom')
                x1,y1,x2,y2 = detection['box']
                stream = snapshot['streams']['rgb']
                measurements.append(dict(point_odom_m=point,
                    stamp_ns=stamp, confidence=roi_identity['confidence'] if roi_identity else detection['confidence'],
                    full_frame_confidence=detection['confidence'],
                    confidence_basis='same_frame_near_grasp_roi' if roi_identity else 'full_frame_detector',
                    roi_identity_evidence=roi_identity,
                    radius_m=candidate['radius_m'], fit_residual_p95_m=candidate['fit_residual_p95_m'],
                    measurement='fresh_rgbd', class_name=detection['name'],
                    red_appearance=True, depth_valid=True,
                    clipped=x1<=1 or y1<=1 or x2>=stream['width']-1 or y2>=stream['height']-1,
                    support_consistent=proposal['support'].get('support_consistent') is True))
            evidence = self.track.update(measurements, stamp_ns=stamp, now_ns=now_ns,
                task_id=self.task_id, epoch=snapshot.get('tracking_epoch'), room_confirmed=True)
            if self.geometry is not None:
                self.geometry.remember_identity(evidence,result,snapshot,capture)
            return dict(evidence, room_evidence=room_evidence,
                        timestamp_caveat='Upstream bridge stamps may be receipt time, not acquisition time.')
        except (KeyError, ValueError, TypeError) as exc:
            self.track.reset()
            if self.geometry is not None:self.geometry.reset()
            return dict(status='reobserve', reason=str(exc), identity_confirmed=False,
                        motion_authorized=False, grasp_center_verified=False)
