"""Current object depth feedback; background calibration is not a prerequisite.

Approach, lateral/forward correction and closure retain CanRuntime semantics.
The odom prediction selects an ROI only; it never substitutes for object depth.
"""


def runtime_class():
    from canned_grasp_runtime import CanRuntime
    from stopped_capture_runtime import with_stopped_capture
    from canned_grasp_geometry import predicted_before_contact, measure_near

    class FeedbackCanRuntime(with_stopped_capture(CanRuntime)):
        def observe_near(self, reference, task_id, max_frames=5):
            reason = 'no_current_can_measurement'
            for _ in range(max_frames):
                item = self.sample(force_new=True)
                snapshot = item['snapshot']
                expected = predicted_before_contact(reference, snapshot, task_id)
                try:
                    measured = measure_near(
                        item['depth'], snapshot, expected,
                        reference['radius_m'], reference['height_m'],
                        rgb=item['rgb'], appearance=reference['source_appearance'])
                    self.fresh_measurement(snapshot)
                    self.record(dict(
                        event='can_current_surface_feedback',
                        center_m=measured['center_m'],
                        depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
                        prediction_used_as_measurement=False))
                    return item, measured
                except ValueError as exc:
                    reason = str(exc)
                    index = getattr(self, 'feedback_rejected_count', 0) + 1
                    self.feedback_rejected_count = index
                    self.save(item, f'feedback-rejected-{index:03d}')
                    self.record(dict(event='can_feedback_reacquire', reason=reason,
                                     robot_commands_sent=False))
            raise ValueError('current_object_measurement_unresolved:' + reason)
    return FeedbackCanRuntime
