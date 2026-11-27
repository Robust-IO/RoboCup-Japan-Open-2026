"""Fresh jaw-contact confirmation at the delivery position, never release."""
import time

def confirm(runtime,check,record,*,clock=time.monotonic):
    from canned_grasp_runtime import TOPICS
    from grasp_runtime.autograsp.grip_confirmation import RecloseEvidence
    from grasp_runtime.autograsp.coordinator import AttemptFailure
    from grasp_runtime.autograsp.recovery import Failure
    topic=TOPICS[3];pub=runtime.publisher(topic);gate=RecloseEvidence()
    try:
        check();runtime.ownership(topic)
        pub.publish(runtime.trajectory(['hand_motor_joint'],[0.],6.))
        record(dict(event='handover_reclose_sent',release_sent=False))
        start=clock()
        while clock()-start<11:
            check();runtime.pump(.02);runtime.ownership(topic)
            if clock()-start<6:continue
            state=runtime.state()
            result=gate.observe(state['stamp_ns'],float(state['joints'][5]))
            if result is True:
                opening=float(state['joints'][5])
                record(dict(event='handover_reclose_confirmed',opening_rad=opening,observations=len(gate.rows),pressure_measured=False))
                return opening
            if result is False:raise AttemptFailure(Failure.EMPTY_CLOSE,'handover_closed_without_object')
        raise AttemptFailure(Failure.HOLD_UNCERTAIN,'handover_reclose_feedback_unstable')
    finally:runtime.node.destroy_publisher(pub)
