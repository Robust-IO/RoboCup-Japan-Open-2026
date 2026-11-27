"""Current RGBD adapter for generic held-surface placement.

No ROS initialization. Owning runtime supplies sample/palm_pose/save/fresh_measurement.
Default dependencies are existing deployed measurement and placement modules.
"""
import numpy as np
from transfer_actions import measured_release_shape

class DestinationObserver:
    def __init__(self,runtime,context,destination,config,task_id,release_rotation,*,
                 outcome_policy=None,measure=None,planner=None,rotation=None):
        if measure is None:
            from can_destination_surface import measure
        if planner is None:
            from placement_planner import plan_placement as planner
        if rotation is None:
            from hand_camera_ray import rotation
        self.runtime=runtime;self.context=context;self.destination=destination
        self.config=config;self.task_id=task_id;self.rotation=rotation
        self.measure=measure;self.planner=planner;self.outcome_policy=outcome_policy
        self.release_rotation=release_rotation;self.serial=0;self.last_stamp=None

    def observe(self,label,reference_plan=None):
        item=self.runtime.sample(processing_reserve_ns=100_000_000)
        snapshot=item['snapshot'];stamp=snapshot['streams']['depth']['stamp_ns']
        if self.last_stamp is not None and stamp<=self.last_stamp:
            raise ValueError('new_destination_depth_frame_required')
        if reference_plan is None:
            plane=self.measure(item['depth'],snapshot,self.destination)
        else:
            plane=dict(destination_id=reference_plan['destination_id'],
                       plane_map=reference_plan['plane_map'],
                       source_depth_stamp_ns=reference_plan.get('source_depth_stamp_ns'),
                       geometry_source='fixed_planned_support_plane')
        if plane['destination_id']!=self.destination['id'] or (reference_plan is None and plane['source_depth_stamp_ns']!=stamp):
            raise ValueError('same_frame_destination_plane_required')
        # Current palm pose maps palm-frame observed vertices into base coordinates.
        palm=self.runtime.palm_pose(snapshot)
        r=self.rotation(palm['quaternion'])
        points=np.asarray(self.context['vertices_palm_m'],float)@r.T+np.asarray(palm['translation'])
        lo=points.min(0);hi=points.max(0)
        volume=dict(center_m=((lo+hi)/2).tolist(),axes_base=np.eye(3).tolist(),
                    sizes_m=(hi-lo).tolist(),stamp_ns=stamp,
                    geometry_source='grasp_surface_transformed_by_current_palm_TF',
                    current_target_depth_measured=False)
        self.runtime.fresh_measurement(snapshot)
        self.serial+=1
        capture=self.runtime.save(item,f'destination-{self.serial:03d}-{label}',volume)
        self.last_stamp=stamp
        self.last_row=dict(item=item,plane=plane,volume=volume,evidence_ref=str(capture))
        return self.last_row

    def measure_and_plan(self,shape=None):
        # Keep observed geometry independent from object-category dimensions.
        shape=measured_release_shape(self.context,self.release_rotation) if shape is None else dict(shape)
        row=self.observe('plan')
        shape['holding_opening_rad']=float(self.runtime.state()['joints'][5])
        plan=self.planner(row['item']['depth'],row['item']['snapshot'],self.destination,
                          row['plane'],shape,self.config,self.task_id,minimum_shift_m=0.,
                          held_volume=row['volume'],outcome_policy=self.outcome_policy)
        plan.update(class_name=self.context['target'],shape='measured_irregular_surface',
                    evidence_ref=row['evidence_ref'],held_pose_is_kinematic=True,
                    navigation_parameters_changed=False)
        return plan

    def verify_support(self,plan):
        """Same-frame current palm/table check using the existing support raster.

        Must be paired with a fresh hold check; transformed grasp points alone
        cannot prove that the object remains in the gripper.
        """
        from session_table_support import support_from_plan
        from apple_transfer_auto_plan import map_point
        import task_outcome_policy
        from placement_verification import support_check
        row=self.observe('pre-release-support',reference_plan=plan)
        snapshot=row['item']['snapshot'];palm=self.runtime.palm_pose(snapshot)
        vertices=np.asarray(self.context['vertices_palm_m'])@self.rotation(palm['quaternion']).T+np.asarray(palm['translation'])
        ref=self.context.get('source_support_reference')
        if ref:
            support_point=np.asarray(ref['point_palm_m'])@self.rotation(palm['quaternion']).T+np.asarray(palm['translation'])
            vertices=np.vstack([vertices,support_point])
        mapped=np.asarray([map_point(snapshot,p) for p in vertices])
        check=support_check(mapped,row['plane'],plan,support_from_plan(plan),task_outcome_policy)
        self.runtime.fresh_measurement(snapshot)
        check.update(evidence_ref=row['evidence_ref'],geometry_source='held_surface_current_TF')
        return check

    def verify_release(self,plan):
        """Release above the destination using current TF, without object re-ID."""
        from apple_transfer_auto_plan import map_point
        from grasp_runtime.autograsp.kinematics import CONTACT
        from release_geometry import release_check
        import task_outcome_policy
        row=self.observe('release-center',reference_plan=plan)
        snapshot=row['item']['snapshot'];palm=self.runtime.palm_pose(snapshot)
        rotation=self.rotation(palm['quaternion'])
        center=map_point(snapshot,np.asarray(palm['translation'])+rotation@CONTACT)
        # Height remains based on the measured held object's support offset.
        vertices=np.asarray(self.context['vertices_palm_m'])@rotation.T+np.asarray(palm['translation'])
        ref=self.context.get('source_support_reference')
        if ref:
            vertices=np.vstack([vertices,np.asarray(ref['point_palm_m'])@rotation.T+np.asarray(palm['translation'])])
        mapped=np.asarray([map_point(snapshot,p) for p in vertices])
        result=release_check(center,mapped,row['plane'],self.destination,plan,task_outcome_policy.RELEASE_BOTTOM_GAP_M)
        self.runtime.fresh_measurement(snapshot)
        result.update(evidence_ref=row['evidence_ref'],depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
                      visual_hold_required=False,object_presence_verified=False,
                      height_source='current_palm_TF_and_held_support_offset',
                      support_reference='fixed_planned_support_plane',support_plane_remeasured=False)
        self.runtime.record(dict(event='release_center_check',**result))
        return result
