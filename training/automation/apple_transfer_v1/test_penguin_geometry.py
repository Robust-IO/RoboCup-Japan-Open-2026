import copy,json,math,subprocess,sys,unittest
from pathlib import Path
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
from penguin_geometry import estimate_source,measure_near,fit_observed_radius,lift_follow,connected_object_surface
from penguin_hold import expected_centers,palm_points
from penguin_runtime import PenguinRuntime,view_goal_for_pixel
from penguin_tasks import load_task,validate_task
from manipulation_profiles import object_profile
from canned_grasp_geometry import contact_point
from hand_camera_ray import rotation


def rendered_penguin(*,lift=0.,remove_body=False,remove_head=False,lateral=0.):
    h,w,f=360,480,420.;v,u=np.indices((h,w));s,c=np.sin(.65),np.cos(.65)
    orient=np.array([[0,-s,c],[-1,0,0],[0,-c,-s]]);origin=np.array([.1,0,1.])
    rays=np.stack(((u-w/2)/f,(v-h/2)/f,np.ones_like(u)),-1)@orient.T
    body=np.array([.75,lateral,.475+lift]);head=body+[0,0,.0645];values=[];colors=[]
    for name,center,radius,color in (('body',body,.049,[195]*3),('head',head,.023,[45]*3)):
        if name=='body' and remove_body or name=='head' and remove_head:continue
        offset=origin-center;aa=np.sum(rays*rays,-1);bb=2*(rays@offset);cc=offset@offset-radius**2
        disc=bb*bb-4*aa*cc;t=(-bb-np.sqrt(np.maximum(disc,0)))/(2*aa)
        valid=(disc>=0)&(t>0)
        if name=='body':valid&=(origin+t[:,:,None]*rays)[:,:,2]>=body[2]-.035
        values.append(np.where(valid,t,np.inf));colors.append(color)
    t=(.52+lift-origin[2])/rays[:,:,2];p=origin+t[:,:,None]*rays
    wing=(t>0)&(abs(p[:,:,0]-body[0])<.012)&(abs(p[:,:,1])<.075)
    values.append(np.where(wing,t,np.inf));colors.append([45]*3)
    table=(.44-origin[2])/rays[:,:,2];stack=np.stack(values);nearest=stack.min(0);part=stack.argmin(0);hit=nearest<table
    depth=np.where(hit,nearest,table);rgb=np.full((*depth.shape,3),[235,230,220],np.uint8)
    for i,col in enumerate(colors):rgb[hit&(part==i)]=col
    yy,xx=np.where(hit);det=dict(name='toy_penguin',confidence=.94,box=[float(xx.min()-2),float(yy.min()-2),float(xx.max()+3),float(yy.max()+3)])
    snapshot=dict(streams={k:dict(stamp_ns=1_000_000_000,width=w,height=h) for k in ('rgb','depth')},
        rgb_k=[f,0,w/2,0,f,h/2,0,0,1],depth_k=[f,0,w/2,0,f,h/2,0,0,1],rgb_d=[0]*5,
        rgb_from_depth=dict(translation=[0,0,0],quaternion=[0,0,0,1]),
        base_from_rgb=dict(translation=origin.tolist(),quaternion=Rotation.from_matrix(orient).as_quat().tolist()))
    return rgb,depth,snapshot,det,body,head


class PenguinGeometryTests(unittest.TestCase):
    def test_extent_uses_connected_depth_not_background_or_radius_clipping(self):
        own=np.arange(.02,.12,.008)
        points=np.zeros((1,len(own)+1,3));points[0,:-1,0]=own;points[0,-1,0]=.5
        mask=connected_object_surface(points,np.ones(points.shape[:2],bool),np.zeros(3),.02,np.array([0,0,10]),.023)
        self.assertTrue(mask[0,:-1].all());self.assertFalse(mask[0,-1])
        self.assertGreater(points[mask,0].max(),.09)
    def setUp(self):self.rgb,self.depth,self.s,self.det,self.body,self.head=rendered_penguin()
    def source(self):return estimate_source(self.rgb,self.depth,self.s,self.det)
    def test_foreground_hidden_body_is_labelled_estimate_and_head_stays_measured(self):
        ref=self.source();rgb=self.rgb.copy();d=self.depth.copy()
        body_pixels=np.all(rgb==[195,195,195],axis=-1)
        d[body_pixels]*=.7;rgb[body_pixels]=[245]*3
        m=measure_near(rgb,d,self.s,self.body,self.head,ref)
        self.assertFalse(m['current_body_depth_measured']);self.assertTrue(m['current_head_depth_measured'])
        self.assertTrue(m['predicted_center_returned']);self.assertTrue(m['body_occlusion']['foreground_occlusion_observed'])
        np.testing.assert_allclose(m['cap_center_m'],self.head,atol=.002)
        d[np.all(rgb==[45,45,45],axis=-1)]=0
        with self.assertRaises(ValueError):measure_near(rgb,d,self.s,self.body,self.head,ref)
    def test_head_only_lift_proof_requires_independent_head_motion(self):
        tf=dict(translation=[0,0,0],quaternion=[0,0,0,1],stamp_ns=1,source_frame='base_link',target_frame='odom')
        bs=dict(tracking_epoch='one',palm_base_at_depth=[.6,.078,.55],
                streams=dict(depth=dict(stamp_ns=1)),
                tracking_transforms=dict(odom_from_base_at_depth=tf),joints=dict(arm_lift_joint=.32,hand_motor_joint=.3))
        ass=copy.deepcopy(bs);ass['palm_base_at_depth'][2]+=.1;ass['joints']['arm_lift_joint']+=.1
        ass['streams']['depth']['stamp_ns']=2;ass['tracking_transforms']['odom_from_base_at_depth']['stamp_ns']=2
        before=dict(current_body_depth_measured=False,current_head_depth_measured=True,stamp_ns=1,
                    cap_center_m=[.7,.078,.6])
        after=dict(before,stamp_ns=2,cap_center_m=[.7,.078,.7])
        proof=lift_follow(before,bs,after,ass,.3)
        self.assertTrue(proof['grasp_lift_verified']);self.assertFalse(proof['body_motion_independently_measured'])
        after['cap_center_m']=before['cap_center_m']
        with self.assertRaises(ValueError):lift_follow(before,bs,after,ass,.3)
    def test_open_source_can_be_reobserved_after_small_table_shift(self):
        ref=self.source();rgb,d,s,_,b,c=rendered_penguin(lateral=.04)
        s['joints']=dict(hand_motor_joint=.65)
        m=measure_near(rgb,d,s,self.body,self.head,ref,open_pregrasp=True)
        np.testing.assert_allclose(m['center_m'],b,atol=.002)
        np.testing.assert_allclose(m['cap_center_m'],c,atol=.002)
        self.assertFalse(m['predicted_center_returned'])
        with self.assertRaises(ValueError):measure_near(rgb,d,s,self.body,self.head,ref)
        s['joints']['hand_motor_joint']=.3
        with self.assertRaises(ValueError):measure_near(rgb,d,s,self.body,self.head,ref,open_pregrasp=True)
    def test_occluded_patch_uses_observed_size_and_current_points(self):
        rng=np.random.default_rng(712);radius=.049;center=np.array([.7,.08,.53])
        az=rng.uniform(-.7,.7,180);el=rng.uniform(-.12,.32,180)
        directions=np.c_[-np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)]
        points=center+radius*directions+rng.normal(0,.00025,directions.shape)
        actual,r,proof=fit_observed_radius(points,radius,center+[.008,-.006,.002])
        np.testing.assert_allclose(actual,center,atol=.0015)
        self.assertEqual(r,radius);self.assertFalse(proof['hidden_points_used'])
        plane=points.copy();plane[:,0]=.65
        with self.assertRaises(ValueError):fit_observed_radius(plane,radius,center)
    def test_known_radius_fit_excludes_small_clutter_without_creating_points(self):
        rng=np.random.default_rng(912);center=np.array([.7,.08,.53]);r=.049
        az=rng.uniform(-.7,.7,240);el=rng.uniform(-.4,.4,240)
        points=center+r*np.c_[-np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)]
        clutter=points[:35]+[.012,0,0]
        actual,_,proof=fit_observed_radius(np.r_[points,clutter],r,center)
        np.testing.assert_allclose(actual,center,atol=.001)
        self.assertGreater(proof['outlier_fraction'],.05)
        self.assertFalse(proof['hidden_points_used'])
    def test_wide_body_is_not_the_head_contact_target(self):
        m=self.source();np.testing.assert_allclose(m['center_m'],self.body,atol=.0015)
        np.testing.assert_allclose(m['cap_center_m'],self.head,atol=.0015)
        self.assertGreater(2*m['body_radius_m'],.093);self.assertLess(2*m['cap_radius_m'],.093)
        self.assertGreaterEqual(m['head_above_support_m']+m['head_grasp_axial_offset_m']-m['observed_wing_top_above_support_m'],.0215-1e-9)
        self.assertFalse(m['motion_authorized'])
    def test_new_near_depth_tracks_both_parts(self):
        ref=self.source();rgb,d,s,_,b,c=rendered_penguin(lift=.08)
        m=measure_near(rgb,d,s,b+[.002,0,.001],c+[.002,0,.001],ref)
        np.testing.assert_allclose(m['center_m'],b,atol=.002);np.testing.assert_allclose(m['cap_center_m'],c,atol=.002)
        self.assertFalse(m['predicted_center_returned']);self.assertGreaterEqual(m['horizontal_support_radius_m'],ref['observed_horizontal_extent_m'])
    def test_wrong_class_registration_zero_and_missing_parts_fail(self):
        with self.assertRaises(ValueError):estimate_source(self.rgb,self.depth,self.s,dict(self.det,name='ground_pepper'))
        with self.assertRaises(ValueError):estimate_source(self.rgb,self.depth*0,self.s,self.det)
        s=copy.deepcopy(self.s);s['rgb_from_depth']['translation']=[.01,0,0]
        with self.assertRaises(ValueError):estimate_source(self.rgb,self.depth,s,self.det)
        for kw in (dict(remove_body=True),dict(remove_head=True)):
            rgb,d,s,det,_,_=rendered_penguin(**kw)
            with self.subTest(kw=kw),self.assertRaises(ValueError):estimate_source(rgb,d,s,det)
    def test_reference_cannot_borrow_pepper_identity(self):
        from penguin_geometry import validate_reference
        model=self.source();model['class_name']='ground_pepper'
        with self.assertRaises(ValueError):validate_reference(model)
    def test_current_palm_rotation_transports_both_parts(self):
        palm=dict(translation=[.67,0,.575],quaternion=[0,0,0,1]);bp,cp=palm_points(self.body,self.head,palm)
        held=dict(class_name='toy_penguin',grasp_lift_verified=True,model=self.source(),body_palm_m=bp,cap_palm_m=cp)
        rotated=dict(translation=[.3,.078,.67],quaternion=Rotation.from_euler('y',.8).as_quat().tolist())
        b,c=expected_centers(rotated,held)
        np.testing.assert_allclose(c-b,rotation(rotated['quaternion'])@(self.head-self.body),atol=1e-12)

class PenguinEntryTests(unittest.TestCase):
    def test_wings_increase_clearance_before_lateral_sweep(self):
        r=PenguinRuntime.__new__(PenguinRuntime)
        r.current_penguin_geometry=dict(horizontal_support_radius_m=.0714,cap_radius_m=.023)
        plan=r.approach_plan([.1514,.399,.0003])
        self.assertEqual(plan['phase'],'clear_penguin_extent_before_lateral_alignment')
        self.assertAlmostEqual(plan['delta'][0],-.04)
        self.assertEqual(r.approach_plan([.1914,.399,0.])['delta'],[0.,.399,0.])
        self.assertEqual(r.approach_plan([.1914,.002,0.])['phase'],'straight_contact_approach')
        self.assertEqual(r.approach_plan([-.006,-.017,.0015]),
                         dict(phase='small_open_head_centering',delta=[0.,-.017,0.]))
    def view_snapshot(self):
        return dict(rgb_k=[554.,0.,320.,0.,554.,240.,0.,0.,1.],
                    streams=dict(rgb=dict(width=640)),joints=dict(head_pan_joint=0.,head_tilt_joint=-.5),
                    base_from_rgb=dict(translation=[0.,0.,0.],quaternion=[0.,0.,0.,1.]))
    def test_clipped_live_view_turns_camera_left_and_requires_new_sample(self):
        r=PenguinRuntime.__new__(PenguinRuntime);r.face_source=Mock();r.record=Mock();s=self.view_snapshot()
        self.assertTrue(r.recenter_clipped_detection(dict(box=[0.,201.,75.57,296.]),s))
        self.assertAlmostEqual(r.face_source.call_args.args[0][0],.4712,places=3)
        self.assertEqual(r.face_source.call_args.args[0][1],-.5)
        self.assertFalse(r.recenter_clipped_detection(dict(box=[100.,201.,175.,296.]),s))
        self.assertEqual(r.face_source.call_count,1)
    def test_unconfirmed_identity_gets_new_views_without_lowering_threshold(self):
        r=PenguinRuntime.__new__(PenguinRuntime);r.face_source=Mock();r.record=Mock();s=self.view_snapshot()
        s['joints']['head_pan_joint']=.47148
        detections=[dict(name='toy_penguin',confidence=.7566,box=[237.29,207.,330.74,296.])]
        self.assertTrue(r.improve_unconfirmed_source_view(detections,s))
        self.assertAlmostEqual(r.face_source.call_args.args[0][0],.53634,places=3)
        self.assertEqual(r.record.call_args.args[0]['identity_threshold_unchanged'],.8)
        other=PenguinRuntime.__new__(PenguinRuntime);other.face_source=Mock();other.record=Mock()
        self.assertFalse(other.improve_unconfirmed_source_view([dict(detections[0],name='canned_juice')],s))
        for _ in range(5):r.improve_unconfirmed_source_view(detections,s)
        self.assertLessEqual(r.face_source.call_count,3)
    def test_low_score_duplicate_proposals_can_guide_a_new_view_without_confirming_identity(self):
        r=PenguinRuntime.__new__(PenguinRuntime);r.face_source=Mock();r.record=Mock();s=self.view_snapshot()
        first=dict(name='toy_penguin',confidence=.371,box=[111.54,162.72,198.08,236.64])
        duplicate=dict(first,confidence=.103)
        self.assertTrue(r.improve_unconfirmed_source_view([first,duplicate],s))
        goal=r.face_source.call_args.args[0];self.assertGreater(goal[0],.25)
        self.assertFalse(r.record.call_args.args[0]['proposal_is_identity_confirmation'])
        s['joints']['head_pan_joint']=goal[0]
        self.assertTrue(r.improve_unconfirmed_source_view([],s))
        self.assertEqual(r.record.call_args.args[0]['identity_threshold_unchanged'],.8)
    def test_tracking_recenters_view_only_outside_central_band(self):
        r=PenguinRuntime.__new__(PenguinRuntime);r.face_source=Mock();r.record=Mock();s=self.view_snapshot()
        self.assertFalse(r.recenter_target_view([0.,0.,1.],[0.,0.,1.1],s))
        self.assertTrue(r.recenter_target_view([.5,0.,1.],[.5,0.,1.1],s))
        self.assertLess(r.face_source.call_args.args[0][0],0.)
        s['joints']['head_pan_joint']=.6
        self.assertIsNone(view_goal_for_pixel(-200.,s))
    def test_contact_error_targets_current_head_instead_of_body(self):
        r=PenguinRuntime.__new__(PenguinRuntime);s=dict(streams=dict(depth=dict(stamp_ns=123)),palm_base_at_depth=[.6,.078,.55])
        target=contact_point(s);body=target-[0,0,.065]
        m=dict(class_name='toy_penguin',stamp_ns=123,center_m=body.tolist(),grasp_center_m=target.tolist())
        r.accept_geometry(m,s);np.testing.assert_allclose(r.contact_error(body,s),[0,0,0],atol=1e-12)
        with self.assertRaises(ValueError):r.contact_error(target,s)
    def test_default_cli_never_moves_and_reports_the_new_class(self):
        proc=subprocess.run([sys.executable,str(Path(__file__).with_name('run_object_transfer.py')),'--object','toy_penguin'],capture_output=True,text=True,timeout=15)
        self.assertEqual(proc.returncode,0,proc.stderr);d=json.loads(proc.stdout)
        self.assertEqual(d['class_name'],'toy_penguin');self.assertFalse(d['robot_commands_sent'])
        self.assertEqual(d['configured_task']['object'],'toy_penguin');self.assertEqual(object_profile('toy_penguin')['class_id'],24)
    def test_task_rejects_wrong_class(self):
        task=load_task();task['object']='ground_pepper'
        with self.assertRaises(ValueError):validate_task(task)
    def test_head_height_uses_existing_empty_arm_and_grip_controls(self):
        r=PenguinRuntime.__new__(PenguinRuntime);r.state=Mock(return_value=dict(joints=[0.,0.,0.,-math.pi/2,0.,.65]));r.move_joints=Mock()
        arm=r.prepare(.322,.1035)
        self.assertEqual(arm,[.322,-1.7,0.,.1292036732051034,0.]);self.assertEqual(r.move_joints.call_count,2)
        self.assertEqual(r.vertical_alignment_tolerance_m,.004)

if __name__=='__main__':unittest.main()
