"""Actual stage bindings + transfer actions + app/protocol; synthetic hardware IO."""
import importlib.util
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
import numpy as np
from application import Application
from stage_bindings import StageBindings
from transfer_bindings import TransferBindings
from session1 import INSTRUCTION
from concurrent.futures import Future

root=Path(__file__).resolve().parents[2]/'handyman_common_placement_v1/frozen'
if not root.exists():root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
def module(name):
    spec=importlib.util.spec_from_file_location(name,root/(name+'.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
core=module('table_transfer_core');policy=module('task_outcome_policy')
def done(x):f=Future();f.set_result(x);return f
class Runtime:
    def __init__(self):self.joints=[0,0,0,-1.57,0,.65];self.commands=[]
    def face_source(self,*args):self.commands.append("face_source")
    def sample(self,**kw):pass
    def state(self):return dict(joints=list(self.joints),poses={'map':[1,1,0]})
    def fresh_measurement(self,s):pass
    def record(self,e):pass
    def move_joints(self,g,label,**kw):
        if kw.get('before_publish'):kw['before_publish']()
        self.joints=list(g);self.commands.append(label)
class AssembledFlowTests(unittest.TestCase):
    def test_bound_grasp_to_release_to_verification(self):
        with tempfile.TemporaryDirectory() as root:
            rt=Runtime();published=[];navigation=[]
            context=dict(schema='handyman_held_surface_v1',frame='hand_palm_link',target='dog_doll',source_table='white_side_table#living_room',vertices_palm_m=[[-.04,-.02,-.08],[.04,.02,.03]])
            backend=SimpleNamespace(worker=object(),hand=object(),policy=object(),record=lambda e:None,held_context=context,
                source_snapshot={},table={'id':context['source_table'],'axis_aligned_bounds':{'center':{'x':1,'y':1},'size':{'x':.5,'y':.5}}},
                observe_stopped=lambda target:SimpleNamespace(target=np.array([[1,1,.5],[1.02,1,.5]]),stamp_ns=10),
                _sample=lambda:dict(snapshot={'streams':{'depth':{'stamp_ns':50}},'joints':{'hand_motor_joint':rt.joints[5]}}))
            def grasp(target):
                rt.joints=[.3,-1.7,0,.1292,0,.31]
                return dict(status='grasp_verified',lift_evidence=dict(verified=True,object_followed=True,grip_retained=True,observations=3))
            def navigate(*args):navigation.append(args);return dict(verified=True,evidence='synthetic-nav')
            motion=SimpleNamespace(room_at=lambda p:'living_room',navigate=navigate)
            env=dict(rooms={'living_room':{'region':[[0,0],[2,0],[2,2],[0,2]]}},destinations={'white_side_table':[dict(room='living_room',x=1,y=1,yaw=0)],'square_low_table':[dict(room='living_room',x=1.5,y=1,yaw=0)]})
            stages=StageBindings(rt,motion,backend,{},env,root,map_point=lambda s,p:p,coordinator_factory=lambda *a,**kw:SimpleNamespace(run=grasp))
            def observer(*args,**kw):return SimpleNamespace(measure_and_plan=lambda shape:dict(palm_height_m=core.release_height(.4,shape['bottom_below_palm_m']),destination_id='square_low_table#living_room'),verify_support=lambda p:{'ready':True})
            def verify(plan,*,release_stamp_ns):
                self.assertEqual(release_stamp_ns,50);self.assertGreaterEqual(rt.joints[5],.6)
                return dict(status='ok',target='dog_doll',destination=plan['destination_id'],placement_verified=True)
            transfer=TransferBindings(stages,{'id':'square_low_table#living_room'},np.eye(3),core=core,policy=policy,
                withdraw_source=lambda:{'clear_to_fold':True},withdraw_empty=lambda:{'clear_to_fold':True},bottom_evidence=lambda c:True,
                hold_factory=lambda *a:lambda label:{'holding_verified':True},observer_factory=observer,
                standoff_factory=lambda *a:lambda p:p,approach_factory=lambda *a:lambda p:None,
                capture_factory=lambda *a,**kw:SimpleNamespace(verify=verify))
            def publish(effect):published.append(effect['message']);return done({'status':'sent'})
            app=Application('test',stages.handlers(transfer.handlers()),publish,lambda *a,**kw:done({'motion_stopped':True}))
            app.session.preflight(ready=True);app.receive('Environment','LayoutA',session_id='test');app.receive('Are_you_ready?',session_id='test')
            app.tick();app.tick();app.receive('Instruction',INSTRUCTION,session_id='test')
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                app.tick();time.sleep(.001)
                if app.session.state=='waiting_moderator' and app.executor.retired:break
            self.assertEqual(app.session.state,'waiting_moderator',app.session.snapshot())
            self.assertEqual(published,['I_am_ready','Room_reached','Object_grasped','Task_finished'])
            self.assertEqual(rt.commands.count('release_once'),1);self.assertEqual(len(navigation),1)
            self.assertTrue((Path(root)/'release-record.json').is_file())
            app.receive('Task_succeeded',session_id='test');app.tick();self.assertEqual(app.session.state,'succeeded');app.close()
