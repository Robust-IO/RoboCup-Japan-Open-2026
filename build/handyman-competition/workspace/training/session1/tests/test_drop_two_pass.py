import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from concurrent.futures import Future
from threading import Event
from carry_drop_recovery import search_dropped_target,DroppedTargetNotFound,FLOOR_VIEWS
from application import WorkerPort
from executor import Executor
from session1 import Session1
from dynamic_session import DynamicSession

class SearchTests(unittest.TestCase):
    def stages(self):
        state={'poses':{'map':[1,2,.7]},'joints':[0,0,0,0,0,.1]}
        return NS(grasp_backend=NS(worker=object(),record=Mock()),runtime=NS(sample=Mock(),state=lambda:state),motion=NS(stage='navigate_destination',adjust=Mock()),check=Mock(),target='filled_ketchup',map_point=object())
    def frames(self,found_at=None):
        return [dict(depth_stamp_ns=i+1,floor_visible=True,evidence_ref=str(i),floor_targets=([dict(target='filled_ketchup',center_map_m=[1,2,.1])] if found_at is not None and i in (found_at,found_at+1) else [])) for i in range(12)]
    def run_search(self,s,frames):
        scan=Mock(side_effect=frames)
        with patch('container_floor_capture.ContainerFloorCapture',return_value=scan) as factory:
            result=search_dropped_target(s,Event(),[1,2,0])
        return result,scan,factory
    def test_first_pass_found_no_retreat(self):
        s=self.stages();row,scan,_=self.run_search(s,self.frames(2))
        self.assertEqual(scan.call_count,4);s.motion.adjust.assert_not_called();self.assertEqual(row['target'],s.target)
    def test_second_pass_found_one_half_meter_retreat(self):
        s=self.stages();row,scan,factory=self.run_search(s,self.frames(8))
        self.assertEqual(scan.call_count,10);s.motion.adjust.assert_called_once()
        self.assertEqual(s.motion.adjust.call_args.args[2],[-.5,0.,0.]);self.assertEqual(s.motion.stage,'navigate_destination')
        self.assertEqual(factory.call_args.kwargs['views'],FLOOR_VIEWS)
    def test_exhausted_exactly_twelve_frames(self):
        s=self.stages();scan=Mock(side_effect=self.frames())
        with patch('container_floor_capture.ContainerFloorCapture',return_value=scan):
            with self.assertRaises(DroppedTargetNotFound):search_dropped_target(s,Event(),[1,2,0])
        self.assertEqual(scan.call_count,12);s.motion.adjust.assert_called_once()
    def test_no_pair_across_views(self):
        f=self.frames();f[1]['floor_targets']=[dict(target='filled_ketchup',center_map_m=[1,2,.1])];f[2]['floor_targets']=f[1]['floor_targets']
        with self.assertRaises(DroppedTargetNotFound):self.run_search(self.stages(),f)
    def test_cancel_before_retreat(self):
        s=self.stages();s.check.side_effect=[None]*6+[RuntimeError('cancelled')]
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.run_search(s,self.frames())
        s.motion.adjust.assert_not_called()
    def test_all_angles_supported(self):
        for pan,tilt in FLOOR_VIEWS:self.assertTrue(-.6<=pan<=.6 and -.8<=tilt<=0)

class ProtocolTests(unittest.TestCase):
    def test_fixed_and_dynamic_give_up_once_after_motion_retirement(self):
        from ros_protocol import OUTGOING
        self.assertIn('Give_up',OUTGOING)
        for session in (Session1('s'),DynamicSession('s',profile={'layout':'LayoutD'})):
            session._request('navigate_destination');effect=session.drain()[0]
            result=Future();result.set_result({'status':'failed','reason':'DroppedTargetNotFound:floor_target_not_found_after_retreat'})
            proof=Future();stop=Mock(return_value=proof)
            port=WorkerPort(None,None,stop)
            handle={'future':result,'cancel':Event(),'stop_future':None,'stop_required':False,'effect':effect}
            proto=Mock();proto.start.return_value='p';proto.poll.return_value={'status':'sent'}
            exe=Executor(session,{k:port for k in session.steps},proto);exe.active=(port,handle,effect)
            exe.tick();self.assertEqual(session.state,'navigate_destination');proto.start.assert_not_called()
            proof.set_result({'motion_stopped':True});exe.tick();self.assertEqual(session.state,'waiting_give_up')
            exe.tick();exe.tick();exe.tick();proto.start.assert_called_once()
            self.assertEqual(proto.start.call_args.args[0]['message'],'Give_up')
            session.moderator('Task_failed','Give_up',session_id='s');exe.tick();self.assertEqual(session.state,'failed')
