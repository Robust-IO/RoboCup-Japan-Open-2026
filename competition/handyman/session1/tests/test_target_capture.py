import tempfile
import unittest
from pathlib import Path
import numpy as np
from target_capture import TargetCapture
class Runtime:
    def __init__(self,root):self.root=root
    def save(self,item,label):p=self.root/label;p.mkdir();return p
    def pump(self,t):pass
class Backend:
    def __init__(self,root):self.rt=Runtime(root);self.stamp=20
    def _sample(self):return dict(depth=np.ones((10,10)),snapshot={'streams':{'depth':{'stamp_ns':self.stamp}}})
    def _cloud(self,item):
        x,y=np.meshgrid(np.arange(10)*.004,np.arange(10)*.004)
        p=np.c_[x.ravel(),y.ravel(),np.full(100,.415)]
        return p,np.zeros((100,3)),np.arange(100)
class Worker:
    def __init__(self):self.rows=[dict(name='dog_doll',confidence=.9,box=[0,0,10,10])]
    def infer(self,path,**kw):return {'detections':self.rows}
class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.b=Backend(Path(self.tmp.name));self.w=Worker()
        d=dict(id='table',axis_aligned_bounds={'center':{'x':0,'y':0},'size':{'x':1,'y':1}})
        def plane(depth,s,d):return dict(destination_id=d['id'],source_depth_stamp_ns=s['streams']['depth']['stamp_ns'],plane_map=[0,0,.4])
        self.c=TargetCapture(self.b,self.w,d,measure=plane,map_point=lambda s,p:p,component=lambda p:np.ones(len(p),bool))
    def test_independent_points_saved(self):
        r=self.c.capture(release_stamp_ns=10)
        self.assertEqual(r['geometry_source'],'independent_target_RGBD');self.assertTrue(r['bottom_observed'])
        self.assertTrue((Path(r['evidence_ref'])/'target-surface.json').exists())
    def test_old_frame_never_infers(self):
        with self.assertRaises(ValueError):self.c.capture(release_stamp_ns=20)
        self.assertEqual(list(Path(self.tmp.name).iterdir()),[])
    def test_wrong_class_not_accepted(self):
        self.w.rows[0]['name']='apple'
        with self.assertRaises(ValueError):self.c.capture(release_stamp_ns=10)
    def test_duplicate_boxes_same_surface(self):
        self.w.rows*=2
        self.assertEqual(self.c.capture(release_stamp_ns=10)['target'],'dog_doll')
    def test_post_release_low_confidence_identity(self):
        self.w.rows[0]['confidence']=.525
        self.assertEqual(self.c.capture(release_stamp_ns=10)['target'],'dog_doll')
    def test_threshold_requires_strictly_over_forty_percent(self):
        for confidence in (.39,.4,float('nan')):
            with self.subTest(confidence=confidence):
                self.w.rows[0]['confidence']=confidence
                self.b.stamp+=1
                with self.assertRaisesRegex(ValueError,'placed_target_not_uniquely'):
                    self.c.capture(release_stamp_ns=10)
