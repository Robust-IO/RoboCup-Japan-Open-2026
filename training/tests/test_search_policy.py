import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_policy import room_scan
from rehearse_search import rehearse

def plan(layout,room,connected=True):
    p=dict(layout=layout,frame_id='map',rooms=[dict(room=room,valid=True,errors=[],points=[dict(pose=dict(x=0,y=0,yaw=0),issues=[])])])
    p['map_bundle_sha256']='a'*64
    p['connectivity']=dict(layout=layout,map_bundle_sha256='a'*64,method='optimistic_8_connected_point_agent',initial_known_free=True,
        rooms=[dict(room=room,status='point_connectivity_present' if connected else 'structural_disconnection_candidate',
                    cells=dict(known_free=10,known_connected=10 if connected else 0,optimistic_traversable=10,optimistic_connected=10 if connected else 0))])
    return p

class PolicyTests(unittest.TestCase):
    def test_exclusions_cannot_be_bypassed_by_valid_flag(self):
        for layout,room in [('LayoutB','lobby'),('LayoutC','kitchen')]:
            self.assertIsNone(room_scan(plan(layout,room,False),room,'t','apple')[0])
    def test_missing_room(self): self.assertIsNone(room_scan(plan('LayoutA','bedroom'),'kitchen','t','apple')[0])
    def test_invalid_room(self):
        p=plan('LayoutA','bedroom'); p['rooms'][0]['valid']=False
        self.assertIsNone(room_scan(p,'bedroom','t','apple')[0])
    def test_rehearsal_results(self):
        rows=rehearse([plan('LayoutA','bedroom')])['results']
        self.assertEqual([r['state'] for r in rows],['found','plan_exhausted','incomplete','incomplete','incomplete','cancelled'])
        self.assertTrue(all(not r['does_not_exist_authorized'] and not r['actionable'] for r in rows))
    def test_no_requests_for_excluded_rooms(self):
        rows=rehearse([plan('LayoutB','lobby',False),plan('LayoutC','kitchen',False)])['results']
        self.assertTrue(all(r['state']=='blocked' and not r['requests'] for r in rows))

    def test_old_names_not_hardcoded(self):
        self.assertIsNotNone(room_scan(plan('LayoutB','lobby',True),'lobby','t','apple')[0])
    def test_random_name_disconnected(self):
        self.assertIsNone(room_scan(plan('Random42','room9',False),'room9','t','apple')[0])
    def test_legacy_plan_rejected(self):
        p=plan('A','r'); del p['connectivity']
        self.assertIsNone(room_scan(p,'r','t','apple')[0])
    def test_hash_mismatch(self):
        p=plan('A','r');p['map_bundle_sha256']='b'*64
        self.assertIsNone(room_scan(p,'r','t','apple')[0])

if __name__=='__main__': unittest.main()
