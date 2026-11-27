import unittest
from task_profile import resolve
class TaskProfileTests(unittest.TestCase):
    def data(self):
        p=dict(schema='handyman_session_profile_v1',session_number=2,layout='LayoutB',target='canned_juice',source=dict(room='living_room',furniture_id='wooden_side_table#living_room'),destination=dict(room='living_room',furniture_id='trash_box_for_bottle_can#living_room',mode='container_drop'))
        s={'destinations':[{'id':x+'#'+room,'room':room} for x in ('wooden_side_table','trash_box_for_bottle_can') for room in ('living_room','kitchen')]}
        e={'destinations':{x:[dict(room=room,x=0,y=0,yaw=0) for room in ('living_room','kitchen')] for x in ('wooden_side_table','trash_box_for_bottle_can')}}
        return p,s,e
    def test_room_qualified_container_and_not_live_ready(self):
        r=resolve(*self.data());self.assertEqual(r['resolved']['destination']['furniture']['room'],'living_room');self.assertEqual(r['destination_mode'],'container_drop');self.assertFalse(r['execution_ready'])
    def test_missing_correct_room_never_falls_back_to_other_room(self):
        p,s,e=self.data();s['destinations']=[x for x in s['destinations'] if x['id']!='trash_box_for_bottle_can#living_room']
        with self.assertRaises(ValueError):resolve(p,s,e)
