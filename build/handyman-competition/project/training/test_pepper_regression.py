import copy
import unittest
from pepper_regression import sequence,verify_leg,STAGES
from pepper_tasks import load_task,task_digest,DEFAULT_TASK,REVERSE_TASK


class RegressionTests(unittest.TestCase):
    def valid(self):
        task=load_task()
        return task,dict(status='automatic_ground_pepper_transfer_verified',class_name='ground_pepper',
            physical_transfer_verified=True,grasp_success_verified=True,placement_success_verified=True,
            robot_ready_for_next_task=True,cleanup_completed=True,object_remains_held=False,
            uninterrupted_transfer_verified=True,close_commands_sent=1,release_commands_sent=1,
            confirmed_stages=STAGES,task_sha256=task_digest(task),configured_task=task,task_finished_sent=False,elapsed_s=123.)
    def test_reset_and_rounds_alternate_without_human_object_moves(self):
        legs=sequence(3)
        self.assertEqual(len(legs),6)
        self.assertEqual([leg['kind'] for leg in legs],['automatic_reset','forward_regression']*3)
        for a,b in zip(legs,legs[1:]):
            self.assertEqual(load_task(a['task_config'])['destination']['table_id'],load_task(b['task_config'])['source']['table_id'])
    def test_uninterrupted_functional_success_counts(self):
        task,result=self.valid();self.assertEqual(verify_leg(result,task)['elapsed_s'],123.)
    def test_staged_cleanup_only_or_repeated_close_cannot_count(self):
        task,result=self.valid()
        for key,value in [('uninterrupted_transfer_verified',False),('close_commands_sent',2),
                ('release_commands_sent',0),('cleanup_completed',False),('confirmed_stages',['place_verify'])]:
            bad=copy.deepcopy(result);bad[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):verify_leg(bad,task)
    def test_old_result_or_other_direction_cannot_count_for_new_task(self):
        task,result=self.valid()
        with self.assertRaises(ValueError):verify_leg(result,load_task(REVERSE_TASK))
        result['task_sha256']='old'
        with self.assertRaises(ValueError):verify_leg(result,task)

if __name__=='__main__':unittest.main()
