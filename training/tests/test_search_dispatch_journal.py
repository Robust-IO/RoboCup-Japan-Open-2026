import copy
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_dispatch_journal import DispatchJournal


def row():
    return dict(schema='handyman-owned-goal-v1',task_id='1'*32,goal_id='2'*32,
        point_id='LayoutA/kitchen/0',map_sha256='a'*64,generation=1,frame_id='map',
        role='final_search_point',pose=dict(x=1.,y=2.,yaw=0.))


class JournalTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.path=Path(self.temp.name)/'journal.sqlite3'
        self.journal=DispatchJournal(self.path)
    def tearDown(self):
        self.journal.close();self.temp.cleanup()
    def test_commit_reopens(self):
        self.journal.record(row(),'/nav');self.journal.close();self.journal=DispatchJournal(self.path)
        self.assertEqual(self.journal.rows(True)[0]['goal_id'],'2'*32)
    def test_exclusive_owner(self):
        with self.assertRaises(BlockingIOError):DispatchJournal(self.path)
    def test_duplicate_and_conflict(self):
        self.assertTrue(self.journal.record(row(),'/nav'));self.assertFalse(self.journal.record(row(),'/nav'))
        other=row();other['pose']['x']=7.
        with self.assertRaises(ValueError):self.journal.record(other,'/nav')
        with self.assertRaises(ValueError):self.journal.record(row(),'/other')
    def test_unknown_never_resolves(self):
        self.journal.record(row(),'/nav')
        for status in (0,1,2,3,True):
            with self.assertRaises(ValueError):self.journal.resolve('2'*32,status)
        self.assertEqual(len(self.journal.rows(True)),1)
    def test_terminal_durable(self):
        self.journal.record(row(),'/nav');self.journal.resolve('2'*32,5)
        self.journal.close();self.journal=DispatchJournal(self.path)
        self.assertFalse(self.journal.rows(True))
        with self.assertRaises(ValueError):self.journal.resolve('2'*32,4)
    def test_explicit_rejection_only(self):
        self.journal.record(row(),'/nav');self.journal.resolve('2'*32,0,rejected=True)
        self.assertFalse(self.journal.rows(True))
    def test_bad_identifiers(self):
        for key,value in [('goal_id','0'*32),('generation',True),('map_sha256','bad'),('pose',dict(x=float('nan'),y=0,yaw=0))]:
            bad=copy.deepcopy(row());bad[key]=value
            with self.assertRaises(ValueError):self.journal.record(bad,'/nav')
    def test_checksum_tampering(self):
        self.journal.record(row(),'/nav')
        with self.journal.db:self.journal.db.execute("UPDATE intents SET checksum='invalid'")
        with self.assertRaises(ValueError):self.journal.rows()


if __name__=='__main__':unittest.main()
