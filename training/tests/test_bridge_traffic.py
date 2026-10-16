import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from probe_bridge_traffic import parse_ss,changes

class TrafficTests(unittest.TestCase):
    def row(self,n):return dict(local='192.168.0.2:50001',peer='192.168.0.1:60077',pid=45,fd=19,bytes_received=n)
    def test_parse(self):
        text='State Recv-Q Send-Q Local Peer\nESTAB 0 0 192.168.0.2:50001 192.168.0.1:60077 users:(("bridge",pid=45,fd=19))\n\t cubic bytes_received:12345 segs_in:55\n'
        r=parse_ss(text)[0]
        self.assertEqual((r['bytes_received'],r['pid'],r['fd']),(12345,45,19))
    def test_delta(self):
        r=changes([self.row(10)],[self.row(50)],2)[0]
        self.assertEqual((r['received_bytes_delta'],r['bytes_per_second']),(40,20))
    def test_reset(self):self.assertEqual(changes([self.row(50)],[self.row(10)],1)[0]['delta_status'],'counter_reset')
    def test_new(self):self.assertNotIn('received_bytes_delta',changes([],[self.row(10)],1)[0])
    def test_other_port_ignored(self):self.assertEqual(parse_ss('ESTAB 0 0 127.0.0.1:22 127.0.0.1:100\n\t bytes_received:20'),[])

if __name__=='__main__':unittest.main()
