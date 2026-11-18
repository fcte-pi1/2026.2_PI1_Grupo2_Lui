import contextlib
import io
import json
import unittest
from receber_telemetria import receive_frames

class Clock:
    def __init__(self): self.now = 0.0
    def __call__(self): self.now += .001; return self.now

class Link:
    def __init__(self, packets, bad=b''):
        self.packets=packets; self.bad=bad; self.pending=b''
    def write(self, data):
        token=data.decode().strip().split()[1]
        self.pending=(json.dumps({'type':'pong','token':token})+'\n').encode()+self.bad
        self.pending+=b''.join((json.dumps(p)+'\n').encode() for p in self.packets)
    def read(self, count):
        chunk=self.pending[:7]; self.pending=self.pending[7:]; return chunk

def packet(seq=1):
    return {'type':'bt_telemetry_test','seq':seq,'uptime_ms':seq*250,
            'motors_enabled':False,'ultrasonic_enabled':False,
            'imu':{'valid':True,'yaw_deg':0.1,'gyro_z_dps':0.2}}

class ReceiverTests(unittest.TestCase):
    def run_link(self, packets, bad=b''):
        capture=io.StringIO()
        with contextlib.redirect_stdout(io.StringIO()):
            result=receive_frames(Link(packets,bad),1,capture,3,Clock())
        return result,capture
    def test_fragmented_frames_and_matching_ping(self):
        summary,capture=self.run_link([packet(1),packet(2),packet(3)])
        self.assertEqual(summary['frames'],3)
        self.assertEqual(summary['sequence_gaps'],0)
        self.assertTrue(summary['ping_pong_ok'])
        self.assertEqual(len(capture.getvalue().splitlines()),4)
    def test_corrupt_json_is_not_accepted_as_success(self):
        with self.assertRaises(RuntimeError): self.run_link([packet(1),packet(2),packet(3)],b'{broken}\n')
    def test_enabled_motor_state_aborts_receiver(self):
        value=packet();value['motors_enabled']=True
        with self.assertRaises(RuntimeError): self.run_link([value])
    def test_duplicate_sequence_is_not_accepted_as_success(self):
        with self.assertRaises(RuntimeError): self.run_link([packet(1),packet(1),packet(2)])

if __name__=='__main__': unittest.main()
