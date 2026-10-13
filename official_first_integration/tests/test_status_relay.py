import json,socket,struct,sys,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from status_relay import RelayFrames,StatusRelay,REMOTE_CODE

def frame(ns,kind=1002,cmd=4):
    return (json.dumps(dict(kind='state',received_ns=ns,message_id=7,
        PatrolDevice=dict(Type=kind,Command=cmd,Items=dict(MotionStatus=dict(Height=.57)))))+'\n').encode()

class RelayTests(unittest.TestCase):
    def test_fragmented_and_coalesced_frames_keep_source_age(self):
        p=RelayFrames();one=frame(10_000_000_000)
        self.assertEqual(p.feed(one[:15],10_200_000_000,100.),[])
        out=p.feed(one[15:]+frame(10_100_000_000),10_200_000_000,100.)
        self.assertEqual(len(out),2);self.assertAlmostEqual(out[0][2],99.8);self.assertAlmostEqual(out[1][3],.1)

    def test_old_future_reordered_or_invalid_state_cannot_refresh(self):
        p=RelayFrames();self.assertEqual(p.feed(frame(8_000_000_000),10_000_000_000,100.),[])
        self.assertEqual(p.feed(frame(11_000_000_000),10_000_000_000,100.),[])
        self.assertEqual(len(p.feed(frame(10_000_000_000),10_000_000_000,100.)),1)
        self.assertEqual(p.feed(frame(9_900_000_000)+b'[]\n',10_000_000_000,100.),[])
        self.assertEqual(p.stats['rejected_stale'],1);self.assertEqual(p.stats['rejected_future'],1)
        self.assertEqual(p.stats['rejected_order'],1);self.assertEqual(p.stats['rejected_invalid'],1)

    def test_delayed_motion_does_not_become_fresh_on_receipt(self):
        from robot import Gate
        gate=Gate();out=RelayFrames().feed(frame(10_000_000_000),10_500_000_000,100.)
        gate.receive(out[0][1],out[0][2]);self.assertAlmostEqual(gate.motion_at,99.5)
        gate.basic=dict(ControlUsageMode=1,HES=0,MotionState=17,Direction=0,Sleep=0)
        gate.basic_at=100.;gate.command_at=100.;gate.command=(1.,0.,0.)
        self.assertEqual(gate.stop_reason(100.),'stale_motion_status')
        self.assertEqual(gate.output(100.),(0.,0.,0.))

    def test_remote_worker_sends_only_heartbeat_and_preserves_kernel_time(self):
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sock.setblocking(False)
        code=REMOTE_CODE.replace("('10.21.33.103',30000)",repr(sock.getsockname()))
        relay=StatusRelay(command=[sys.executable,'-u','-c',code]);received=[];heartbeats=0;start=time.monotonic();peer=None;last_send=0.
        try:
            while time.monotonic()-start<1.8:
                received.extend(relay.poll())
                try:
                    data,peer=sock.recvfrom(65536);doc=json.loads(data[16:])['PatrolDevice']
                    self.assertEqual((doc['Type'],doc['Command']),(100,100));heartbeats+=1
                except BlockingIOError:pass
                if peer and time.monotonic()-last_send>.1:
                    body=json.dumps(dict(PatrolDevice=dict(Type=1002,Command=4,Items=dict(MotionStatus=dict(Height=.57))))).encode()
                    sock.sendto(struct.pack('<4sHHB7x',bytes.fromhex('eb91eb90'),len(body),0,1)+body,peer);last_send=time.monotonic()
                time.sleep(.005)
            self.assertGreaterEqual(heartbeats,2);self.assertGreater(len(received),8)
            self.assertTrue(relay.connected);self.assertLess(max(x[3] for x in received),.2)
            relay.process.kill();relay.process.wait(timeout=1);relay.poll();self.assertFalse(relay.connected)
            # Loss invalidates the channel; reconnect must establish a new
            # heartbeat subscription and receive new frames, not reuse cache.
            restarted=[];deadline=time.monotonic()+2.5
            while time.monotonic()<deadline and not restarted:
                restarted.extend(relay.poll())
                try:
                    data,new_peer=sock.recvfrom(65536);doc=json.loads(data[16:])['PatrolDevice']
                    self.assertEqual((doc['Type'],doc['Command']),(100,100))
                    sock.sendto(struct.pack('<4sHHB7x',bytes.fromhex('eb91eb90'),len(body),0,1)+body,new_peer)
                except BlockingIOError:pass
                time.sleep(.005)
            self.assertTrue(restarted);self.assertEqual(relay.attempts,2);self.assertTrue(relay.connected)
        finally:relay.close();sock.close()

if __name__=='__main__':unittest.main()
