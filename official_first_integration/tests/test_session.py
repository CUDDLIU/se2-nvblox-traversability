"""Launcher execution selection and real subprocess shutdown ordering."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

HERE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('first_session',HERE/'run.py')
session=importlib.util.module_from_spec(spec);spec.loader.exec_module(session)

class SessionTests(unittest.TestCase):
    def test_explicit_execution_and_private_channels(self):
        a,b=session.session_remaps('a'),session.session_remaps('b')
        self.assertTrue(set(a.values()).isdisjoint(b.values()))
        readonly=session.robot_command(False,False,Path('/tmp/log'),a)
        execute=session.robot_command(True,True,Path('/tmp/log'),b)
        self.assertIn('--telemetry-only',readonly);self.assertNotIn('--execute',readonly)
        self.assertIn('--execute',execute);self.assertIn('--enter-navigation',execute)
        self.assertNotIn('--telemetry-only',execute)
        self.assertIn('/first/cmd_vel_stamped:=/first/session_b/command',execute)

    def test_sender_stops_before_slow_gui(self):
        with tempfile.TemporaryDirectory() as d:
            code='''import signal,time,sys
from pathlib import Path
d=Path(sys.argv[1]);name=sys.argv[2]
def stop(*_):
    with (d/'events').open('a') as f:f.write(name+'\\n')
    if name=='rviz':time.sleep(10)
    raise SystemExit(0)
signal.signal(signal.SIGINT,stop)
(d/name).touch()
while True:time.sleep(.02)
'''
            children={}
            try:
                for name in ('robot','first','rviz'):
                    children[name]=subprocess.Popen([sys.executable,'-c',code,d,name],start_new_session=True)
                until=time.monotonic()+3
                while not all((Path(d)/n).exists() for n in children) and time.monotonic()<until:time.sleep(.01)
                self.assertTrue(all((Path(d)/n).exists() for n in children))
                began=time.monotonic();self.assertEqual(session.stop_children(children),[])
                self.assertLess(time.monotonic()-began,4)
                self.assertEqual((Path(d)/'events').read_text().splitlines()[0],'robot')
                self.assertTrue(all(p.poll() is not None for p in children.values()))
            finally:session.stop_children(children)

if __name__=='__main__':unittest.main()
