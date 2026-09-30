"""Verify a saved bag through natural EOF and inspect RViz captures."""
import json
import subprocess
import sys
import time
from bag_manager import BagManager

manager = BagManager()
try:
    key = sys.argv[1]
    manager.play(key, .5)
    time.sleep(9)
    manager.toggle_pause()
    subprocess.run(['gnome-screenshot', '-f', str(manager.runtime / 'verified_replay_paused.png')], check=True)
    time.sleep(1)
    manager.toggle_pause()
    deadline = time.monotonic() + 50
    while time.monotonic() < deadline:
        status = manager.tick()
        if status['mode'] == 'finished':
            break
        time.sleep(.5)
    assert status['mode'] == 'finished', status
    subprocess.run(['gnome-screenshot', '-f', str(manager.runtime / 'verified_replay_finished.png')], check=True)
    print(json.dumps(status, ensure_ascii=False), flush=True)
finally:
    manager.close()
