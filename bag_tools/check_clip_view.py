"""Hold a rebuilt stair clip for a manual RViz camera check."""
import sys
import time
import subprocess
from pathlib import Path
from bag_manager import BagManager

manager = BagManager()
stop = manager.root / 'diagnostics/stair_clip_view.stop'
stop.unlink(missing_ok=True)
try:
    manager.play(sys.argv[1], rebuild=True, sensor_mode='fusion',
                 settings={'max_step': .2, 'max_slope_deg': 40.})
    print(str(manager.session), flush=True)
    for _ in range(int(sys.argv[2]) if len(sys.argv) > 2 else 65):
        time.sleep(1)
        manager.tick()
    manager.toggle_pause()
    print('PAUSED_FOR_VIEW_CHECK', flush=True)
    time.sleep(1)
    subprocess.run(['gnome-screenshot', '-f', str(manager.session / 'clip_view.png')],
                   timeout=15, check=True)
    for _ in range(int(sys.argv[3]) if len(sys.argv) > 3 else 240):
        if stop.exists():
            break
        time.sleep(1)
        manager.tick()
finally:
    manager.stop_play()
    manager.restore_live()
    manager.close()
    stop.unlink(missing_ok=True)
