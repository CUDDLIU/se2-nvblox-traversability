"""Exercise the real tuning GUI's new default on the latest complete bag."""
import json
from pathlib import Path
import re
import subprocess
import sys
import time

import yaml

sys.path.insert(0, '/home/nvidia/scanplanner_test/se2_terrain_check')
from terrain_tuning_gui import Panel

key = '20260929_155717_4a77a9'
panel = Panel()
widget = panel.bag_widget
manager = widget.manager
out = manager.root / 'diagnostics/stair_defaults_20260929'
out.mkdir(parents=True, exist_ok=True)
began = time.monotonic()
stage = 'ready'
session = None
reset_checked = False
last_report = 0.
report = {'bag': key, 'scope': 'GUI default rebuild, recorded sensor data and poses'}


def finish(error=None):
    report['result'] = 'FAIL' if error else 'PASS'
    if error:
        report['error'] = str(error)
    (out / 'gui_acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False), flush=True)
    panel.close()


def step():
    global stage, session, reset_checked, last_report
    try:
        if time.monotonic() - began > 450:
            raise RuntimeError('GUI acceptance timeout: ' + widget.status.get())
        if stage == 'ready' and not widget.busy and key in widget.rows:
            assert widget.sensor_mode.get() == '纯雷达'
            assert widget.rebuild.get()
            widget.tree.selection_set(key)
            widget.play_button.invoke()
            stage = 'starting'
        elif stage == 'starting' and not widget.busy:
            assert manager.mode == 'play', widget.status.get()
            session = manager.session
            report['session'] = str(session)
            print('REPLAY ' + str(session), flush=True)
            stage = 'playing'
        elif stage == 'playing':
            if panel.target == 'replay' and panel.ready and not reset_checked:
                # The existing reset button must restore the stair profile too.
                panel.defaults()
                assert panel.desired()['max_step'] == .2
                assert panel.desired()['max_slope_deg'] == 40.
                reset_checked = True
            if time.monotonic() - last_report > 30:
                last_report = time.monotonic()
                d = panel.last_data or {}
                print(json.dumps({'mode': manager.mode, 'elapsed_wall': round(time.monotonic()-began),
                                  'config': d.get('config'), 'counts': d.get('display_counts'),
                                  'latency_ms': d.get('latency_ms')}), flush=True)
            if manager.mode == 'idle':
                raise RuntimeError(manager.message)
            if manager.mode == 'finished':
                assert reset_checked
                assert manager.children['play'].returncode == 0
                assert all(manager.children[n].poll() is None for n in ('terrain', 'deskew', 'nvblox', 'rviz'))
                report['final_status'] = panel.last_data
                subprocess.run(['gnome-screenshot', '-f', str(out/'replay_finished.png')], check=True, timeout=15)
                widget.stop_play_button.invoke()
                stage = 'stopping'
        elif stage == 'stopping' and not widget.busy:
            cfg = yaml.safe_load((session/'terrain_config.yaml').read_text())['se2_terrain_check']['ros__parameters']
            assert (cfg['max_step'], cfg['max_slope_deg'], cfg['tile_cells']) == (.2, 40., 8)
            assert cfg['surface_normal_filter'] and cfg['stair_riser_filter']
            assert 'TERRAIN_IMPLEMENTATION '+str(manager.root/'terrain_variants/local_window') in (session/'terrain.log').read_text()
            log = (session/'nvblox.log').read_text(errors='replace')
            report['lidar_integrated'] = int(re.findall(r'^ros/lidar/integration\s+(\d+)\s', log, re.M)[-1])
            final = re.findall(r'DESKEW_FINAL (.*)', (session/'deskew.log').read_text())[-1]
            report['deskew'] = {k: float(v) for k, v in re.findall(r'(\w+)=([\d.]+)', final)}
            assert report['deskew']['received'] == 3266
            assert report['deskew']['dropped'] <= 3
            assert report['lidar_integrated'] == report['deskew']['frames']
            report['reset_button_stair_defaults'] = reset_checked
            report['effective_config'] = cfg
            finish()
            return
        panel.root.after(500, step)
    except Exception as exc:
        finish(exc)


panel.root.after(1000, step)
panel.root.mainloop()
