"""Jetson GUI acceptance: live RViz -> rebuilt bag -> restore live button."""
import json
from pathlib import Path
import subprocess
import sys
import time
from tkinter import messagebox

sys.path.insert(0, '/home/nvidia/scanplanner_test/se2_terrain_check')
from terrain_tuning_gui import Panel


def service_pid(name):
    return subprocess.check_output(['systemctl', '--user', 'show', name,
                                    '--property=MainPID', '--value'], text=True).strip()


panel = Panel()
widget = panel.bag_widget
manager = widget.manager
key = sys.argv[1] if len(sys.argv) > 1 else None
report = {'errors': []}
messagebox.showerror = lambda title, msg, **kw: report['errors'].append(str(msg))
live_units = ('nvblox-lio', 'nvblox-d435i-shadow', 'se2-terrain-check')
initial = {name: service_pid(name) for name in live_units}
assert all(pid != '0' for pid in initial.values()), initial
assert manager._live_rviz_active()
stage = 'start'
started = time.monotonic()
record_started = None


def step():
    global stage, key, record_started
    try:
        if time.monotonic() - started > 150 or report['errors']:
            raise RuntimeError(f'{stage}: {widget.status.get()}; {report["errors"]}')
        if not widget.busy:
            if stage == 'start' and key is None:
                widget.title.set('验收：实时与回放切换')
                widget.lidar.set(False)
                widget.new_map.set(False)  # This test verifies preserving an existing live session.
                widget.record_button.invoke()
                record_started = time.monotonic()
                stage = 'recording'
            elif stage == 'recording' and manager.mode == 'record' and time.monotonic() - record_started > 8:
                key = manager.active.name
                report['bag'] = key
                widget.stop_record_button.invoke()
                stage = 'start'
            elif stage == 'start' and key in widget.rows:
                widget.tree.selection_set(key)
                widget.rate.set('0.5')
                widget.play_button.invoke()
                stage = 'playing'
            elif stage == 'playing' and panel.target == 'replay' and panel.ready and panel.last_data and panel.last_data.get('mesh_messages', 0) > 0:
                assert not manager._live_rviz_active(), 'Live RViz was not closed'
                assert manager.children['rviz'].poll() is None, 'Replay RViz exited'
                report['live_window_closed_during_replay'] = True
                widget.pause_button.invoke()
                stage = 'paused'
            elif stage == 'paused' and manager.paused:
                widget.restore_live_button.invoke()
                stage = 'restored'
            elif stage == 'restored' and manager.mode == 'idle' and panel.target == 'live' and panel.ready:
                assert manager._live_rviz_active()
                assert not manager.children, manager.children
                assert not manager.live_suspended.exists()
                assert initial['nvblox-lio'] == service_pid('nvblox-lio')
                assert all(service_pid(name) != '0' for name in live_units)
                report.update(replay_children_stopped=True, live_target_ready=True,
                              live_mapping_restarted=True, lio_pid_preserved=True)
                report['rviz_pid'] = service_pid('se2-terrain-rviz')
                widget.restore_live_button.invoke()
                stage = 'repeat'
            elif stage == 'repeat':
                assert service_pid('se2-terrain-rviz') == report['rviz_pid']
                report.update(repeated_restore_no_duplicate=True, result='PASS')
                finish()
                return
        panel.root.after(250, step)
    except Exception as exc:
        report.update(result='FAIL', error=str(exc), stage=stage)
        finish()


def finish():
    (manager.runtime / 'live_switch_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False), flush=True)
    panel.close()


panel.root.after(1500, step)
panel.root.mainloop()
sys.exit(0 if report.get('result') == 'PASS' else 1)
