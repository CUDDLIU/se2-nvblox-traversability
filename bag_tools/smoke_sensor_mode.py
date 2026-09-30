"""Exercise a selected GUI replay mode, RViz, paused tuning, export and cleanup.

Requires the live GPU services to already be inactive. Does not start them.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from tkinter import messagebox

from rebuild_config import MODES

root = Path(__file__).resolve().parents[1]
for unit in ('se2-terrain-tuner', 'nvblox-d435i-shadow', 'se2-terrain-check', 'se2-terrain-rviz'):
    if subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit]).returncode == 0:
        raise RuntimeError('Live service must already be inactive: ' + unit)
key, mode = sys.argv[1:3]
assert mode in MODES
sys.path.insert(0, str(root.parent / 'se2_terrain_check'))
from terrain_tuning_gui import Panel

panel = Panel()
widget = panel.bag_widget
assert widget is not None
panel.notebook.select(1)
report = dict(mode=mode, errors=[], scope='GUI smoke, not full-bag performance or connectivity acceptance')
messagebox.showerror = lambda title, msg, **kw: report['errors'].append(str(msg))
protected = [root.parent / 'se2_terrain_check' / f for f in ('paper_config.yaml', 'paper.rviz', 'start_after_boot.sh')]
digest = lambda: {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in protected}
before = digest()
def mesh_count():
    data = panel.last_data or {}
    return data.get('mesh_messages', data.get('input_sequence', 0))

stage = 'start'
started = entered = time.monotonic()
exit_status = 1


def finish(error=None):
    global exit_status
    report.update(result='FAIL' if error else 'PASS', error=error, stage=stage,
                  session=str(widget.manager.session))
    exit_status = 1 if error else 0
    (widget.manager.runtime / f'sensor_mode_{mode}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False), flush=True)
    panel.close()


def step():
    global stage, entered
    try:
        now = time.monotonic()
        if now - started > 180 or report['errors']:
            raise RuntimeError(f'{stage}: {widget.status.get()}; {report["errors"]}')
        if widget.busy:
            panel.root.after(250, step)
            return
        if stage == 'start' and key in widget.rows:
            widget.tree.selection_set(key)
            widget.rebuild.set(True)
            widget.sensor_mode.set(MODES[mode])
            widget.rate.set('1')
            widget.play_button.invoke()
            stage, entered = 'play', now
        elif stage == 'play' and widget.manager.mode == 'play' and panel.target == 'replay' and panel.ready:
            assert widget.manager.sensor_mode == mode
            assert str(widget.sensor_selector['state']) == 'disabled'
            report['session_manifest'] = json.loads((widget.manager.session / 'replay.json').read_text())
            report['children'] = {name: proc.pid for name, proc in widget.manager.children.items()}
            report['mesh_version_before'] = mesh_count()
            stage, entered = 'collect', now
        elif stage == 'collect' and now - entered > 15 and panel.last_data:
            if mesh_count() <= report['mesh_version_before']:
                raise RuntimeError('No new Mesh input in GUI')
            report['mesh_messages'] = mesh_count()
            widget.pause_button.invoke()
            stage, entered = 'pause', now
        elif stage == 'pause' and widget.manager.paused and now - entered > 2:
            report['old_config_version'] = panel.last_data['config_version']
            panel.vars['max_slope_deg'].set(0.)
            panel.changed('max_slope_deg')
            stage, entered = 'tune', now
        elif stage == 'tune' and panel.last_data.get('config_version', 0) > report['old_config_version'] and now - entered > 3:
            assert panel.config.get('max_slope_deg') == 0.
            widget.submit(widget.manager.save_comparison, f'界面验证：{MODES[mode]}，暂停后调参')
            stage, entered = 'save', now
        elif stage == 'save' and now - entered > 2:
            saved = list((widget.manager.session / 'comparisons').glob('*.json'))
            assert len(saved) == 1
            result = json.loads(saved[0].read_text())
            assert result['sensor_mode'] == mode
            assert result['source_publishers']['/nvblox_node/mesh'] == ['nvblox_node']
            assert result['parameters']['max_slope_deg'] == 0.
            report['snapshot'] = str(saved[0])
            panel.notebook.select(1)
            panel.root.attributes('-topmost', True)
            panel.root.lift()
            panel.root.update_idletasks()
            stage, entered = 'gui', now
        elif stage == 'gui' and now - entered > 2:
            subprocess.run(['gnome-screenshot', '-f', str(widget.manager.session / 'mode_gui.png')], check=True, timeout=10)
            panel.root.attributes('-topmost', False)
            panel.root.iconify()
            stage, entered = 'rviz', now
        elif stage == 'rviz' and now - entered > 2:
            subprocess.run(['gnome-screenshot', '-f', str(widget.manager.session / 'mode_rviz.png')], check=True, timeout=10)
            widget.stop_play_button.invoke()
            stage, entered = 'close', now
        elif stage == 'close' and widget.manager.mode == 'idle' and panel.target == 'live':
            assert not widget.manager.children
            assert digest() == before
            report.update(live_configs_unchanged=True, paused_tuning=True, owned_processes_stopped=True)
            finish()
            return
        panel.root.after(250, step)
    except Exception as exc:
        finish(repr(exc))


panel.root.after(1500, step)
panel.root.mainloop()
raise SystemExit(exit_status)
