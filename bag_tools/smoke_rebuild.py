"""Explicit replay integration check. Requires only an existing sensor bag."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from tkinter import messagebox

sys.path.insert(0, '/home/nvidia/scanplanner_test/se2_terrain_check')
from terrain_tuning_gui import Panel

panel = Panel()
widget = panel.bag_widget
if widget is None:
    raise RuntimeError('Bag tab did not load')
panel.notebook.select(1)
key = sys.argv[1]
stage = 'start'
started = entered = time.monotonic()
report = {'errors': []}
live_config = Path('/home/nvidia/scanplanner_test/se2_terrain_check/paper_config.yaml')
before = hashlib.sha256(live_config.read_bytes()).hexdigest()
messagebox.showerror = lambda title, msg, **kw: report['errors'].append(str(msg))
baseline = None


def step():
    global stage, entered, baseline
    try:
        now = time.monotonic()
        if now-started > 180 or report['errors']:
            raise RuntimeError('stage=' + stage + '; ' + widget.status.get() + '; ' + str(report['errors']))
        if widget.busy:
            panel.root.after(250, step)
            return
        if stage == 'start' and key in widget.rows:
            widget.tree.selection_set(key)
            widget.rebuild.set(True)
            widget.rate.set('0.5')
            widget.play_button.invoke()
            stage, entered = 'rebuild', now
        elif stage == 'rebuild' and widget.manager.mode == 'play' and panel.target == 'replay' and panel.ready:
            report['target_switched'] = True
            stage, entered = 'collect', now
        elif stage == 'collect' and now-entered > 12 and panel.last_data:
            widget.pause_button.invoke()
            stage, entered = 'paused', now
        elif stage == 'paused' and widget.manager.paused and now-entered > 3:
            report['before'] = panel.last_data
            if not panel.last_data.get('mesh_messages', panel.last_data.get('input_sequence', 0)):
                raise RuntimeError('No reconstructed Mesh received')
            baseline = panel.vars['max_slope_deg'].get()
            report['old_version'] = panel.last_data.get('config_version', 0)
            panel.vars['max_slope_deg'].set(0.)
            panel.changed('max_slope_deg')
            panel.notebook.select(0)
            stage, entered = 'tuned', now
        elif stage == 'tuned' and panel.last_data and panel.last_data.get('config_version', 0) > report['old_version']:
            if panel.config.get('max_slope_deg') != 0.:
                raise RuntimeError('Replay parameter was not changed')
            if now-entered > 4:
                report['after'] = panel.last_data
                panel.save()
                widget.submit(widget.manager.save_comparison, '验收：暂停后最大坡度改为 0°')
                stage, entered = 'snapshot', now
        elif stage == 'snapshot' and now-entered > 2:
            panel.root.lift()
            panel.root.update_idletasks()
            subprocess.run(['gnome-screenshot', '-f', str(widget.manager.runtime / 'rebuild_tuning.png')], timeout=10)
            widget.pause_button.invoke()
            stage, entered = 'finish', now
        elif stage == 'finish' and widget.manager.mode == 'finished':
            # End-of-bag must still allow reclassification on the held map.
            report['eof_version'] = panel.last_data.get('config_version', 0)
            panel.vars['max_slope_deg'].set(baseline)
            panel.changed('max_slope_deg')
            stage, entered = 'eof_tune', now
        elif stage == 'eof_tune' and panel.last_data.get('config_version', 0) > report['eof_version'] and now-entered > 4:
            report['after_eof'] = panel.last_data
            widget.submit(widget.manager.save_comparison, '验收：播完后恢复原始坡度')
            stage, entered = 'close', now
        elif stage == 'close' and now-entered > 2:
            report['session'] = str(widget.manager.session)
            snapshots=[json.loads(p.read_text()) for p in (widget.manager.session/'comparisons').glob('*.json')]
            assert len(snapshots)==2
            for data in snapshots:
                assert data.get('clock',0)>0,data
                assert 'recorded' in data,data.keys()
                assert data['source_publishers']['/nvblox_node/mesh']==['nvblox_node'],data['source_publishers']
            report['new_mapper_verified']=True
            panel.root.iconify()
            subprocess.run(['gnome-screenshot', '-f', str(widget.manager.runtime / 'rebuild_rviz.png')], timeout=10)
            widget.stop_play_button.invoke()
            stage, entered = 'restored', now
        elif stage == 'restored' and widget.manager.mode == 'idle' and panel.target == 'live':
            assert hashlib.sha256(live_config.read_bytes()).hexdigest() == before
            report['live_config_unchanged'] = True
            report['target_restored'] = True
            report['result'] = 'PASS'
            (widget.manager.runtime / 'rebuild_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            print(json.dumps({k: v for k, v in report.items() if k not in ('before', 'after', 'after_eof')}, ensure_ascii=False), flush=True)
            panel.close()
            return
        panel.root.after(250, step)
    except Exception as exc:
        report.update(result='FAIL', error=str(exc), ui_status=widget.status.get(),
                      session=str(widget.manager.session))
        (widget.manager.runtime / 'rebuild_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False), flush=True)
        panel.close()


panel.root.after(2000, step)
panel.root.mainloop()
