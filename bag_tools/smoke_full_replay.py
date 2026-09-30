"""Replay an entire user bag without modifying it; check memory and live restore."""
import json
from pathlib import Path
import subprocess
import sys
import time
from tkinter import messagebox
import yaml

sys.path.insert(0, '/home/nvidia/scanplanner_test/se2_terrain_check')
from terrain_tuning_gui import Panel

panel = Panel()
widget = panel.bag_widget
manager = widget.manager
key = sys.argv[1]
report = {'errors': [], 'progress': []}
messagebox.showerror = lambda title, msg, **kw: report['errors'].append(str(msg))
info = yaml.safe_load((manager.bags/key/'data/metadata.yaml').read_text())['rosbag2_bagfile_information']
duration = info['duration']['nanoseconds']/1e9
bag_start = info['starting_time']['nanoseconds_since_epoch']/1e9
lio_pid = subprocess.check_output(['systemctl', '--user', 'show', 'nvblox-lio', '-p', 'MainPID', '--value'], text=True).strip()
stage = 'start'
started = time.monotonic()
last_report = started


def active(unit):
    return subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit]).returncode == 0


def step():
    global stage, last_report
    try:
        now = time.monotonic()
        if now-started > duration+180 or report['errors']:
            raise RuntimeError(f'{stage}: {widget.status.get()}; {report["errors"]}')
        if stage == 'playing' and now-last_report > 15:
            data = panel.last_data or {}
            memory = {k: int(v.strip().split()[0]) for k, v in
                      (line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())
                      if k in ('MemFree', 'MemAvailable')}
            sample = dict(elapsed_s=round(now-started, 1), mode=manager.mode,
                          mesh_messages=data.get('mesh_messages'), memory_kib=memory)
            report['progress'].append(sample)
            print(json.dumps(sample), flush=True)
            last_report = now
        if widget.busy:
            panel.root.after(300, step)
            return
        if stage == 'start' and key in widget.rows:
            widget.tree.selection_set(key)
            widget.rebuild.set(True)
            widget.rate.set('1')
            widget.play_button.invoke()
            stage = 'playing'
        elif stage == 'playing':
            if manager.mode == 'idle':
                raise RuntimeError(manager.message)
            if manager.mode in ('play', 'finished'):
                assert not active('nvblox-d435i-shadow')
                assert not active('se2-terrain-check')
                assert active('nvblox-lio')
            if manager.mode == 'finished' and panel.last_data:
                report['mesh_messages'] = panel.last_data.get('mesh_messages', 0)
                assert report['mesh_messages'] > 100
                report['version'] = panel.last_data['config_version']
                report['original_slope'] = panel.vars['max_slope_deg'].get()
                panel.vars['max_slope_deg'].set(report['original_slope']+1.)
                panel.changed('max_slope_deg')
                stage = 'tune'
        elif stage == 'tune' and panel.last_data['config_version'] > report['version']:
            widget.submit(manager.save_comparison, '完整播放验收：结束后调参')
            stage = 'saved'
        elif stage == 'saved':
            snapshot = max((manager.session/'comparisons').glob('*.json'), key=lambda p:p.stat().st_mtime)
            data = json.loads(snapshot.read_text())
            report['playback_seconds'] = data['clock']-bag_start
            assert report['playback_seconds'] >= duration-2., report['playback_seconds']
            assert data['source_publishers']['/nvblox_node/mesh'] == ['nvblox_node']
            assert all(manager.children[n].poll() is None for n in ('nvblox', 'rviz', 'terrain', 'height'))
            report['session'] = str(manager.session)
            widget.restore_live_button.invoke()
            stage = 'restore'
        elif stage == 'restore' and manager.mode == 'idle' and panel.target == 'live' and panel.ready:
            assert active('nvblox-d435i-shadow') and active('se2-terrain-check') and manager._live_rviz_active()
            assert subprocess.check_output(['systemctl', '--user', 'show', 'nvblox-lio', '-p', 'MainPID', '--value'], text=True).strip() == lio_pid
            assert not manager.mapping_suspended.exists()
            report.update(result='PASS', live_restored=True, lio_origin_preserved=True)
            finish()
            return
        panel.root.after(300, step)
    except Exception as exc:
        report.update(result='FAIL', error=repr(exc), stage=stage, session=str(manager.session))
        finish()


def finish():
    (manager.runtime/'full_replay_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False), flush=True)
    panel.close()


panel.root.after(1500, step)
panel.root.mainloop()
sys.exit(0 if report.get('result') == 'PASS' else 1)
