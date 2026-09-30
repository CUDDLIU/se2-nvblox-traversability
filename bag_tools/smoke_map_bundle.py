"""Live Jetson acceptance. Creates then deletes only its own test bag + map."""
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
manager = widget.manager
report = {'errors': []}
messagebox.showerror = lambda title, msg, **kw: report['errors'].append(str(msg))
stage = 'start'
started = entered = time.monotonic()
key = None
bag = None
map_hash = None
initial_session = subprocess.check_output(['systemctl', '--user', 'show', 'nvblox-lio',
    '-p', 'InvocationID', '--value'], text=True).strip()
original_bags = {p.name for p in manager.bags.iterdir() if p.is_dir() and not p.name.startswith('.')}


def step():
    global stage, entered, key, bag, map_hash
    try:
        now = time.monotonic()
        if now-started > 300 or report['errors']:
            raise RuntimeError(f'{stage}: {widget.status.get()}; {report["errors"]}')
        if widget.busy:
            panel.root.after(250, step)
            return
        if stage == 'start' and panel.ready:
            widget.title.set('自动验收：新地图与 bag 绑定（测试后删除）')
            widget.lidar.set(True)
            widget.new_map.set(True)
            widget.record_button.invoke()
            stage = 'recording'
            print('Starting fresh online mapping + recording', flush=True)
        elif stage == 'recording' and manager.mode == 'record':
            key = manager.active.name
            bag = manager.active
            doc = json.loads((bag/'session.json').read_text())
            assert doc['nvblox_lio_session'] != initial_session
            assert doc['fresh_map'] and not doc['loaded_old_map']
            report['new_mapping_session'] = doc['nvblox_lio_session']
            stage, entered = 'collect', now
            print('Collecting 75 seconds of live data: ' + key, flush=True)
        elif stage == 'collect' and now-entered >= 75:
            report['health_before_stop'] = manager._map_health(bag)
            assert not report['health_before_stop']['warning'], report['health_before_stop']
            widget.stop_record_button.invoke()
            stage = 'saved'
        elif stage == 'saved' and manager.mode == 'idle':
            health = manager._map_health(bag)
            assert health['final'] and health['points'] > 100 and not health['errors'], health
            assert not health['had_input_gaps'], health
            assert health['counts']['map'] > 500, health['counts']
            map_hash = hashlib.sha256((bag/'maps/superlio_map.pcd').read_bytes()).hexdigest()
            manifest = json.loads((bag/'session.json').read_text())
            assert manifest['topics']['/IMU'] > 0 and manifest['topics']['/nvblox_lio/cloud_world'] > 500
            assert '/LIDAR/POINTS' not in manifest['topics_requested']
            report.update(saved_map_points=health['points'], recorded_topics=manifest['topics'])
            widget.refresh()
            stage = 'select'
            print('Saved bound map and trajectory', flush=True)
        elif stage == 'select' and key in widget.rows:
            widget.tree.selection_set(key)
            widget.rebuild.set(True)
            widget.rate.set('1')
            widget.play_button.invoke()
            stage = 'playing'
        elif stage == 'playing' and panel.target == 'replay' and panel.ready and panel.last_data and panel.last_data.get('mesh_messages', 0) > 2:
            assert manager.session.parent == bag/'replays'
            assert not manager._live_rviz_active()
            widget.pause_button.invoke()
            stage = 'paused'
        elif stage == 'paused' and manager.paused:
            saved = json.loads(manager._helper('saved-map'))
            assert saved['points'] == report['saved_map_points'], saved
            report['saved_map_published_in_rviz_domain'] = saved
            widget.submit(manager.save_comparison, '验收：绑定地图与重建结果')
            stage = 'snapshot'
        elif stage == 'snapshot':
            assert list(manager.session.glob('comparisons/*.json'))
            widget.restore_live_button.invoke()
            stage = 'restored_live'
        elif stage == 'restored_live' and manager.mode == 'idle' and panel.target == 'live' and panel.ready:
            assert manager._live_rviz_active()
            widget.submit(manager.move_to_trash, key)
            stage = 'trashed'
        elif stage == 'trashed':
            assert not bag.exists()
            assert (manager.trash/key/'maps/superlio_map.pcd').exists()
            assert list((manager.trash/key/'replays').glob('*/comparisons/*.json'))
            widget.submit(manager.restore, key)
            stage = 'restored_bag'
        elif stage == 'restored_bag':
            assert hashlib.sha256((bag/'maps/superlio_map.pcd').read_bytes()).hexdigest() == map_hash
            widget.submit(manager.move_to_trash, key)
            stage = 'purging'
        elif stage == 'purging':
            widget.submit(manager.purge, key)
            stage = 'done'
        elif stage == 'done':
            assert not bag.exists() and not (manager.trash/key).exists()
            assert all((manager.bags/k).exists() for k in original_bags)
            report.update(result='PASS', binding_trash_restore_purge=True, original_bags_preserved=True)
            finish()
            return
        panel.root.after(250, step)
    except Exception as exc:
        report.update(result='FAIL', error=repr(exc), stage=stage, bag=key)
        finish()


def finish():
    (manager.runtime/'map_bundle_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False), flush=True)
    panel.close()


panel.root.after(1500, step)
panel.root.mainloop()
sys.exit(0 if report.get('result') == 'PASS' else 1)
