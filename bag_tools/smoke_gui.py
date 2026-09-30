"""Explicit hardware integration check; run only while a static live scene is available."""
import json
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, '/home/nvidia/scanplanner_test/se2_terrain_check')
from terrain_tuning_gui import Panel
from bag_manager import metadata

panel = Panel()
widget = panel.bag_widget
if widget is None:
    raise RuntimeError('Bag tab did not load')
panel.notebook.select(1)
widget.title.set('功能验收：真实深度与雷达')
widget.rebuild.set(False)
stage = 'start'
entered = time.monotonic()
started = entered
key = None
report = {}


def screenshot(name):
    subprocess.run(['gnome-screenshot', '-f', str(widget.manager.runtime / name)], timeout=10)


def step():
    global stage, entered, key
    try:
        now = time.monotonic()
        if now-started > 150:
            raise RuntimeError('GUI smoke timeout: ' + stage + ' ' + widget.status.get())
        if widget.busy:
            panel.root.after(250, step)
            return
        if stage == 'start' and panel.ready:
            widget.record_button.invoke()
            stage, entered = 'recording', now
        elif stage == 'recording' and widget.manager.mode == 'record' and now-entered > 18:
            key = widget.manager.active.name
            screenshot('smoke_recording.png')
            widget.stop_record_button.invoke()
            stage, entered = 'saved', now
        elif stage == 'saved' and widget.manager.mode == 'idle':
            info = metadata(widget.manager.bags / key / 'data')
            report['bag'] = key
            report['metadata'] = info
            for topic in ('/camera/d435i/depth/image_rect_raw', '/nvblox/height_mesh',
                          '/se2_navmesh/local_display', '/tf_static', '/LIDAR/POINTS_NX'):
                if info['topics'].get(topic, 0) == 0:
                    raise RuntimeError('missing recorded topic: ' + topic)
            widget.populate(widget.manager.entries())
            widget.tree.selection_set(key)
            widget.rate.set('0.5')
            screenshot('smoke_saved.png')
            widget.play_button.invoke()
            stage, entered = 'playing', now
        elif stage == 'playing' and widget.manager.mode == 'play' and now-entered > 8:
            widget.pause_button.invoke()
            stage, entered = 'paused', now
        elif stage == 'paused' and widget.manager.paused:
            report['pause'] = True
            panel.root.iconify()
            screenshot('smoke_replay.png')
            widget.pause_button.invoke()
            stage, entered = 'resumed', now
        elif stage == 'resumed' and not widget.manager.paused and now-entered > 3:
            report['resume'] = True
            widget.stop_play_button.invoke()
            stage, entered = 'stopped', now
        elif stage == 'stopped' and widget.manager.mode == 'idle':
            widget.manager.rename(key, '验收样例：D435i + Airy')
            widget.manager.move_to_trash(key)
            assert widget.manager.checked_path(key, True).is_dir()
            widget.manager.restore(key)
            report['management'] = True
            report['result'] = 'PASS'
            (widget.manager.runtime / 'smoke_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            print(json.dumps(report, ensure_ascii=False), flush=True)
            panel.close()
            return
        panel.root.after(250, step)
    except Exception as exc:
        report.update(result='FAIL', error=str(exc), ui_status=widget.status.get())
        (widget.manager.runtime / 'smoke_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False), flush=True)
        panel.close()


panel.root.after(2000, step)
panel.root.mainloop()
