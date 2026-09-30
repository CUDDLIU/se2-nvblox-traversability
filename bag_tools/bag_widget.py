"""Bag tab embedded in the existing Tk tuning window."""
from concurrent.futures import ThreadPoolExecutor
from datetime import timedelta
import json
import subprocess
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog

from bag_manager import BagManager
from rebuild_config import MODES
from replay_profiles import DEFAULT_SENSOR_MODE


def human_size(size):
    return f'{size / 1024**3:.2f} GiB' if size >= 1024**3 else f'{size / 1024**2:.1f} MiB'


class BagWidget:
    def __init__(self, parent, settings, available_topics):
        self.frame = parent
        self.settings = settings
        self.available_topics = available_topics
        self.manager = BagManager()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='bag-manager')
        self.future = None
        self.monitor = None
        self.busy = False
        self.closing = False
        self.rows = {}
        self.status = tk.StringVar(value='就绪')
        self.input_status = tk.StringVar(value='等待输入诊断…')
        self.title = tk.StringVar()
        self.lidar = tk.BooleanVar(value=True)
        self.deleted = tk.BooleanVar(value=False)
        self.rate = tk.StringVar(value='1')
        self.rebuild = tk.BooleanVar(value=True)
        self.sensor_mode = tk.StringVar(value=MODES[DEFAULT_SENSOR_MODE])
        self.new_map = tk.BooleanVar(value=True)
        self.controls = []
        ttk.Label(parent, text='录制与回放', font=('', 13, 'bold')).pack(anchor='w', padx=12, pady=(12, 4))
        ttk.Label(parent, text='录制时关闭 Mesh 和通行计算，保存传感器、位姿、实测关节及 SuperLIO 地图。\n'
                  '可选择深度、融合或纯雷达重建；开始后判据调参页自动控制回放。',
                  wraplength=740).pack(anchor='w', padx=12)
        form = ttk.Frame(parent)
        form.pack(fill='x', padx=12, pady=8)
        ttk.Label(form, text='录制名称：').pack(side='left')
        ttk.Entry(form, textvariable=self.title, width=30).pack(side='left', padx=4, fill='x', expand=True)
        ttk.Checkbutton(form, text='附带雷达 / IMU', variable=self.lidar).pack(side='left')
        mapping = ttk.Frame(parent)
        mapping.pack(fill='x', padx=12, pady=(0, 5))
        ttk.Checkbutton(mapping, text='新建地图后录制（重置实时地图和位姿原点）', variable=self.new_map).pack(side='left')
        self.button(mapping, '打开绑定地图', self.open_map)
        actions = ttk.Frame(parent)
        actions.pack(fill='x', padx=12)
        self.record_button = self.button(actions, '开始录制', self.record)
        self.stop_record_button = self.button(actions, '停止并保存', lambda: self.submit(self.manager.finish_record))
        ttk.Label(actions, text='  回放速度：').pack(side='left')
        ttk.Combobox(actions, textvariable=self.rate, values=('0.25', '0.5', '1', '2'),
                     width=5, state='readonly').pack(side='left')
        ttk.Label(actions, text='倍（开始回放时生效）').pack(side='left')
        self.state_label = ttk.Label(parent, textvariable=self.status, wraplength=740)
        self.state_label.pack(anchor='w', padx=12, pady=8)
        ttk.Label(parent,textvariable=self.input_status,wraplength=840,foreground='#075985').pack(anchor='w',padx=12,pady=(0,5))
        list_bar = ttk.Frame(parent)
        list_bar.pack(fill='x', padx=12)
        self.button(list_bar, '刷新列表', self.refresh)
        ttk.Checkbutton(list_bar, text='查看回收站', variable=self.deleted, command=self.refresh).pack(side='left')
        self.button(list_bar, '诊断日志', self.open_diagnostics)
        columns = ('title', 'created', 'duration', 'size', 'state')
        table = ttk.Frame(parent)
        table.pack(fill='both', expand=True, padx=12, pady=5)
        self.tree = ttk.Treeview(table, columns=columns, show='headings', selectmode='browse', height=10)
        for name, label, width in zip(columns, ('名称', '录制时间', '时长', '大小', '状态'), (220, 165, 75, 95, 110)):
            self.tree.heading(name, text=label)
            self.tree.column(name, width=width, minwidth=60, stretch=name == 'title')
        self.tree.pack(side='left', fill='both', expand=True)
        scroll = ttk.Scrollbar(table, command=self.tree.yview)
        scroll.pack(side='right', fill='y')
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.bind('<<TreeviewSelect>>', self.show_details)
        inputs = ttk.Frame(parent)
        inputs.pack(fill='x', padx=12, pady=(3, 0))
        ttk.Label(inputs, text='重建输入：').pack(side='left')
        self.sensor_selector = ttk.Combobox(inputs, textvariable=self.sensor_mode,
            values=tuple(MODES.values()), width=22, state='readonly')
        self.sensor_selector.pack(side='left', padx=4)
        ttk.Label(inputs, text='楼梯默认：20 cm 台阶 / 40° 坡度；回放中可调。').pack(side='left')
        playback = ttk.Frame(parent)
        playback.pack(fill='x', padx=12, pady=5)
        ttk.Checkbutton(playback, text='重新建图并调参', variable=self.rebuild,
                        command=self.update_buttons).pack(side='left', padx=4)
        self.play_button = self.button(playback, '在 RViz 回放', self.play)
        self.pause_button = self.button(playback, '暂停 / 继续', lambda: self.submit(self.manager.toggle_pause))
        self.stop_play_button = self.button(playback, '关闭回放', lambda: self.submit(self.manager.stop_play))
        self.compare_button = self.button(playback, '保存对比快照', self.compare)
        planning = ttk.Frame(parent)
        planning.pack(fill='x', padx=12, pady=3)
        self.plan_button = self.button(planning, '全局路径规划', self.open_global_plan)
        self.plan_start_button = self.button(planning, '在 RViz 设起点',
            lambda: self.submit(self.manager.global_plan_action, 'pick_start'))
        self.plan_reset_button = self.button(planning, '恢复 bag 终点',
            lambda: self.submit(self.manager.global_plan_action, 'reset_start'))
        ttk.Label(planning, text='Publish Point 点选三维目标；关闭规划请点“关闭回放”。').pack(side='left')
        live = ttk.Frame(parent)
        live.pack(fill='x', padx=12, pady=5)
        self.restore_live_button = self.button(live, '恢复实时重建',
                                               lambda: self.submit(self.manager.restore_live))
        ttk.Label(live, text='重建回放暂停实时深度建图；恢复时重启深度建图，保留 LIO 位姿。').pack(side='left')
        manage = ttk.Frame(parent)
        manage.pack(fill='x', padx=12, pady=5)
        self.button(manage, '重命名', self.rename)
        self.button(manage, '打开目录', self.open_folder)
        self.button(manage, '移入回收站', self.trash)
        self.button(manage, '恢复', self.restore)
        self.button(manage, '永久删除', self.purge)
        self.button(manage, '修复中断的录制', self.repair)
        self.button(manage, '打开回放结果', self.open_results)
        self.details = ttk.Label(parent, text='选择 bag 查看消息数量和话题。', wraplength=740, justify='left')
        self.details.pack(fill='x', padx=12, pady=8)
        ttk.Label(parent, text=f'数据目录：{self.manager.bags}\n'
                  '不足 2 GiB 自动停止录制；移入回收站不释放空间，永久删除才释放。',
                  wraplength=740).pack(anchor='w', padx=12, pady=(0, 10))
        self.last_refresh = 0.
        self.refresh()
        parent.after(500, self.poll)

    def button(self, parent, text, command):
        button = ttk.Button(parent, text=text, command=command)
        button.pack(side='left', padx=(0, 5))
        self.controls.append(button)
        return button

    def submit(self, fn, *args):
        if self.busy or self.closing:
            return
        self.busy = True
        self.show_prepare_progress = getattr(fn, '__name__', '') == 'play'
        self.status.set('处理中…')
        self.update_buttons()
        self.future = self.worker.submit(fn, *args)

    def selected(self):
        selection = self.tree.selection()
        if not selection:
            self.status.set('请先选择一个 bag')
            return None
        return selection[0]

    def record(self):
        topics = self.available_topics()
        if not self.new_map.get() and not any(t in topics for t in ('/nvblox_node/mesh', '/camera/d435i/depth/image_rect_raw')):
            messagebox.showinfo('没有实时输入', '尚未发现深度或 Mesh 发布者。请先启动实时程序并等待数据。', parent=self.frame)
            return
        self.submit(self.manager.record, self.title.get(), self.lidar.get(), self.settings(), self.new_map.get())

    def play(self):
        key = self.selected()
        if key and not self.deleted.get():
            mode = next(key for key, label in MODES.items() if label == self.sensor_mode.get())
            self.submit(self.manager.play, key, float(self.rate.get()), self.rebuild.get(), None, mode)

    def compare(self):
        if self.manager.rebuild:
            title = simpledialog.askstring('保存对比', '为当前参数和结果填写备注：', parent=self.frame)
            if title is not None:
                self.submit(self.manager.save_comparison, title)

    def rename(self):
        key = self.selected()
        if key:
            title = simpledialog.askstring('重命名', 'bag 显示名称：', initialvalue=self.rows[key]['title'], parent=self.frame)
            if title:
                self.submit(self.manager.rename, key, title, self.deleted.get())

    def open_folder(self):
        key = self.selected()
        if key:
            path = self.manager.checked_path(key, self.deleted.get())
        else:
            path = self.manager.bags
        subprocess.Popen(['xdg-open', str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def open_results(self):
        path = self.manager.session or self.manager.runtime
        subprocess.Popen(['xdg-open', str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def open_global_plan(self):
        key = self.selected()
        if key and not self.deleted.get():
            self.submit(self.manager.open_global_plan, key)

    def open_map(self):
        key = self.selected()
        if key:
            path = self.manager.checked_path(key, self.deleted.get()) / 'maps'
            if not path.is_dir():
                self.status.set('这个旧 bag 没有绑定的 SuperLIO 地图；回放结果仍随 bag 管理')
                return
            subprocess.Popen(['xdg-open', str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def open_diagnostics(self):
        selection=self.tree.selection()
        path=self.manager.root/'diagnostics/live'
        if selection:
            candidate=self.manager.checked_path(selection[0],self.deleted.get())/'diagnostics'
            if candidate.is_dir():path=candidate
        if path.is_dir():
            subprocess.Popen(['xdg-open',str(path)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        else:self.status.set('诊断日志尚未生成；运行原启动命令会自动开启')

    def trash(self):
        key = self.selected()
        if key and not self.deleted.get() and messagebox.askyesno('移入回收站',
                f'将“{self.rows[key]["title"]}”及绑定地图、回放结果一起移入回收站？可恢复。', parent=self.frame):
            self.submit(self.manager.move_to_trash, key)

    def restore(self):
        key = self.selected()
        if key and self.deleted.get():
            self.submit(self.manager.restore, key)

    def purge(self):
        key = self.selected()
        if key and self.deleted.get() and messagebox.askyesno('永久删除',
                f'永久删除“{self.rows[key]["title"]}”及绑定地图、轨迹和回放结果？无法恢复。', parent=self.frame):
            self.submit(self.manager.purge, key)
        elif key and not self.deleted.get():
            self.status.set('请先移入回收站；永久删除仅对回收站中的 bag 开放')

    def repair(self):
        key = self.selected()
        if key and not self.deleted.get():
            self.submit(self.manager.repair, key)

    def refresh(self):
        if not self.busy and not self.closing:
            deleted = self.deleted.get()
            self.submit(lambda: ('rows', deleted, self.manager.entries(deleted)))

    def populate(self, rows):
        selected = self.tree.selection()
        self.tree.delete(*self.tree.get_children())
        self.rows = {r['key']: r for r in rows}
        states = {'ready': '已保存', 'recording': '录制中', 'failed': '失败', 'incomplete': '待检查'}
        for row in rows:
            self.tree.insert('', 'end', iid=row['key'], values=(row['title'], row['created'].replace('T', ' '),
                str(timedelta(seconds=int(row['duration']))), human_size(row['size']), states.get(row['state'], row['state'])))
        if selected and selected[0] in self.rows:
            self.tree.selection_set(selected[0])
        self.last_refresh = time.monotonic()

    def show_details(self, event=None):
        selection = self.tree.selection()
        if not selection:
            return
        row = self.rows[selection[0]]
        names = list(row['topics'])
        path = self.manager.checked_path(selection[0], self.deleted.get())
        health = self.manager._map_health(path)
        map_text = f" · 地图 {health['points']:,} 点" if 'points' in health else ' · 无绑定点云地图'
        self.details.configure(text=f"消息 {row['messages']:,} 条 · 话题 {len(names)} 个{map_text}\n" + ', '.join(names))

    def update_buttons(self):
        self.sensor_selector.configure(state='readonly' if not self.busy and not self.closing
            and self.manager.mode == 'idle' and self.rebuild.get() else 'disabled')
        for button in self.controls:
            button.configure(state='disabled' if self.busy or self.closing else 'normal')
        if self.busy or self.closing:
            return
        mode = self.manager.mode
        self.record_button.configure(state='normal' if mode == 'idle' else 'disabled')
        self.stop_record_button.configure(state='normal' if mode == 'record' else 'disabled')
        self.play_button.configure(state='normal' if mode == 'idle' and not self.deleted.get() else 'disabled')
        self.pause_button.configure(state='normal' if mode == 'play' else 'disabled')
        self.stop_play_button.configure(state='normal' if mode in ('play', 'finished', 'plan') else 'disabled')
        self.plan_button.configure(state='normal' if mode == 'idle' and not self.deleted.get() else 'disabled')
        self.plan_start_button.configure(state='normal' if mode == 'plan' else 'disabled')
        self.plan_reset_button.configure(state='normal' if mode == 'plan' else 'disabled')
        self.compare_button.configure(state='normal' if self.manager.rebuild and mode in ('play', 'finished') else 'disabled')
        self.restore_live_button.configure(state='disabled' if mode == 'record' else 'normal')

    def poll(self):
        if self.closing:
            return
        if self.busy and getattr(self, 'show_prepare_progress', False) and self.manager.session:
            # Read a tiny progress file without waiting for BagManager's lock,
            # which is held by the worker while it prepares the replay.
            try:
                progress=json.loads((self.manager.session/'prepare_progress.json').read_text())
                if time.time()-progress['updated']<30:
                    self.status.set(progress['message'])
            except (OSError,ValueError,KeyError,TypeError):
                pass
        if self.future and self.future.done():
            try:
                result = self.future.result()
                if isinstance(result, tuple) and result[0] == 'rows':
                    if result[1] == self.deleted.get():
                        self.populate(result[2])
                else:
                    self.status.set(self.manager.message)
                    self.last_refresh = 0
            except Exception as exc:
                self.status.set('操作失败：' + str(exc))
                messagebox.showerror('bag 操作失败', str(exc), parent=self.frame)
            self.future, self.busy = None, False
            self.update_buttons()
        if self.monitor and self.monitor.done():
            try:
                result = self.monitor.result()
                self.input_status.set(result.get('input_health',''))
                if not self.busy:
                    duration = f" · 已录 {int(result['elapsed'])} 秒" if result['mode'] == 'record' else ''
                    self.status.set(result['message'] + duration + f" · 磁盘可用 {human_size(result['free'])}")
                    self.update_buttons()
            except Exception as exc:
                self.status.set('状态检查失败：' + str(exc))
            self.monitor = None
        if not self.busy:
            if time.monotonic() - self.last_refresh > 5:
                self.refresh()
            elif self.monitor is None:
                # Disk checks and stop/flush can block, so even monitoring uses the worker.
                self.monitor = self.worker.submit(self.manager.tick)
        self.frame.after(500, self.poll)

    def close(self, done):
        if self.closing:
            return
        self.closing = True
        self.status.set('正在停止并保存录制，关闭回放…')
        self.update_buttons()
        future = self.worker.submit(self.manager.close)
        def wait():
            if future.done():
                self.worker.shutdown(wait=False)
                done()
            else:
                self.frame.after(100, wait)
        wait()
