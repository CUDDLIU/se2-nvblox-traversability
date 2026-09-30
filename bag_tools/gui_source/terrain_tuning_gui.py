#!/usr/bin/env python3
"""Live, atomic geometry tuning panel. No robot commands are produced."""
import json
import os
from pathlib import Path
import signal
import sys
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.parameter import Parameter
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
import yaml

FIELDS = (
    ('max_slope_deg', '最大坡度', '°', 0., 45., .5, 1.),
    ('max_step', '相邻最大高差', 'cm', 0., 20., 1., 100.),
    ('height', '机器人高度', 'cm', 50., 130., 1., 100.),
    ('top_margin', '顶部余量', 'cm', 0., 40., 1., 100.),
    ('side_margin', '侧向余量', 'cm', 0., 30., .5, 100.),
    ('length', '机器人长度', 'cm', 40., 140., 1., 100.),
    ('width', '机器人宽度', 'cm', 30., 100., .2, 100.),
)
DEFAULTS=dict(max_slope_deg=15.,max_step=.03,height=.9,top_margin=.1,
              side_margin=.08,length=.82,width=.506)


class ParameterClient:
    """Humble-compatible client; newer rclpy's parameter_client is optional."""
    def __init__(self,node,target):
        self.get=node.create_client(GetParameters,target+'/get_parameters')
        self.set=node.create_client(SetParametersAtomically,target+'/set_parameters_atomically')

    def services_are_ready(self):
        return self.get.service_is_ready() and self.set.service_is_ready()

    def get_parameters(self,names):
        return self.get.call_async(GetParameters.Request(names=names))

    def set_parameters_atomically(self,parameters):
        return self.set.call_async(SetParametersAtomically.Request(
            parameters=[p.to_parameter_msg() for p in parameters]))


class Panel:
    def __init__(self):
        self.original_ros_env={k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY',
            'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE')}
        self.target='live'
        self.reference=None
        self.last_data=None
        self.replay_client=None
        self.create_ros()
        self.live_client=self.client
        self.root=tk.Tk()
        self.root.title('M20 可通行判据 · 实时调节')
        self.root.geometry('900x780')
        self.root.minsize(820,700)
        self.notebook=ttk.Notebook(self.root)
        self.notebook.pack(fill='both',expand=True)
        tuning=ttk.Frame(self.notebook)
        bags=ttk.Frame(self.notebook)
        self.notebook.add(tuning,text='判据调参')
        self.notebook.add(bags,text='Bag 录制 / 回放 / 管理')
        self.bag_widget=None
        self.closing=False
        self.ready=False
        self.future=None
        self.get_future=None
        self.dirty=False
        self.due=0.
        self.last_status=0.
        self.last_attempt=0.
        self.future_started=0.
        self.config=None
        self.vars={}
        self.labels={}
        self.scales=[]
        self.target_label=ttk.Label(tuning,text='当前调参对象：实时程序',foreground='#075985')
        self.target_label.pack(anchor='w',padx=16,pady=(10,0))
        self.applied=ttk.Label(tuning,text='连接节点，读取当前参数…')
        self.applied.pack(anchor='w',padx=16,pady=10)
        grid=ttk.Frame(tuning)
        grid.pack(fill='x',padx=16)
        grid.columnconfigure(1,weight=1)
        for row,(name,label,unit,lo,hi,step,multiplier) in enumerate(FIELDS):
            ttk.Label(grid,text=label).grid(row=row,column=0,sticky='w')
            var=tk.DoubleVar(value=DEFAULTS[name]*multiplier)
            scale=tk.Scale(grid,from_=lo,to=hi,resolution=step,showvalue=False,
                           variable=var,orient='horizontal',state='disabled',
                           command=lambda value,n=name:self.changed(n))
            scale.grid(row=row,column=1,sticky='ew',padx=8,pady=5)
            text=ttk.Label(grid,width=11)
            text.grid(row=row,column=2)
            self.vars[name],self.labels[name]=var,text
            self.scales.append(scale)
        self.refresh_labels()
        ttk.Label(tuning,text='拖动后自动应用；停止拖动 300 ms 后提交整组参数。\n'
                  '已确认包络：长 82 cm、宽 50.6 cm、高 90 cm。',
                  wraplength=510).pack(anchor='w',padx=16,pady=10)
        buttons=ttk.Frame(tuning)
        buttons.pack(fill='x',padx=16)
        ttk.Button(buttons,text='恢复初始判据',command=self.defaults).pack(side='left')
        self.save_button=ttk.Button(buttons,text='保存当前参数供下次启动',command=self.save)
        self.save_button.pack(side='right')
        self.summary=ttk.Label(tuning,text='等待实时状态…',wraplength=780,justify='left')
        self.summary.pack(anchor='w',padx=16,pady=16)
        ttk.Label(tuning,text='绿色：所有朝向通过  橙色：部分朝向通过\n'
                  '蓝色：历史或旧参数结果，不代表当前可通行。',
                  wraplength=510).pack(anchor='w',padx=16)
        try:
            bag_tools=Path(os.environ.get('SE2_BAG_TOOLS',
                '/home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/bag_tools'))
            sys.path.insert(0,str(bag_tools))
            from bag_widget import BagWidget
            self.bag_widget=BagWidget(bags,lambda: self.config or self.desired(),
                lambda: {name for name,_ in self.node.get_topic_names_and_types()
                         if self.node.count_publishers(name)>0})
        except Exception as exc:
            ttk.Label(bags,text='Bag 功能加载失败：'+str(exc),wraplength=780).pack(padx=16,pady=16)
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        for sig in (signal.SIGINT,signal.SIGTERM):
            signal.signal(sig,lambda *_: self.root.after(0,self.close))
        self.root.after(30,self.poll)

    def create_ros(self):
        for key,value in self.original_ros_env.items():
            if value is None:os.environ.pop(key,None)
            else:os.environ[key]=value
        domain=int(self.original_ros_env.get('ROS_DOMAIN_ID') or 0)
        self.context=Context()
        rclpy.init(context=self.context,domain_id=domain,signal_handler_options=SignalHandlerOptions.NO)
        self.node=rclpy.create_node('se2_terrain_tuning_panel',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.client=ParameterClient(self.node,'/se2_terrain_check')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String,'/se2_terrain/status',
            lambda msg: self.status(msg) if self.target=='live' else None,qos)

    def destroy_ros(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def switch_target(self,replay):
        self.ready=False
        self.dirty=False
        for name in ('future','get_future'):
            future=getattr(self,name)
            if future:future.cancel()
            setattr(self,name,None)
        if self.replay_client:
            self.replay_client.close()
            self.replay_client=None
        self.target='replay' if replay else 'live'
        self.config=self.reference=self.last_data=None
        self.counts={}
        self.last_status=self.last_attempt=0.
        for scale in self.scales:scale.configure(state='disabled')
        if replay:
            from replay_control import ReplayClient
            manager=self.bag_widget.manager
            self.replay_client=ReplayClient(manager.tools,manager.session,manager.env(True),self.status,self.reference_status)
            self.client=self.replay_client
        else:
            self.client=self.live_client
        self.target_label.configure(text='当前调参对象：'+('Bag 重建回放（实时参数不变）' if replay else '实时程序'))
        self.save_button.configure(text='保存本次回放参数' if replay else '保存当前参数供下次启动')
        self.applied.configure(text='正在读取'+('回放' if replay else '实时')+'节点参数…')
        self.summary.configure(text='等待目标节点状态…')
        if replay:
            self.notebook.select(0)
            self.root.lift()

    def reference_status(self,msg):
        self.reference=json.loads(msg.data)

    def refresh_labels(self):
        for name,label,unit,lo,hi,step,m in FIELDS:
            self.labels[name].configure(text=f'{self.vars[name].get():.1f} {unit}')

    def changed(self,name):
        self.refresh_labels()
        if self.ready:
            self.dirty=True
            self.due=time.monotonic()+.3

    def defaults(self):
        if not self.ready:return
        defaults = dict(DEFAULTS)
        if self.target == 'replay':
            from replay_profiles import STAIRS_DEFAULTS
            defaults.update(STAIRS_DEFAULTS)
        for name,_,_,_,_,_,m in FIELDS:self.vars[name].set(defaults[name]*m)
        self.changed('')

    def desired(self):
        return {name:self.vars[name].get()/m for name,_,_,_,_,_,m in FIELDS}

    def status(self,msg):
        data=json.loads(msg.data)
        self.last_data=data
        self.last_status=time.monotonic()
        self.config=data.get('config',self.config)
        counts=data.get('display_counts',{})
        if counts:self.counts=counts
        counts=getattr(self,'counts',{})
        reasons=data.get('reasons',{})
        latency=data.get('latency_ms')
        text=f"待处理区域：{data.get('pending_slabs','—')}    输入等待块：{data.get('inbox_blocks','—')}\n"
        text+=f"最近输入→结果：{latency:.0f} ms\n" if latency is not None else '等待第一份局部结果…\n'
        text+=f"绿 {counts.get('green',0)}    橙 {counts.get('amber',0)}    蓝色历史 {counts.get('archived',0)}\n"
        if self.target=='replay':
            original=(self.reference or {}).get('display_counts',{})
            text+='录制时对照：'+(f"绿 {original.get('green',0)} / 橙 {original.get('amber',0)}" if original else '等待原始状态')+'\n'
        labels={'slope':'坡度','headroom':'净空','unknown_boundary':'未知边界',
                'ledge_or_nonwalkable_neighbor':'高差/相邻不可通行','ambiguous_layer':'层连接不明确',
                'footprint_or_boundary_distance':'轮廓或边界距离'}
        text+='已计算区域拒绝：'+ '；'.join(f'{labels.get(k,k)} {v}' for k,v in reasons.items()
                                             if k not in ('safe','restricted','walkable'))
        error=data.get('compute_error') or data.get('input_error')
        if error:text+='\n错误：'+error
        self.summary.configure(text=text)
        if self.config and self.ready and not self.dirty and not self.future:
            if all(abs(self.config[n]-v)<1e-6 for n,v in self.desired().items()):
                self.applied.configure(text=f"参数版本 {data.get('config_version',0)} 已生效；剩余重算 {data.get('pending_slabs',0)} 区域")

    def save(self):
        if not self.config or self.dirty or self.future or any(
                abs(self.config[n]-v)>1e-6 for n,v in self.desired().items()):
            self.applied.configure(text='请等待参数生效后再保存')
            return
        try:
            replay=self.target=='replay'
            path=(self.bag_widget.manager.session/'terrain_config.yaml') if replay else Path(__file__).with_name('paper_config.yaml')
            original=path.read_text()
            document=yaml.safe_load(original)
            document['se2_terrain_check']['ros__parameters'].update(self.desired())
            backup=path.with_name('paper_config.before_tuning.yaml')
            if not backup.exists():backup.write_text(original)
            temporary=path.with_suffix('.tmp')
            temporary.write_text(yaml.safe_dump(document,sort_keys=False,allow_unicode=True))
            os.replace(temporary,path)
            self.applied.configure(text='已保存本次回放参数；实时配置未改变' if replay else '已保存到 paper_config.yaml，下次启动继续使用')
        except Exception as exc:self.applied.configure(text='保存失败：'+str(exc))

    def poll(self):
        if self.closing:return
        if self.bag_widget:
            manager=self.bag_widget.manager
            replay=manager.rebuild and manager.mode in ('play','finished')
            if replay != (self.target=='replay'):
                self.switch_target(replay)
        self.executor.spin_once(timeout_sec=0)
        if self.replay_client:self.replay_client.poll()
        now=time.monotonic()
        if self.get_future and now-self.last_attempt>5.:
            self.get_future.cancel()
            self.get_future=None
            self.applied.configure(text='读取超时，等待连接恢复…')
        if self.future and now-self.future_started>10.:
            self.future.cancel()
            self.future=None
            self.applied.configure(text='提交超时；请查看节点状态后再次调整')
        if not self.ready and not self.get_future and now-self.last_attempt>1.:
            self.last_attempt=now
            if self.client.services_are_ready():
                self.get_future=self.client.get_parameters([f[0] for f in FIELDS])
        if self.get_future and self.get_future.done():
            try:
                result=self.get_future.result()
                for (name,_,_,_,_,_,m),value in zip(FIELDS,result.values):
                    self.vars[name].set(value.double_value*m)
                self.refresh_labels()
                for scale in self.scales:scale.configure(state='normal')
                # Tk delivers programmatic Scale updates at idle. They must
                # not submit parameters merely because the panel was opened.
                self.root.after_idle(lambda: setattr(self,'ready',True))
            except Exception as exc:self.applied.configure(text='读取失败：'+str(exc))
            self.get_future=None
        if self.future and self.future.done():
            try:
                result=self.future.result().result
                self.applied.configure(text='已接收，正在重算…' if result.successful else '拒绝：'+result.reason)
            except Exception as exc:self.applied.configure(text='提交失败：'+str(exc))
            self.future=None
        if self.ready and self.dirty and not self.future and now>=self.due:
            values=self.desired()
            self.future=self.client.set_parameters_atomically([
                Parameter(n,Parameter.Type.DOUBLE,v) for n,v in values.items()])
            self.future_started=now
            self.dirty=False
            self.applied.configure(text='提交参数中…')
        if self.ready and now-self.last_status>5.:
            self.summary.configure(text='节点状态超时；等待恢复连接。')
        self.root.after(30,self.poll)

    def close(self):
        if self.closing:return
        self.closing=True
        if self.bag_widget:
            self.bag_widget.close(self.finish_close)
        else:
            self.finish_close()

    def finish_close(self):
        if self.replay_client:self.replay_client.close()
        self.destroy_ros()
        for key,value in self.original_ros_env.items():
            if value is None:os.environ.pop(key,None)
            else:os.environ[key]=value
        self.root.destroy()


if __name__=='__main__':
    Panel().root.mainloop()
