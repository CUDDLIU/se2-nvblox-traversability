"""Build a compact comparison from recorded evidence, without promoting PASS to acceptance."""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser();parser.add_argument('experiments',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    rows=[]
    for folder in sorted(args.experiments.iterdir()):
        if not (folder/'summary.json').exists() or not (folder/'experiment.json').exists():continue
        manifest=json.loads((folder/'experiment.json').read_text());s=json.loads((folder/'summary.json').read_text())
        if not manifest.get('arguments',{}).get('large_shm'):continue
        if not s.get('run_completed') or manifest['arguments'].get('duration',0):continue
        route=folder/'route_analysis_common_offset.json'
        r=json.loads(route.read_text()) if route.exists() else {}
        actual=s.get('sensor_ingestion',{}).get('ros/pointcloud_callback')
        expected=s.get('sensor_ingestion',{}).get('raw_lidar_messages_in_full_bag')
        latency=s.get('latency_oldest_ms',{})
        rows.append(dict(experiment=folder.name,mode=manifest['arguments']['mode'],
            voxel_size_m=manifest['arguments'].get('voxel'),
            truncation_m=manifest['arguments'].get('voxel',0)*manifest['arguments'].get('truncation',0),
            mesh_min_weight=manifest['arguments'].get('min_mesh_weight'),
            weighting=manifest['arguments'].get('weighting') or 'inverse_square',
            effective_tsdf_weighting=s.get('effective_tsdf_weighting',{}),
            lidar_weighting=manifest['arguments'].get('lidar_weighting'),
            truncation_vox=manifest['arguments'].get('truncation'),
            capture_mesh=manifest['arguments'].get('capture_mesh',False),
            multi_lidar=manifest['arguments'].get('multi_lidar',False),
            lidar_native_axes=manifest['arguments'].get('lidar_native_axes',False),
            lidar_optical_origin=manifest['arguments'].get('lidar_optical_origin',False),
            restore_scan_time=manifest['arguments'].get('restore_scan_time',False),
            depth_rate_ceiling=manifest['arguments'].get('depth_rate',15.),
            depth_integrated=s.get('sensor_ingestion',{}).get('ros/depth/integrate'),
            depth_received=s.get('sensor_ingestion',{}).get('ros/depth_image_callback'),
            self_filter=manifest['arguments'].get('self_filter',False),
            complete_lidar_ingestion=bool(expected and actual==expected),
            latency_p50_ms=latency.get('p50'),latency_p95_ms=latency.get('p95'),
            distinct_input_result_hz=s.get('distinct_result_sequence_hz'),
            complete_input_result_hz=s.get('complete_input_result_hz'),
            lidar_received=actual,lidar_in_bag=expected,
            route_coverage=r.get('route_coverage'),upper_floor_coverage=r.get('upper_floor_coverage'),
            route_components=r.get('route_components'),
            run_completed=s.get('run_completed'),performance_target_met=s.get('performance_target_met'),
            connectivity_verified=s.get('connectivity_verified',False)))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'comparison.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    def number(v,percent=False):return '待测' if v is None else (f'{v:.1%}' if percent else f'{v:.1f}')
    lines=['# 传输修正后的回放对照','',
        '同一楼梯 bag，1 倍速；所有程序和数据位于原项目内。这里只纳入显式扩大共享内存后的实验，旧实验的雷达接收不完整，见各 transport_audit。','',
        '|实验 / 输入|体素 cm / Mesh 权重 / 截断 vox|附加条件|接收雷达帧 / 包内帧|不同 Mesh 输入结果 Hz|延迟 p50 / p95 ms|历史路线 / 上层覆盖|路线分量|',
        '|---|---:|---|---:|---:|---:|---:|---:|']
    for r in rows:
        flags='、'.join(label for key,label in [('self_filter','本体盒'),('multi_lidar','双原点'),
            ('lidar_native_axes','物理轴900×96'),('lidar_optical_origin','轴向光心'),('restore_scan_time','恢复扫描时间'),('capture_mesh','保存快照')] if r[key]) or '无'
        if r['weighting']!='inverse_square':
            flags+='、相机权重 '+r['weighting']
            if r['mode']=='lidar':flags+='（未作用于启用输入）'
        if r['lidar_weighting']:flags+='、雷达权重 '+r['lidar_weighting']
        voxel_cm=number(r['voxel_size_m']*100 if r['voxel_size_m'] is not None else None)
        lines.append(f"|{r['experiment']} / {r['mode']}|{voxel_cm} / {r['mesh_min_weight']} / {r['truncation_vox']}|{flags}|{r['lidar_received']} / {r['lidar_in_bag']}|{number(r['distinct_input_result_hz'])}|{number(r['latency_p50_ms'])} / {number(r['latency_p95_ms'])}|{number(r['route_coverage'],True)} / {number(r['upper_floor_coverage'],True)}|{r['route_components']}|")
    lines+=['','延迟范围为机器人附近局部窗口，从 Mesh 接收至结果准备发布；包含排队、解码、判定，不包含上游传感器至 Mesh，也未完全计入最终 ROS 序列化/传输。窗口外明确为历史结果。',
        'Hz 统计不同 Mesh 增量序号产生的结果，不能等同于原始雷达扫描频率；原始 LiDAR 约 10 Hz。',
        '此安装版本的 projective_integrator_weighting_mode 仅作用于相机 TSDF / color，不改变 LiDAR TSDF；193124 纯雷达试验的权重参数未生效，不能作为新雷达权重效果证据。有效积分器类型另存 comparison.json。',
        '快照录制会增加测量开销，不能与无快照的延迟直接等同。接收到全部 bag 消息也不保证每帧包含前后两颗雷达；独立 ring 审计见 multi_lidar/ring_availability.json。',
        '覆盖与连通性来自历史 SE(2) 单元，统一身体至地面偏移 0.565 m，位置/高度容差 0.30/0.20 m；不是机器人步态或实走验证。',
        '程序运行完成不代表达标。用户要求延迟尽力接近 50 ms；这里同时报告 p50/p95，并单独评价有效频率和楼梯连通性，不把内部较严格的 p95 检查冒充用户的硬性要求。','']
    lines+=['|融合实验|深度积分限频 Hz|深度接收 / 实际积分帧|','|---|---:|---:|']
    for r in rows:
        if r['mode']=='fusion':lines.append(f"|{r['experiment']}|{r['depth_rate_ceiling']}|{r['depth_received']} / {r['depth_integrated']}|")
    lines+=['','深度限频 15 Hz 时，时间戳抖动可能跳过部分已接收的 15 Hz 相机帧；提高限频的实验单独列明，不冒充完全相同参数。','']
    (args.output/'comparison.md').write_text('\n'.join(lines));print('\n'.join(lines))

if __name__=='__main__':main()
