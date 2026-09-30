"""Explicit Jetson acceptance: sensor-only record, then automatic live restore."""
import json
from pathlib import Path
import subprocess
import time
from bag_manager import BagManager, metadata
from recording_policy import cache_losses
from rebuild_config import playback_topics

ROOT=Path(__file__).resolve().parent.parent
OUT=ROOT/'bag_tools/validation/20260929_sensor_only'


def heavy_processes():
    found=[]
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try:args=p.read_bytes().decode().split('\0')
        except (OSError,UnicodeError):continue
        if any(Path(arg).name in ('nvblox_node','mesh_height.py','realtime_node.py') for arg in args if arg):
            found.append(dict(pid=int(p.parent.name),args=args))
    return found


def main():
    manager=BagManager();result={}
    try:
        key=manager.record('纯传感器录制验收（Mesh 关闭）',new_map=True)
        result['bag']=key;path=manager.bags/key
        result['heavy_during_start']=heavy_processes()
        assert not result['heavy_during_start'],result
        for index in range(6):
            time.sleep(5);print(manager.tick()['message'],flush=True)
        result['heavy_during_end']=heavy_processes()
        assert not result['heavy_during_end'],result
        manager.finish_record()
        data=metadata(path/'data');result['metadata']=data
        result['cache_lost']=cache_losses(path/'record.log')
        assert result['cache_lost']==0
        for topic in ['/camera/d435i/depth/image_rect_raw','/camera/d435i/depth/camera_info',
                      '/LIDAR/POINTS_NX','/IMU','/tf','/tf_static','/nvblox_lio/odom',
                      '/nvblox_robot/joint_states','/nvblox_robot/robot_description']:
            assert data['topics'].get(topic,0)>0,topic
        assert not any('mesh' in t or t.startswith('/se2_navmesh/') for t in data['topics'])
        result['map']=manager._map_health(path);assert result['map']['points']>0
        for mode in ('depth','fusion','lidar'):playback_topics(data['topics'],True,mode)
        result['heavy_after_restore']=heavy_processes()
        assert any(Path(v['args'][0]).name=='nvblox_node' for v in result['heavy_after_restore'])
        assert not manager.recording_inputs.exists()
        assert not manager.mapping_suspended.exists()
        assert manager._live_rviz_active()
        result['result']='PASS';print(json.dumps(result,ensure_ascii=False),flush=True)
    except Exception as exc:
        result.update(result='FAIL',error=str(exc));raise
    finally:
        (OUT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        manager.close()


if __name__=='__main__':main()
