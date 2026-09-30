"""Read-only prerequisite and provenance checks for the nvblox replay adapter."""
import hashlib
import os
from pathlib import Path


def prepare_lidar(root, installed_library='/opt/ros/humble/lib/libnvblox_ros_lib.so'):
    root = Path(root)
    variants = root / 'terrain_variants'
    files = {
        'converter': variants / 'fast_lidar/libfast_lidar.so',
        'deskew': variants / 'deskew/build/terrain_lidar_deskew',
        'transport': variants / 'fastdds_large_shm.xml',
        'abi_manifest': variants / 'fast_lidar/installed_library.sha256',
    }
    for path in files.values():
        if not path.is_file():
            raise RuntimeError('雷达回放组件尚未就绪：' + str(path))
    if not os.access(files['deskew'], os.X_OK):
        raise RuntimeError('雷达去畸变程序不可执行：' + str(files['deskew']))
    installed = Path(installed_library)
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    tokens = files['abi_manifest'].read_text().split()
    if not tokens or not installed.is_file() or digest(installed) != tokens[0]:
        raise RuntimeError('nvblox 安装库已变化，请重新构建项目内 fast_lidar 适配器')
    return dict(sensor_adapter='single_origin_deskew', restore_scan_time=False,
        multi_lidar=False, self_filter=True, installed_nvblox_sha256=tokens[0],
        files={name: dict(path=str(path.relative_to(root)), sha256=digest(path))
               for name, path in files.items()})
