"""Supplement old experiment summaries with final mapper integration counters."""
import argparse
import json
from pathlib import Path
import re
import yaml


def main():
    p=argparse.ArgumentParser();p.add_argument('bag',type=Path);args=p.parse_args()
    metadata=yaml.safe_load((args.bag/'data/metadata.yaml').read_text())['rosbag2_bagfile_information']
    expected={t['topic_metadata']['name']:t['message_count'] for t in metadata['topics_with_message_count']}
    for folder in sorted((args.bag/'experiments').iterdir()):
        if not all((folder/n).exists() for n in ('experiment.json','summary.json','mapper.log')):continue
        manifest=json.loads((folder/'experiment.json').read_text())
        if not manifest.get('arguments',{}).get('large_shm'):continue
        path=folder/'summary.json';summary=json.loads(path.read_text())
        if not summary.get('run_completed'):continue
        log=(folder/'mapper.log').read_text(errors='replace')
        ingestion=summary.setdefault('sensor_ingestion',{})
        for timer in ('ros/pointcloud_callback','ros/lidar','ros/lidar/integration',
                      'ros/lidar/front_integrated','ros/lidar/rear_integrated',
                      'ros/depth_image_callback','ros/depth','ros/depth/integrate'):
            counts=re.findall(r'^'+re.escape(timer)+r'\s+(\d+)\s+[\deE.+-]+\s+\(',log,re.M)
            if counts:ingestion[timer]=int(counts[-1])
        ingestion['raw_depth_messages_in_full_bag']=expected.get('/camera/d435i/depth/image_rect_raw',0)
        ingestion['raw_lidar_messages_in_full_bag']=expected.get('/LIDAR/POINTS_NX',0)
        ingestion['final_counter_audit']='Read final cumulative timing rows from this experiment mapper.log'
        path.write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        print(folder.name,ingestion.get('ros/lidar/integration'),ingestion.get('ros/depth/integrate'))


if __name__=='__main__':main()
