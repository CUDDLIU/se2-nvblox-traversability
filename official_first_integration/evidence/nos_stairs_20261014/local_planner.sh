#!/bin/bash
node_type=${1:-drdds}

SCRIPT_PATH=$(dirname "$(readlink -f "$0")")
echo "脚本路径: "
echo $SCRIPT_PATH


# 接可通行域  确认话题 /impassable_area 是否有效存在
# sudo chrt 50 taskset -c 6,7 $SCRIPT_PATH/../bin/pcl_remove &     

# 接普通点云     确认话题 /LIDAR/POINTS 是否有效存在
sudo chrt 50 taskset -c 6,7 $SCRIPT_PATH/../bin/pcl_pass_grid &

sudo chrt 50 taskset -c 6,7 $SCRIPT_PATH/../bin/localPlanner



