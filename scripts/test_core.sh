#!/usr/bin/env bash
# CPU-only checks; does not start ROS, sensors, or robot services.
set -euo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
python_bin=${PYTHON:-python3}
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-3}
export OMP_WAIT_POLICY=PASSIVE
export PYTHONPATH="$project_root/global_planner:$project_root/terrain_variants/local_window:$project_root/bag_tools:$project_root/robot_model${PYTHONPATH:+:$PYTHONPATH}"
bash "$project_root/terrain_variants/local_window/build.sh"
bash "$project_root/global_planner/build_native.sh"
cd -- "$project_root"
"$python_bin" -m unittest -v test_physical_clearance test_pose_graph test_angular_display test_window test_folded_floors
"$python_bin" -m unittest discover -s bag_tools -p 'test_*.py' -v
"$python_bin" -m unittest -v test_kinematics test_planner
