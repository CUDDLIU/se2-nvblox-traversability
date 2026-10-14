#!/usr/bin/env bash
# Build a private perception output variant; never edit/install over upstream.
set -eo pipefail
scan_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
stair_upstream=${1:-$(dirname -- "$scan_root")/Super-LIO-nx-scanplanner-port}
stair_workspace="$scan_root/official_first_integration/build/stair_lio"
if [[ ! -d "$stair_workspace/src/super_lio" ]]; then
  mkdir -p "$stair_workspace/src/super_lio"
  tar -C "$stair_upstream/src/super_lio" --exclude=map_pgo --exclude=__pycache__ -cf - . |
    tar -C "$stair_workspace/src/super_lio" -xf -
  patch --batch --forward -d "$stair_workspace/src/super_lio" -p1 < "$scan_root/scanplanner_integration/lio_dense_body.patch"
fi
source /opt/ros/humble/setup.bash
source "$stair_upstream/install/setup.bash"
cd "$stair_workspace"
MAKEFLAGS=-j2 colcon build --packages-select super_lio --executor sequential --allow-overriding super_lio
