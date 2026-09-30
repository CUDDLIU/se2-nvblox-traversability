#!/usr/bin/env bash
# Large fragmented lidar datagrams otherwise exhaust the default 4 MiB queue.
# Keep this reversible runtime setting here, not in an external sysctl.d file.
set -eo pipefail
current=$(sysctl -n net.ipv4.ipfrag_high_thresh)
if (( current < 134217728 )); then
  if [[ "${1:-}" == normal ]]; then
    sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
  else
    sudo -n sysctl -w net.ipv4.ipfrag_high_thresh=134217728
  fi
fi
