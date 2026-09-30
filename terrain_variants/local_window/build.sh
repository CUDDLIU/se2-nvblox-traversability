#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
# Match the reference arithmetic on ARM as well as x86; no fused boundary
# rounding differences at triangle/grid-plane intersections.
temporary="raster.so.tmp.$$"
trap 'rm -f -- "$temporary"' EXIT
g++ -O3 -fopenmp -ffp-contract=off -std=c++17 -Wall -Wextra -shared -fPIC raster.cpp native_bvh.cpp native_nav.cpp stair_filter.cpp -o "$temporary"
mv -f -- "$temporary" raster.so
