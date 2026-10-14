#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
sdk="$PWD/vendor/native"
mkdir -p build
c++ -std=c++17 -O2 -pthread -I"$sdk/include" -I"$sdk/include/dridl" native/bridge.cpp -L"$sdk/lib" -Wl,-rpath-link,"$sdk/lib" -Wl,-rpath,"$sdk/lib" -l:libdrdds.so.1 -l:libfastrtps.so.2.14 -l:libfastcdr.so.2 -o build/native_bridge
