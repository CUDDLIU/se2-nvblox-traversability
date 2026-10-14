#!/bin/bash
set -euo pipefail
source_dir=$(cd "$(dirname "$0")" && pwd)
cmake -S "$source_dir" -B "$source_dir/../build/source" -DCMAKE_BUILD_TYPE=Release
cmake --build "$source_dir/../build/source" -j2
