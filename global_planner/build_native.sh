#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
g++ -O3 -std=c++17 -Wall -Wextra -shared -fPIC astar.cpp -o astar.so
