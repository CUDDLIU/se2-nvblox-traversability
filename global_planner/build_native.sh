#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
artifact=$(mktemp ./astar.so.XXXXXX)
trap 'rm -f "$artifact"' EXIT
g++ -O3 -std=c++17 -Wall -Wextra -shared -fPIC astar.cpp -o "$artifact"
mv -f "$artifact" astar.so
