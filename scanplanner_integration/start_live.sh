#!/usr/bin/env bash
set -eo pipefail
scan_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$scan_directory/start.sh" --live --localize-at-start "$@"
