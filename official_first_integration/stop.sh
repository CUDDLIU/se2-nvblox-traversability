#!/usr/bin/env bash
set -eo pipefail
scan_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 - "$scan_directory/run.py" <<'PY'
import os,signal,sys
from pathlib import Path
script=os.fsencode(sys.argv[1]);found=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        if p.stat().st_uid!=os.getuid():continue
        argv=(p/'cmdline').read_bytes().split(b'\0')
        if len(argv)<2 or argv[1]!=script or int(p.name)==os.getpid():continue
        os.kill(int(p.name),signal.SIGTERM);found.append(int(p.name))
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
print('Stopping FIRST session: '+str(found) if found else 'No FIRST session is running.')
PY
