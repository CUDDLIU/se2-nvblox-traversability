#!/usr/bin/env bash
# Recreate only the RViz window owned by this navigation session.
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
        if b'--no-rviz' in argv:raise SystemExit('This session was started with --no-rviz.')
        caught=next(line.split()[1] for line in (p/'status').read_text().splitlines() if line.startswith('SigCgt:'))
        if not int(caught,16)&(1<<(signal.SIGUSR1-1)):
            raise SystemExit('This older session cannot restart RViz alone; stop and start navigation once to load the updated launcher.')
        os.kill(int(p.name),signal.SIGUSR1);found.append(int(p.name))
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
if not found:raise SystemExit('No ScanPlanner session is running.')
print('Requested RViz window restart for session: '+str(found))
PY
