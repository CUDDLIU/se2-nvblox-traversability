#!/usr/bin/env python3
"""Install the captured official archive, verifying every published manifest hash."""
import argparse,hashlib,json,tarfile
from pathlib import Path
HERE=Path(__file__).resolve().parent

def install(archive):
    manifest=json.loads((HERE/'evidence/manifest.json').read_text())['files'];dest=HERE/'vendor';dest.mkdir(exist_ok=True)
    with tarfile.open(archive,'r:gz') as tar:
        for name,expected in manifest.items():
            member=tar.getmember(name)
            if not member.isfile() or Path(name).is_absolute() or '..' in Path(name).parts:raise ValueError('unsafe archive member')
            data=tar.extractfile(member).read()
            if len(data)!=expected['bytes'] or hashlib.sha256(data).hexdigest()!=expected['sha256']:raise ValueError('official asset mismatch: '+name)
            path=dest/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    print(f'Verified {len(manifest)} official files')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('archive',type=Path);a=p.parse_args();install(a.archive)
