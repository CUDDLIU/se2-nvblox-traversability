import subprocess,tarfile,sys,re
from pathlib import Path
base=Path('/opt/robot/share/planner')
files={str(base/'bin/localPlanner'),str(base/'bin/pcl_pass_grid')}
for exe in tuple(files):
    out=subprocess.check_output(['ldd',exe],text=True)
    for p in re.findall(r'=> (/\S+)',out):
        if Path(p).name.startswith(('libc.so','libm.so','libpthread','libdl.so','librt.so','libstdc++','libgcc_s','libgomp')):continue
        files.add(p)
with tarfile.open(fileobj=sys.stdout.buffer,mode='w|gz',dereference=True) as t:
    for p in sorted(files): t.add(p,arcname=('bin/' if '/planner/bin/' in p else 'lib/')+Path(p).name)
    for name in ('drdds','dridl','fastdds','fastrtps','fastcdr','foonathan_memory'):
        t.add('/usr/local/include/'+name,arcname='include/'+name)
