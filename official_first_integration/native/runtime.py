"""Run original NOS executables in private network, IPC and filesystem mounts."""
import os, signal, socket, struct, subprocess, sys, time
from pathlib import Path
HERE=Path(__file__).resolve().parent
BASE=HERE.parent

def verify_assets():
    import hashlib,json,shutil
    if not shutil.which('bwrap'):raise RuntimeError('bubblewrap is required for NOS network isolation')
    if not (BASE/'build/native_bridge').is_file():raise RuntimeError('Run native/build.sh first')
    manifest=json.loads((BASE/'evidence/manifest.json').read_text())['files']
    entries={k:v['sha256'] for k,v in manifest.items() if k.startswith(('paths/first/','config/'))}
    entries.update({'native/bin/localPlanner':'811d309e1a45d2f2a91d3cf423b809e1b4c1959f53c73d71f8e2dcee490dd354',
                    'native/bin/pcl_pass_grid':'a51736b0b86bb6a202a915f6bb7c22b9ddf50162efc7bab9823f163ddfee68a9'})
    for name,expected in entries.items():
        with (BASE/'vendor'/name).open('rb') as f:
            digest=hashlib.sha256()
            for chunk in iter(lambda:f.read(1024*1024),b''):digest.update(chunk)
        if digest.hexdigest()!=expected:raise RuntimeError('official asset identity mismatch: '+name)
    return entries

class Runtime:
    def __init__(self,output):
        identities=verify_assets()
        self.output=Path(output).resolve();self.output.mkdir(parents=True,exist_ok=True)
        (self.output/'config').mkdir(exist_ok=True)
        import json
        (self.output/'assets.json').write_text(json.dumps(identities,indent=2))
        import shutil
        for name in ('localplanner.yaml','pcl_pass_grid.yaml'):shutil.copy2(BASE/'vendor/config'/name,self.output/'config'/name)
        config=self.output/'config/localplanner.yaml'
        config.write_text(config.read_text().replace('local_try: true','local_try: false'))
        (self.output/'conf').mkdir(exist_ok=True);(self.output/'log').mkdir(exist_ok=True)
        (self.output/'conf/log.properties').write_text('log4cplus.rootLogger=INFO, console\nlog4cplus.appender.console=log4cplus::ConsoleAppender\nlog4cplus.appender.console.layout=log4cplus::PatternLayout\nlog4cplus.appender.console.layout.ConversionPattern=%d %p %m%n\n')
        # Short AF_UNIX pathname, private directory, no TCP port or hardware DDS.
        import tempfile
        self.temp=Path(tempfile.mkdtemp(prefix='nos-native-'))
        self.listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        self.address=str(self.temp/'bridge.sock');self.listener.bind(self.address);self.listener.listen(1)
        self.log=(self.output/'isolation.log').open('w')
        command=['bwrap','--unshare-user','--uid','0','--gid','0','--cap-add','CAP_NET_ADMIN','--unshare-net','--unshare-ipc','--die-with-parent',
          '--ro-bind','/','/','--dev','/dev','--proc','/proc','--tmpfs','/dev/shm',
          '--tmpfs','/opt','--tmpfs','/var/opt',
          '--setenv','NOS_HOST_NETNS',os.readlink('/proc/self/ns/net'),
          '--ro-bind',str(BASE/'vendor'),'/opt/robot/share/planner',
          '--bind',str(self.output),'/var/opt/robot',
          '--ro-bind',str(self.output/'config'),'/opt/robot/share/planner/config',
          '--setenv','LD_LIBRARY_PATH',str(BASE/'vendor/native/lib'),
          '--unsetenv','FASTRTPS_DEFAULT_PROFILES_FILE','--unsetenv','FASTDDS_DEFAULT_PROFILES_FILE',
          '--','/usr/bin/python3',str(HERE/'runtime.py'),'--inner',self.address,'/var/opt/robot']
        self.process=subprocess.Popen(command,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        self.listener.settimeout(15)
        try:self.socket,_=self.listener.accept()
        except BaseException:self.close();raise
        self.socket.settimeout(2);self.receive_buffer=bytearray()
    def send(self,kind,data=b''):
        self.socket.sendall(struct.pack('<II',len(data),kind)+data)
    def receive(self):
        # Preserve partial frames across socket timeouts.
        while True:
            if len(self.receive_buffer)>=8:
                size,kind=struct.unpack_from('<II',self.receive_buffer)
                if size>16000000:raise ValueError('native packet size')
                if len(self.receive_buffer)>=size+8:
                    data=bytes(self.receive_buffer[8:size+8]);del self.receive_buffer[:size+8]
                    return kind,data
            chunk=self.socket.recv(1024*1024)
            if not chunk:raise EOFError('native bridge closed')
            self.receive_buffer.extend(chunk)
    def close(self):
        if hasattr(self,'socket'):self.socket.close()
        if hasattr(self,'process') and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGTERM)
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait()
        self.listener.close();self.log.close()
        (self.temp/'bridge.sock').unlink(missing_ok=True);self.temp.rmdir()

def inner(address,output):
    # The only interfaces are namespace loopback and an isolated dummy eth0.
    import json
    for command in (['ip','link','add','eth0','type','dummy'],['ip','addr','add','10.254.86.1/24','dev','eth0'],['ip','link','set','eth0','up']):
        subprocess.run(command,check=True)
    (Path(output)/'namespace.json').write_text(json.dumps(dict(net=os.readlink('/proc/self/ns/net'),ipc=os.readlink('/proc/self/ns/ipc'),interfaces=json.loads(subprocess.check_output(['ip','-j','addr'])))))
    children=[];logs=[];running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        for name,cmd in [('filter',[BASE/'vendor/native/bin/pcl_pass_grid']),('planner',[BASE/'vendor/native/bin/localPlanner']),('bridge',[BASE/'build/native_bridge',address])]:
            f=(Path(output)/(name+'.log')).open('w');logs.append(f)
            children.append(subprocess.Popen(list(map(str,cmd)),stdout=f,stderr=subprocess.STDOUT))
        while running:
            for p in children:
                if p.poll() is not None:raise RuntimeError(f'native child {p.pid} exited {p.returncode}')
            time.sleep(.1)
    finally:
        for p in children:
            if p.poll() is None:p.terminate()
        for p in children:
            try:p.wait(timeout=1)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in logs:f.close()
if __name__=='__main__':
    if len(sys.argv)==4 and sys.argv[1]=='--inner':inner(sys.argv[2],sys.argv[3])
    else:raise SystemExit('Use Runtime from the ROS adapter or isolated tests.')
