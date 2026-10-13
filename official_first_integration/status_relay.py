"""Receive AOS telemetry through NOS's healthy internal network, without control.

The transient remote Python process writes no files and can send only heartbeat
100/100 to AOS. Its original kernel receive timestamp survives the SSH stream;
queued/replayed state must never refresh the robot's freshness watchdogs.
"""
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time

REMOTE_CODE = r'''
import json,select,socket,struct,time
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.connect(('10.21.33.103',30000));s.setblocking(False)
timestamp_option=getattr(socket,'SO_TIMESTAMPNS',35)
s.setsockopt(socket.SOL_SOCKET,timestamp_option,1)
header=struct.Struct('<4sHHB7x');magic=bytes.fromhex('eb91eb90');seq=0;last_hb=0.
print(json.dumps(dict(kind='ready',source=s.getsockname(),target=s.getpeername())),flush=True)
try:
 while True:
  now=time.monotonic()
  if now-last_hb>=.5:
   body=json.dumps({'PatrolDevice':{'Type':100,'Command':100,'Time':time.strftime('%Y-%m-%d %H:%M:%S'),'Items':{}}},separators=(',',':')).encode()
   s.send(header.pack(magic,len(body),seq,1)+body);seq=(seq+1)%65536;last_hb=now
  if not select.select([s],[],[],.05)[0]:continue
  data,anc,flags,peer=s.recvmsg(65536,socket.CMSG_SPACE(struct.calcsize('@ll')))
  received_ns=None
  for level,kind,value in anc:
   if level==socket.SOL_SOCKET and kind==timestamp_option:
    sec,nsec=struct.unpack('@ll',value[:struct.calcsize('@ll')]);received_ns=sec*1000000000+nsec
  if received_ns is None or len(data)<header.size or flags&socket.MSG_TRUNC:continue
  m,size,mid,fmt=header.unpack_from(data)
  if m!=magic or fmt!=1 or size+header.size!=len(data):continue
  try:doc=json.loads(data[header.size:])['PatrolDevice']
  except (ValueError,KeyError,TypeError):continue
  if not isinstance(doc,dict) or doc.get('Type') not in (100,1002):continue
  print(json.dumps(dict(kind='state',received_ns=received_ns,message_id=mid,PatrolDevice=doc),separators=(',',':')),flush=True)
except (BrokenPipeError,KeyboardInterrupt):pass
finally:s.close()
'''


class RelayFrames:
    """Streaming parser, also used by tests with delayed/fragmented input."""
    def __init__(self):
        self.buffer=bytearray();self.last_stamp={}
        self.stats=dict(accepted=0,rejected_stale=0,rejected_future=0,rejected_order=0,rejected_invalid=0,max_age_s=0.)
        self.source=None

    def feed(self,data,wall_ns=None,monotonic=None):
        wall_ns=time.time_ns() if wall_ns is None else wall_ns
        monotonic=time.monotonic() if monotonic is None else monotonic
        self.buffer.extend(data)
        if len(self.buffer)>262144:
            self.buffer.clear();self.stats['rejected_invalid']+=1;return []
        result=[]
        while b'\n' in self.buffer:
            line,_,rest=self.buffer.partition(b'\n');self.buffer=bytearray(rest)
            try:
                envelope=json.loads(line)
                if envelope.get('kind')=='ready':self.source=envelope.get('source');continue
                if envelope.get('kind')!='state':raise ValueError('bad envelope')
                received_ns=envelope['received_ns'];doc=envelope['PatrolDevice'];mid=envelope['message_id']
                if type(received_ns) is not int or not isinstance(doc,dict) or type(mid) is not int:raise ValueError('bad fields')
                if doc.get('Type') not in (100,1002) or type(doc.get('Command')) is not int or not isinstance(doc.get('Items'),dict):raise ValueError('bad status')
                key=(doc['Type'],doc['Command']);age=(wall_ns-received_ns)/1e9
                if age<-.1:self.stats['rejected_future']+=1;continue
                if age>1.2:self.stats['rejected_stale']+=1;continue
                if received_ns<=self.last_stamp.get(key,0):self.stats['rejected_order']+=1;continue
                self.last_stamp[key]=received_ns;age=max(0.,age)
                self.stats['accepted']+=1;self.stats['max_age_s']=max(self.stats['max_age_s'],age)
                result.append((mid,doc,monotonic-age,age))
            except (ValueError,KeyError,TypeError,AttributeError):self.stats['rejected_invalid']+=1
        return result


class StatusRelay:
    def __init__(self,target='user@10.21.31.106',password_file=None,command=None):
        self.frames=RelayFrames();self.process=None;self.retry_at=0.;self.attempts=0;self.last_io=0.;self.last_error='';self.connected=False
        self.target=target;self.retired=[]
        self.password_file=Path(password_file) if password_file else Path(__file__).resolve().parent/'runtime/private/nos_ssh_password'
        self.test_command=command

    def start(self):
        env=os.environ.copy()
        cmd=['ssh','-T','-o','ConnectTimeout=3','-o','ServerAliveInterval=1','-o','ServerAliveCountMax=2',self.target,'python3 -u -c '+shlex.quote(REMOTE_CODE)]
        password=env.get('FIRST_NOS_SSH_PASSWORD')
        if not password and self.password_file.is_file():
            if self.password_file.stat().st_mode&0o077:raise ValueError('NOS password file must have mode 600')
            password=self.password_file.read_text().rstrip('\r\n')
        if password:
            env['SSHPASS']=password;cmd=['sshpass','-e',*cmd]
        else:cmd[1:1]=['-o','BatchMode=yes']
        if self.test_command is not None:cmd=self.test_command
        self.process=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=env,start_new_session=True)
        os.set_blocking(self.process.stdout.fileno(),False);os.set_blocking(self.process.stderr.fileno(),False)
        self.frames.buffer.clear();self.frames.source=None;self.attempts+=1;self.last_io=time.monotonic();self.connected=False

    def poll(self):
        now=time.monotonic()
        for process,stopped_at in list(self.retired):
            if process.poll() is not None:self.retired.remove((process,stopped_at))
            elif now-stopped_at>.2:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
        if self.process is None:
            if now<self.retry_at:return []
            try:self.start()
            except (OSError,ValueError) as exc:self.last_error=str(exc);self.retry_at=now+1.;return []
        out=[]
        for _ in range(4):
            try:data=os.read(self.process.stdout.fileno(),65536)
            except BlockingIOError:break
            if not data:break
            self.last_io=time.monotonic();out.extend(self.frames.feed(data))
        self.connected=self.frames.source is not None
        try:
            err=os.read(self.process.stderr.fileno(),4096)
            if err:self.last_error=err.decode(errors='replace')[-1000:]
        except BlockingIOError:pass
        if self.process.poll() is not None or time.monotonic()-self.last_io>3.:
            self.last_error=self.last_error or 'NOS telemetry relay disconnected or silent'
            self.retire();self.retry_at=time.monotonic()+1.;return []
        return out

    def health(self):
        return dict(transport='nos_internal_udp_over_ssh',target=self.target,connected=self.connected,
                    attempts=self.attempts,last_error=self.last_error,source=self.frames.source,**self.frames.stats)

    def retire(self):
        """Disconnect without blocking the 30 Hz robot sender on SSH shutdown."""
        if self.process is not None:
            if self.process.poll() is None:
                try:os.killpg(self.process.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                self.retired.append((self.process,time.monotonic()))
            self.process.stdout.close();self.process.stderr.close();self.process=None
        self.connected=False

    def close(self):
        self.retire()
        for process,_ in self.retired:
            try:process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                process.wait(timeout=.5)
        self.retired.clear()
