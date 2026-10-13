#!/usr/bin/env python3
"""Read-only AOS transport comparison. Sends ONLY heartbeats/status queries."""
import argparse,collections,json,select,socket,struct,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scanplanner_integration'))
from core import packet,decode_packet,HEADER,MAGIC

p=argparse.ArgumentParser();p.add_argument('--host',default='10.21.31.103');p.add_argument('--duration',type=float,default=45)
p.add_argument('--queries',action='store_true');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True);log=(a.output/'events.jsonl').open('w');clients={};begin=time.monotonic()
def event(**kw):log.write(json.dumps(dict(monotonic=time.monotonic(),wall=time.time(),**kw))+'\n');log.flush()
for name,kind,port in [('udp',socket.SOCK_DGRAM,30000),('tcp',socket.SOCK_STREAM,30001)]:
    s=socket.socket(socket.AF_INET,kind);s.settimeout(3)
    try:
        s.connect((a.host,port))
        if name=='tcp':s.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        s.setblocking(False);clients[s]=dict(name=name,seq=0,last_hb=-1e9,buf=b'',stats={},last={},errors=[],sent={},local=s.getsockname(),connected_at=time.monotonic(),rx=0)
        event(transport=name,event='connected',local=s.getsockname())
    except OSError as ex:event(transport=name,event='connect_failed',error=str(ex));s.close()
def receive(c,data):
    t=time.monotonic();c['rx']+=1
    try:mid,d=decode_packet(data)
    except (ValueError,TypeError) as ex:event(transport=c['name'],event='decode_error',error=str(ex),bytes=len(data));return
    key=f'{d.get("Type")}/{d.get("Command")}';stats=c['stats'].setdefault(key,dict(packets=0,max_gap_s=0.,gaps_over_350ms=0,gaps_over_1200ms=0))
    if key in c['last']:
        gap=t-c['last'][key];stats['max_gap_s']=max(stats['max_gap_s'],gap);stats['gaps_over_350ms']+=gap>.35;stats['gaps_over_1200ms']+=gap>1.2
    stats['packets']+=1;c['last'][key]=t
    error=d.get('ErrorCode',d.get('Items',{}).get('ErrorCode',0))
    if error:c['errors'].append(dict(key=key,code=error))
    event(transport=c['name'],event='rx',key=key,id=mid,server_time=d.get('Time'),bytes=len(data),error=error,items_keys=list(d.get('Items',{})),heartbeat_rtt=(t-c['sent'][mid] if key=='100/100' and mid in c['sent'] else None))
def send(c,s,kind,cmd):
    seq=c['seq'];data=packet(kind,cmd,{},seq)
    # At most two tiny queries + one heartbeat; no actuator packet is constructed.
    n=s.send(data)
    if n!=len(data):raise RuntimeError('partial diagnostic send')
    c['sent'][seq]=time.monotonic();c['seq']=(seq+1)%65536;event(transport=c['name'],event='tx',key=f'{kind}/{cmd}',id=seq)
try:
    if a.queries:
        for s,c in clients.items():
            if c['name']=='udp':
                for cmd in (4,6):send(c,s,1002,cmd)
    while time.monotonic()-begin<a.duration and clients:
        for s,c in list(clients.items()):
            if time.monotonic()-c['last_hb']>=.5:
                try:send(c,s,100,100);c['last_hb']=time.monotonic()
                except OSError as ex:event(transport=c['name'],event='send_error',error=str(ex));s.close();del clients[s]
        ready,_,_=select.select(list(clients),[],[],.01)
        for s in ready:
            c=clients[s]
            try:data=s.recv(65536)
            except BlockingIOError:continue
            if not data:event(transport=c['name'],event='eof');s.close();del clients[s];continue
            if c['name']=='udp':receive(c,data);continue
            c['buf']+=data
            while len(c['buf'])>=HEADER.size:
                magic,size,seq,fmt=HEADER.unpack_from(c['buf'])
                if magic!=MAGIC or fmt!=1:raise RuntimeError('invalid TCP framing')
                length=HEADER.size+size
                if len(c['buf'])<length:break
                receive(c,c['buf'][:length]);c['buf']=c['buf'][length:]
finally:
    report=dict(duration_s=time.monotonic()-begin,queries=a.queries,transports={c['name']:dict(stats=c['stats'],errors=c['errors'],local=c['local'],frames=c['rx']) for c in clients.values()})
    (a.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));log.close()
    for s in clients:s.close()
