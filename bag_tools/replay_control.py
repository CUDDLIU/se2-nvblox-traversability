"""Process boundary for replay ROS control; FastDDS contexts stay independent."""
from concurrent.futures import Future
import json
from pathlib import Path
import signal
import subprocess
import threading
from types import SimpleNamespace


class ReplayClient:
    def __init__(self, tools, session, env, status, reference):
        self.status_callback, self.reference_callback = status, reference
        self.ready = False
        self.latest = {}
        self.pending = {}
        self.serial = 0
        self.guard = threading.Lock()
        with (Path(session) / 'control.log').open('ab') as log:
            self.proc = subprocess.Popen(['/usr/bin/python3', str(Path(tools) / 'replay_control.py'), '--server'],
                env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log,
                text=True, bufsize=1, start_new_session=True)
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in self.proc.stdout:
            try:
                message = json.loads(line)
                if message['kind'] == 'ready':
                    self.ready = message['value']
                elif message['kind'] in ('status', 'reference'):
                    with self.guard:
                        self.latest[message['kind']] = message['data']
                else:
                    with self.guard:
                        future = self.pending.pop(message['id'], None)
                    if future is None or future.cancelled():
                        continue
                    if message.get('error'):
                        future.set_exception(RuntimeError(message['error']))
                    elif message['kind'] == 'get':
                        future.set_result(SimpleNamespace(values=[SimpleNamespace(double_value=v) for v in message['values']]))
                    else:
                        future.set_result(SimpleNamespace(result=SimpleNamespace(
                            successful=message['successful'], reason=message['reason'])))
            except (ValueError, KeyError):
                continue
        self.ready = False
        with self.guard:
            pending, self.pending = self.pending, {}
        for future in pending.values():
            if not future.done():
                future.set_exception(RuntimeError('回放调参连接已关闭'))

    def poll(self):
        with self.guard:
            latest, self.latest = self.latest, {}
        for kind in ('reference', 'status'):
            if kind in latest:
                callback = self.status_callback if kind == 'status' else self.reference_callback
                callback(SimpleNamespace(data=latest[kind]))

    def services_are_ready(self):
        return self.ready and self.proc.poll() is None

    def request(self, kind, **data):
        self.serial += 1
        future = Future()
        with self.guard:
            self.pending[self.serial] = future
        try:
            self.proc.stdin.write(json.dumps(dict(id=self.serial, kind=kind, **data)) + '\n')
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            with self.guard:
                self.pending.pop(self.serial, None)
            future.set_exception(exc)
        return future

    def get_parameters(self, names):
        return self.request('get', names=names)

    def set_parameters_atomically(self, parameters):
        return self.request('set', parameters={p.name:p.value for p in parameters})

    def close(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=2)
        self.proc.stdin.close()


def server():
    import queue
    import sys
    import time
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from std_msgs.msg import String
    rclpy.init(args=[])
    node = rclpy.create_node('se2_replay_tuning_control')
    requests = queue.Queue()
    def stdin():
        for line in sys.stdin:
            requests.put(json.loads(line))
    threading.Thread(target=stdin, daemon=True).start()
    def emit(**data):
        print(json.dumps(data), flush=True)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/se2_terrain/status', lambda m: emit(kind='status',data=m.data), qos)
    node.create_subscription(String, '/bag_rebuild/reference_status', lambda m: emit(kind='reference',data=m.data), qos)
    get = node.create_client(GetParameters, '/se2_terrain_check/get_parameters')
    set_ = node.create_client(SetParametersAtomically, '/se2_terrain_check/set_parameters_atomically')
    last_ready = 0.
    def complete(future, req):
        try:
            result = future.result()
            if req['kind'] == 'get':
                emit(kind='get', id=req['id'], values=[v.double_value for v in result.values])
            else:
                emit(kind='set', id=req['id'], successful=result.result.successful, reason=result.result.reason)
        except Exception as exc:
            emit(kind=req['kind'], id=req['id'], error=str(exc))
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.03)
            if time.monotonic()-last_ready > .5:
                emit(kind='ready', value=get.service_is_ready() and set_.service_is_ready())
                last_ready=time.monotonic()
            while not requests.empty():
                req = requests.get_nowait()
                if req['kind'] == 'get':
                    future = get.call_async(GetParameters.Request(names=req['names']))
                else:
                    parameters=[Parameter(n,Parameter.Type.DOUBLE,float(v)).to_parameter_msg() for n,v in req['parameters'].items()]
                    future = set_.call_async(SetParametersAtomically.Request(parameters=parameters))
                future.add_done_callback(lambda f, req=req: complete(f, req))
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    server()
