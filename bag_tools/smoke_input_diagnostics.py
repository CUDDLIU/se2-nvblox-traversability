"""Explicit Jetson live acceptance; deletes only the short bag it creates."""
import json
import time
from bag_manager import BagManager


manager = BagManager()
report = {}
key = None
try:
    # Allow a newly restarted diagnostic service to collect pre-record samples.
    time.sleep(3)
    key = manager.record('自动验收：输入诊断（通过后删除）', new_map=False)
    bag = manager.active
    print('Recording diagnostic acceptance: '+key, flush=True)
    time.sleep(10)
    manager.finish_record()
    folder = bag/'diagnostics'
    rows = [json.loads(line) for line in (folder/'samples.jsonl').read_text().splitlines()]
    recent = rows[-5:]
    assert len(rows) >= 8, len(rows)
    for topic in ('lidar','imu','odom','depth','mesh','pose_tf'):
        assert any(row['streams'][topic]['hz'] > 0 for row in recent), topic
    assert all(row['journal_process_alive'] for row in recent)
    assert (folder/'pre_record.jsonl').is_file()
    assert (folder/'journal.jsonl').stat().st_size > 0
    assert (folder/'lio_events.jsonl').stat().st_size > 0
    assert not (manager.runtime/'diagnostic_target.json').exists()
    manifest = json.loads((bag/'session.json').read_text())
    assert manifest['topics'].get('/nvblox_lio/diagnostics',0) > 0
    size = (folder/'samples.jsonl').stat().st_size
    time.sleep(2)
    assert (folder/'samples.jsonl').stat().st_size == size
    report.update(samples=len(rows), latest=rows[-1], recorded_diagnostics=manifest['topics']['/nvblox_lio/diagnostics'])
    manager.move_to_trash(key)
    assert (manager.trash/key/'diagnostics/samples.jsonl').exists()
    manager.restore(key)
    assert (bag/'diagnostics/samples.jsonl').stat().st_size == size
    manager.move_to_trash(key)
    manager.purge(key)
    assert not bag.exists() and not (manager.trash/key).exists()
    report.update(result='PASS', bag_lifecycle=True, test_bag_deleted=True)
except Exception as error:
    report.update(result='FAIL', error=repr(error), bag=key)
    raise
finally:
    manager.close()
    (manager.runtime/'input_diagnostics_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k != 'latest'},ensure_ascii=False),flush=True)
