import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch
import yaml
from replay_transforms import prepare, source_key


class ReplayCacheTests(unittest.TestCase):
    def test_reuse_invalidate_and_recover_incomplete_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            bag=Path(directory);(bag/'data').mkdir()
            (bag/'data/metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'relative_file_paths':['data_0.db3']}}))
            source=bag/'data/data_0.db3';source.write_bytes(b'original source')
            def build(bag,session):
                with sqlite3.connect(session/'timed_transforms.db3') as db:
                    db.execute('CREATE TABLE events(stamp INTEGER,kind TEXT,data BLOB)')
                    db.execute("INSERT INTO events VALUES(123,'tf',?)",(b'measured transform',))
                (session/'timed_transforms.json').write_text(json.dumps(dict(model=False)))
            with patch('replay_transforms._build',side_effect=build) as mocked:
                sessions=[]
                for i in range(2):
                    session=bag/str(i);session.mkdir();sessions.append(session);prepare(bag,session)
                self.assertEqual(mocked.call_count,1)
                self.assertTrue(json.loads((sessions[1]/'prepare_timing.json').read_text())['cache_hit'])
                self.assertEqual(source.read_bytes(),b'original source')
                source.write_bytes(b'changed recording')
                third=bag/'2';third.mkdir();prepare(bag,third)
                self.assertEqual(mocked.call_count,2)
                cache=bag/'.cache'/('timed_tf_'+source_key(bag))
                (cache/'timed_transforms.db3').write_bytes(b'corrupted cache')
                fourth=bag/'3';fourth.mkdir();prepare(bag,fourth)
                self.assertEqual(mocked.call_count,3)
                self.assertFalse(json.loads((fourth/'prepare_timing.json').read_text())['cache_hit'])


if __name__=='__main__':unittest.main()
