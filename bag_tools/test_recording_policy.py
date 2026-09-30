import tempfile
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from recording_policy import recorder_env, cache_losses, critical_qos, NS


class RecordingPolicyTests(unittest.TestCase):
    def test_transport_preserves_network_and_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); source=root/'original.xml'
            source.write_text(f'''<profiles xmlns="{NS}"><transport_descriptors>
<transport_descriptor><transport_id>network</transport_id><type>UDPv4</type>
<interfaceWhiteList><address>10.21.31.120</address></interfaceWhiteList></transport_descriptor>
<transport_descriptor><transport_id>memory</transport_id><type>SHM</type></transport_descriptor>
</transport_descriptors><participant><rtps><userTransports><transport_id>network</transport_id>
<transport_id>memory</transport_id></userTransports></rtps></participant></profiles>''')
            before=source.read_bytes();base={'FASTRTPS_DEFAULT_PROFILES_FILE':str(source),'ROS_DOMAIN_ID':'0'}
            output=root/'record.xml';env=recorder_env(base,output)
            self.assertEqual(source.read_bytes(),before)
            self.assertEqual(base['FASTRTPS_DEFAULT_PROFILES_FILE'],str(source))
            self.assertEqual(env['ROS_DOMAIN_ID'],'0')
            q=lambda name:'{'+NS+'}'+name
            tree=ET.parse(output)
            self.assertEqual(tree.findtext('.//'+q('address')),'10.21.31.120')
            self.assertEqual(tree.findtext('.//'+q('port_queue_capacity')),'2048')
            self.assertEqual(len(tree.findall('.//'+q('userTransports')+'/'+q('transport_id'))),2)

    def test_loss_counters_are_cumulative(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'record.log';p.write_text('Total lost: 10\nTotal lost: 1111\nTotal lost: 1111\n')
            self.assertEqual(cache_losses(p),1111)
            self.assertEqual(cache_losses(p.parent/'absent'),0)

    def test_critical_queues_do_not_expand_large_image_history(self):
        qos=critical_qos()
        self.assertGreaterEqual(qos['/tf']['depth'],1000)
        self.assertEqual(qos['/tf']['reliability'],'reliable')
        self.assertFalse(any('image' in name or 'cloud' in name for name in qos))


if __name__=='__main__':unittest.main()
