"""Deferred-upload integrity, artifact timing, and retry regression tests."""
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest
from test_unified import metadata_v2, synthetic_jpeg
from local_episode import import_local_episode

class LocalEpisodeTest(unittest.TestCase):
    def take(self, folder):
        path = folder/'take.wql'
        jpeg = synthetic_jpeg()
        with path.open('wb') as file:
            file.write(b'WQL1'+struct.pack('<d',30))
            for index in range(1,4):
                meta = metadata_v2(index)
                meta.update(tags=[],tag_detection_pending=False)
                encoded = json.dumps(meta).encode()
                payload = struct.pack('<i',len(encoded))+encoded+jpeg
                file.write(struct.pack('<i',len(payload))+payload)
        return path

    def test_saved_artifacts_and_idempotent_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = self.take(root)
            checksum = hashlib.sha256(path.read_bytes()).hexdigest()
            receipt = import_local_episode(path,root/'output','a'*32,checksum,3)
            self.assertEqual(receipt['state'],'saved')
            self.assertEqual(receipt['source_frames'],3)
            self.assertGreater(receipt['shared_frames'],3)
            self.assertEqual(import_local_episode(path,root/'output','a'*32,checksum,3),receipt)
            with self.assertRaises(ValueError):
                import_local_episode(path,root/'output','a'*32,'b'*64,3)
            self.assertEqual(len(list((root/'output').glob('*/*/metadata.json'))),1)

    def test_truncated_take_never_acknowledged(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = self.take(root)
            path.write_bytes(path.read_bytes()[:-1])
            checksum = hashlib.sha256(path.read_bytes()).hexdigest()
            with self.assertRaises(ValueError):
                import_local_episode(path,root/'output','c'*32,checksum,3)
            self.assertFalse((root/'output'/'.uploads'/('c'*32+'.json')).exists())

if __name__ == '__main__': unittest.main()
