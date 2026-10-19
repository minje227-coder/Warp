import struct
from pathlib import Path
import sys,unittest
import zstandard
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from upload_compression import ZstdBlocks,BLOCK_BYTES,MAX_COMPRESSED_BLOCK

def framed(raw,checksum=True):
    p=zstandard.ZstdCompressor(level=1,write_checksum=checksum).compress(raw)
    return b'WZB1'+struct.pack('>I',len(p))+p+b'\0'*4

class UploadCompressionTests(unittest.TestCase):
    def decode(self,data):
        d=ZstdBlocks();out=b''.join(d.feed(data));d.finish();return out
    def test_arbitrary_chunk_boundaries_preserve_bytes(self):
        raw=bytes(range(256))*4096;wire=framed(raw);d=ZstdBlocks();result=[]
        for i in range(0,len(wire),7):result.extend(d.feed(wire[i:i+7]))
        d.finish();self.assertEqual(b''.join(result),raw)
    def test_truncated_stream_is_rejected(self):
        wire=framed(b'raw pixels')
        for cut in [1,3,4,7,len(wire)-1,len(wire)-4]:
            with self.subTest(cut=cut):
                with self.assertRaises(ValueError):self.decode(wire[:cut])
    def test_trailing_bytes_rejected(self):
        with self.assertRaises(ValueError):self.decode(framed(b'raw')+b'extra')
    def test_oversized_compressed_block_rejected_before_allocation(self):
        with self.assertRaisesRegex(ValueError,'limit'):self.decode(b'WZB1'+struct.pack('>I',MAX_COMPRESSED_BLOCK+1))
    def test_oversized_expanded_frame_rejected(self):
        with self.assertRaisesRegex(ValueError,'dimensions'):self.decode(framed(b'x'*(BLOCK_BYTES+1)))
    def test_missing_checksum_rejected(self):
        with self.assertRaisesRegex(ValueError,'checksum'):self.decode(framed(b'raw',checksum=False))
    def test_block_checksum_corruption_rejected(self):
        wire=bytearray(framed(b'raw pixels'*100));wire[-5]^=1
        with self.assertRaises(zstandard.ZstdError):self.decode(wire)
    def test_no_extra_bytes_after_terminator(self):
        d=ZstdBlocks();list(d.feed(framed(b'raw')))
        with self.assertRaises(ValueError):list(d.feed(b'extra'))
