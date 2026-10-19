"""Bounded decoder for Quest's WZB1 lossless upload framing."""
import struct
import zstandard

BLOCK_BYTES=1024*1024
MAX_COMPRESSED_BLOCK=BLOCK_BYTES+BLOCK_BYTES//128+1024

class ZstdBlocks:
    def __init__(self):
        self.buffer=bytearray();self.magic=False;self.length=None;self.done=False
        self.decoder=zstandard.ZstdDecompressor()
    def feed(self,data):
        if self.done:
            if data:raise ValueError('Bytes after Zstd upload terminator')
            return
        self.buffer.extend(data)
        if not self.magic:
            if len(self.buffer)<4:return
            if self.buffer[:4]!=b'WZB1':raise ValueError('Invalid Zstd upload magic')
            del self.buffer[:4];self.magic=True
        while True:
            if self.length is None:
                if len(self.buffer)<4:return
                self.length=struct.unpack('>I',self.buffer[:4])[0];del self.buffer[:4]
                if self.length==0:
                    self.done=True
                    if self.buffer:raise ValueError('Bytes after Zstd upload terminator')
                    return
                if self.length>MAX_COMPRESSED_BLOCK:raise ValueError('Zstd upload block exceeds limit')
            if len(self.buffer)<self.length:return
            packed=bytes(self.buffer[:self.length]);del self.buffer[:self.length];self.length=None
            frame=zstandard.get_frame_parameters(packed)
            if not 0<frame.content_size<=BLOCK_BYTES or frame.window_size>BLOCK_BYTES or not frame.has_checksum:
                raise ValueError('Invalid Zstd upload block dimensions/checksum')
            raw=self.decoder.decompress(packed,max_output_size=BLOCK_BYTES,allow_extra_data=False)
            if len(raw)!=frame.content_size:raise ValueError('Zstd upload block size mismatch')
            yield raw
    def finish(self):
        if not self.done or self.buffer:raise ValueError('Truncated Zstd upload stream')
