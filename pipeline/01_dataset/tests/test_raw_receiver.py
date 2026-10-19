"""Exercise real ASGI streaming, lossless ingestion and lost-ack recovery."""
import hashlib
import io
import json
from pathlib import Path
import sys
import unittest
import zipfile
import struct
import zstandard
from unittest.mock import patch
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from raw_receiver import create_app
from raw_episode import digest
import test_raw_episode as fixture_module

class RawReceiverTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.fixture = fixture_module.RawEpisodeTests()
        self.fixture.setUp()
        self.f = self.fixture
        self.app = create_app(self.f.out)
        with zipfile.ZipFile(self.f.archive) as original:
            self.members = {n: original.read(n) for n in original.namelist()}
        self.manifest_hash = hashlib.sha256(self.members['manifest.json']).hexdigest()
    def tearDown(self): self.f.tearDown()
    def compressed(self):
        out = io.BytesIO()
        with zipfile.ZipFile(out, 'w', compression=zipfile.ZIP_STORED) as z:
            for n,b in self.members.items(): z.writestr(n,b)
        self.archive_bytes=out.getvalue()
        c=zstandard.ZstdCompressor(level=1,write_checksum=True)
        packed=[c.compress(self.archive_bytes[i:i+1024**2]) for i in range(0,len(self.archive_bytes),1024**2)]
        return b'WZB1'+b''.join(struct.pack('>I',len(p))+p for p in packed)+b'\0'*4
    async def request(self, method, path, body=b'', headers=None, fail=False):
        messages=[]; parts=[body[i:i+137] for i in range(0,len(body),137)] or [b'']
        async def receive():
            if fail and len(parts)<2: raise OSError('connection interrupted')
            data=parts.pop(0)
            return {'type':'http.request','body':data,'more_body':bool(parts)}
        async def send(message): messages.append(message)
        path,_,query=path.partition('?')
        scope={'type':'http','asgi':{'version':'3.0'},'http_version':'1.1','method':method,
               'scheme':'http','path':path,'raw_path':path.encode(),'query_string':query.encode(),
               'headers':[(k.lower().encode(),str(v).encode()) for k,v in (headers or {}).items()],
               'client':('127.0.0.1',1234),'server':('127.0.0.1',8080),'root_path':''}
        await self.app(scope,receive,send)
        status=next(x['status'] for x in messages if x['type']=='http.response.start')
        value=json.loads(b''.join(x.get('body',b'') for x in messages if x['type']=='http.response.body'))
        return status,value
    def headers(self):
        return {'X-Warp-Stream':'zip-zstd-block-v1','X-Warp-Source-Bytes':sum(map(len,self.members.values())),
                'X-Warp-Manifest-SHA256':self.manifest_hash,'Transfer-Encoding':'chunked'}
    async def upload(self, body=None, headers=None, **kw):
        return await self.request('POST','/upload_raw?upload_id='+self.f.episode_id,
                                  self.compressed() if body is None else body,
                                  self.headers() if headers is None else headers,**kw)
    async def lookup(self, checksum=None):
        return await self.request('GET','/upload_raw/receipt?upload_id='+self.f.episode_id+'&manifest_sha256='+(checksum or self.manifest_hash))
    def no_receipt(self):
        self.assertFalse(list(self.f.out.glob('*/*/transfer.json')))
        self.assertFalse(list((self.f.out/'.raw_incoming').iterdir()))
    async def test_chunked_compression_preserves_every_original_byte(self):
        body=self.compressed();status,r=await self.upload(body)
        self.assertEqual(status,200);self.assertEqual(r['sha256'],hashlib.sha256(self.archive_bytes).hexdigest())
        self.assertEqual(r['transport']['wire_sha256'],hashlib.sha256(body).hexdigest())
        self.assertEqual(r['manifest_sha256'],self.manifest_hash)
        for name,data in self.members.items(): self.assertEqual((Path(r['episode'])/name).read_bytes(),data)
        self.assertFalse(list((self.f.out/'.raw_incoming').iterdir()))
    async def test_source_corruption_is_not_acknowledged(self):
        self.members['tracking.jsonl']=b'corrupt raw source'
        status,r=await self.upload();self.assertEqual(status,422);self.assertIn('Source checksum',r['error']);self.no_receipt()
    async def test_manifest_header_mismatch_is_not_acknowledged(self):
        h=self.headers();h['X-Warp-Manifest-SHA256']='0'*64
        status,r=await self.upload(headers=h);self.assertEqual(status,422);self.assertIn('Manifest checksum',r['error']);self.no_receipt()
    async def test_declared_source_length_is_verified(self):
        h=self.headers();h['X-Warp-Source-Bytes']+=1
        status,r=await self.upload(headers=h);self.assertEqual(status,422);self.assertIn('source length',r['error']);self.no_receipt()
    async def test_missing_manifest_checksum_rejected(self):
        h=self.headers();del h['X-Warp-Manifest-SHA256']
        status,_=await self.upload(headers=h);self.assertEqual(status,400);self.no_receipt()
    async def test_legacy_fixed_length_client_remains_supported(self):
        body=self.f.archive.read_bytes()
        h={'Content-Length':len(body),'X-Warp-SHA256':hashlib.sha256(body).hexdigest()}
        status,r=await self.upload(body,h);self.assertEqual(status,200)
        self.assertEqual((await self.lookup())[0],200)
    async def test_lost_reply_recovered_without_reuploading_a_different_zip(self):
        status,r=await self.upload();self.assertEqual(status,200)
        self.assertEqual(await self.lookup(),(200,r))
    async def test_receipt_commit_crash_can_be_recovered(self):
        status,r=await self.upload();self.assertEqual(status,200)
        (self.f.out/'.raw_uploads'/(self.f.episode_id+'.json')).unlink()
        self.assertEqual(await self.lookup(),(200,r))
    async def test_identity_conflict_rejected(self):
        await self.upload();status,r=await self.lookup('0'*64)
        self.assertEqual(status,409);self.assertIn('different manifest',r['error'])
    async def test_missing_source_cannot_get_a_saved_receipt(self):
        _,r=await self.upload();(Path(r['episode'])/'camera.mp4').unlink()
        status,r=await self.lookup();self.assertEqual(status,409);self.assertIn('source missing',r['error'])
    async def test_unknown_episode_returns_404(self): self.assertEqual((await self.lookup())[0],404)
    async def test_interrupted_stream_leaves_no_receipt(self):
        self.assertEqual((await self.upload(fail=True))[0],422);self.no_receipt()
    async def test_disk_full_does_not_acknowledge(self):
        with patch('raw_receiver.shutil.disk_usage',return_value=SimpleNamespace(free=0)):
            status,r=await self.upload()
        self.assertEqual(status,422);self.assertIn('storage low',r['error']);self.no_receipt()
    async def test_declared_content_length_is_enforced(self):
        h=self.headers();h['Content-Length']=1
        self.assertEqual((await self.upload(headers=h))[0],422);self.no_receipt()

if __name__=='__main__': unittest.main()
