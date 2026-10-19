package dev.warp.capture;

import com.github.luben.zstd.Zstd;
import com.github.luben.zstd.ZstdCompressCtx;
import java.io.*;
import java.security.*;
import java.util.Locale;

/** WZB1: bounded independent 1 MiB Zstd frames, uint32 length prefix, zero terminator. */
public final class WarpZstdOutputStream extends OutputStream {
    public static final int BLOCK_BYTES=1024*1024;
    private final byte[] source=new byte[BLOCK_BYTES];
    private final byte[] packed=new byte[(int)Zstd.compressBound(BLOCK_BYTES)];
    private final ZstdCompressCtx context=new ZstdCompressCtx().setLevel(1).setChecksum(true).setContentSize(true);
    private final MessageDigest digest;
    private final DataOutputStream out;
    private int used;
    private boolean closed;
    public long wireBytes;
    public String sha256;
    public WarpZstdOutputStream(OutputStream output)throws Exception{
        digest=MessageDigest.getInstance("SHA-256");
        out=new DataOutputStream(new BufferedOutputStream(new DigestOutputStream(output,digest),262144));
        out.writeInt(0x575a4231);wireBytes=4;
    }
    private void block()throws IOException{
        if(used==0)return;
        try{
            int n=context.compressByteArray(packed,0,packed.length,source,0,used);
            out.writeInt(n);out.write(packed,0,n);wireBytes+=4+n;used=0;
        }catch(RuntimeException error){throw new IOException("Zstd compression failed",error);}
    }
    @Override public void write(int b)throws IOException{byte[] one={(byte)b};write(one,0,1);}
    @Override public void write(byte[] b,int off,int len)throws IOException{
        if(closed)throw new IOException("Closed Zstd upload stream");
        while(len>0){int n=Math.min(len,source.length-used);System.arraycopy(b,off,source,used,n);used+=n;off+=n;len-=n;if(used==source.length)block();}
    }
    @Override public void flush()throws IOException{out.flush();}
    @Override public void close()throws IOException{
        if(closed)return;closed=true;
        try{
            block();out.writeInt(0);wireBytes+=4;out.close();
            StringBuilder s=new StringBuilder();for(byte b:digest.digest())s.append(String.format(Locale.ROOT,"%02x",b&255));sha256=s.toString();
        }finally{context.close();}
    }
}
