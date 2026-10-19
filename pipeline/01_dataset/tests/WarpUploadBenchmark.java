import dev.warp.capture.WarpUploadArchive;
import dev.warp.capture.WarpZstdOutputStream;
import java.io.*;
import java.net.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;

/** Run with app_process to compare the exact same source bytes on Quest. */
public class WarpUploadBenchmark {
    static String sha(File f)throws Exception{
        MessageDigest hash=MessageDigest.getInstance("SHA-256");byte[] b=new byte[262144];
        try(InputStream in=new FileInputStream(f)){int n;while((n=in.read(b))!=-1)hash.update(b,0,n);}
        StringBuilder s=new StringBuilder();for(byte v:hash.digest())s.append(String.format("%02x",v&255));return s.toString();
    }
    public static void main(String[] args)throws Exception{
        File directory=Files.createTempDirectory(Paths.get(args[0]),"warp-lossless-bench-").toFile();
        File sample=new File(directory,"sample.yuyv"),archive=new File(directory,"old-upload.zip");
        try{
            int frame=1920*1080*2;long frames=new File(args[1]).length()/frame;
            try(RandomAccessFile raw=new RandomAccessFile(args[1],"r");OutputStream out=new FileOutputStream(sample)){
                byte[] buffer=new byte[frame];for(long first:new long[]{0,(frames-8)/2,frames-8}){
                    raw.seek(first*frame);for(int i=0;i<8;i++){raw.readFully(buffer);out.write(buffer);}}
            }
            String sourceHash=sha(sample);List<String> names=Collections.singletonList("sample.yuyv");
            String[] modes={"legacy0","zstd1","stream0","zstd1","legacy0","zstd1"};
            for(String mode:modes){
                long started=System.nanoTime();String expected=null;long wire;WarpUploadArchive.Result result=null;
                if(mode.equals("legacy0")){
                    WarpUploadArchive.write(new FileOutputStream(archive),directory,names,0,null);expected=sha(archive);
                }
                HttpURLConnection c=(HttpURLConnection)new URL(args[2]).openConnection();
                try{
                    c.setRequestMethod("POST");c.setDoOutput(true);c.setConnectTimeout(10000);c.setReadTimeout(60000);
                    c.setRequestProperty("X-Sample-SHA256",sourceHash);
                    if(mode.equals("zstd1"))c.setRequestProperty("X-Warp-Stream","zip-zstd-block-v1");
                    if(mode.equals("legacy0")){
                        c.setFixedLengthStreamingMode(archive.length());byte[] buffer=new byte[262144];
                        try(OutputStream out=c.getOutputStream();InputStream in=new FileInputStream(archive)){
                            int n;while((n=in.read(buffer))!=-1)out.write(buffer,0,n);}
                        wire=archive.length();
                    }else{
                        c.setChunkedStreamingMode(262144);
                        WarpZstdOutputStream compressed=mode.equals("zstd1")?new WarpZstdOutputStream(c.getOutputStream()):null;
                        result=WarpUploadArchive.write(compressed==null?c.getOutputStream():compressed,directory,names,0,null);
                        expected=result.sha256;wire=compressed==null?result.wireBytes:compressed.wireBytes;
                    }
                    int code=c.getResponseCode();ByteArrayOutputStream reply=new ByteArrayOutputStream();byte[] b=new byte[4096];
                    try(InputStream in=code==200?c.getInputStream():c.getErrorStream()){int n;while((n=in.read(b))!=-1)reply.write(b,0,n);}
                    String response=reply.toString("UTF-8");
                    if(code!=200||!response.contains("\"sha256\":\""+expected+"\"")||!response.contains("\"source_match\":true"))throw new IOException(response);
                    double seconds=(System.nanoTime()-started)/1e9;
                    System.out.printf(Locale.ROOT,"BENCH_JSON {\"mode\":\"%s\",\"raw_bytes\":%d,\"wire_bytes\":%d,\"elapsed_s\":%.6f,\"source_sha256\":\"%s\",\"roundtrip_match\":true}%n",mode,sample.length(),wire,seconds,sourceHash);
                }finally{c.disconnect();}
            }
        }finally{
            Files.deleteIfExists(archive.toPath());Files.deleteIfExists(sample.toPath());Files.deleteIfExists(directory.toPath());
        }
    }
}
