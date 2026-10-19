import dev.warp.capture.WarpUploadArchive;
import java.io.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;
import java.util.zip.*;

public class WarpUploadArchiveTest {
    static void check(boolean value,String why){if(!value)throw new AssertionError(why);}
    static byte[] encode(File root,List<String> names,int level)throws Exception{
        ByteArrayOutputStream output=new ByteArrayOutputStream();long[] last={0,0};
        WarpUploadArchive.Result r=WarpUploadArchive.write(output,root,names,level,(raw,wire)->{
            check(raw>=last[0]&&wire>=last[1],"non-monotonic progress");last[0]=raw;last[1]=wire;});
        byte[] b=output.toByteArray();check(r.wireBytes==b.length,"wire byte count");
        StringBuilder hash=new StringBuilder();for(byte v:MessageDigest.getInstance("SHA-256").digest(b))hash.append(String.format("%02x",v&255));
        check(r.sha256.equals(hash.toString()),"wire SHA-256");
        check(r.sourceBytes==last[0],"raw byte count");return b;
    }
    public static void main(String[] args)throws Exception{
        Path root=Files.createTempDirectory("warp-upload-test-");
        try{
            byte[] raw=new byte[1024*1024];new Random(42).nextBytes(raw);
            // Mix repeated image-like structure with non-compressible bytes.
            for(int i=raw.length/2;i<raw.length;i++)raw[i]=raw[i%4096];
            Files.write(root.resolve("external_camera.yuyv"),raw);
            Files.write(root.resolve("manifest.json"),"{\"schema_version\":3}".getBytes("UTF-8"));
            List<String> names=Arrays.asList("manifest.json","external_camera.yuyv");
            byte[] packed=encode(root.toFile(),names,1);
            check(Arrays.equals(packed,encode(root.toFile(),Arrays.asList("external_camera.yuyv","manifest.json"),1)),"non-deterministic retry bytes");
            check(packed.length<encode(root.toFile(),names,0).length,"compression did not reduce wire bytes");
            Set<String> seen=new HashSet<>();
            try(ZipInputStream z=new ZipInputStream(new ByteArrayInputStream(packed))){
                ZipEntry e;while((e=z.getNextEntry())!=null){ByteArrayOutputStream data=new ByteArrayOutputStream();byte[] buffer=new byte[4096];int n;
                    while((n=z.read(buffer))!=-1)data.write(buffer,0,n);
                    check(Arrays.equals(data.toByteArray(),Files.readAllBytes(root.resolve(e.getName()))),"lossy source bytes");seen.add(e.getName());}
            }
            check(seen.equals(new HashSet<>(names)),"wrong manifest member set");
            try{encode(root.toFile(),Arrays.asList("../outside"),1);throw new AssertionError("traversal accepted");}catch(IOException expected){}
            try{encode(root.toFile(),Arrays.asList("manifest.json","manifest.json"),1);throw new AssertionError("duplicate accepted");}catch(IOException expected){}
            try{WarpUploadArchive.write(new ByteArrayOutputStream(),root.toFile(),names,1,(a,b)->{throw new IOException("interrupted");});
                throw new AssertionError("interrupted stream accepted");}catch(IOException expected){}
            check(Files.list(root).count()==2,"created a temporary archive or changed sources");
            System.out.println("Lossless streaming, deterministic retry, SHA-256 and interruption cases passed");
        }finally{try(java.util.stream.Stream<Path> paths=Files.walk(root)){paths.sorted(Comparator.reverseOrder()).forEach(p->{try{Files.delete(p);}catch(IOException e){throw new RuntimeException(e);}});}}
    }
}
