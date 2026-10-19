package dev.warp.capture;

import java.io.*;
import java.security.*;
import java.util.*;
import java.util.zip.*;

/** Bounded-memory, deterministic lossless ZIP stream; never creates a local archive. */
public final class WarpUploadArchive {
    public interface Progress { void update(long sourceBytes, long wireBytes) throws Exception; }
    public static final class Result {
        public final long sourceBytes, wireBytes;
        public final String sha256;
        Result(long source, long wire, String sha) { sourceBytes=source;wireBytes=wire;sha256=sha; }
    }
    private static final class Counted extends FilterOutputStream {
        long bytes;
        Counted(OutputStream out) { super(out); }
        @Override public void write(int b) throws IOException { out.write(b);bytes++; }
        @Override public void write(byte[] b,int off,int len) throws IOException { out.write(b,off,len);bytes+=len; }
    }
    public static Result write(OutputStream output, File directory, Collection<String> names,
                               int level, Progress progress) throws Exception {
        File root=directory.getCanonicalFile();
        List<String> ordered=new ArrayList<>(names);Collections.sort(ordered);
        if(new HashSet<>(ordered).size()!=ordered.size())throw new IOException("Duplicate archive member");
        List<File> files=new ArrayList<>();
        for(String name:ordered) {
            if(!name.matches("[A-Za-z0-9_.-]+")||name.equals(".")||name.equals(".."))throw new IOException("Invalid archive member");
            File f=new File(root,name).getCanonicalFile();
            if(!root.equals(f.getParentFile())||!f.isFile())throw new IOException("Missing/unsafe archive source: "+name);
            files.add(f);
        }
        MessageDigest digest=MessageDigest.getInstance("SHA-256");
        Counted counted=new Counted(output);long source=0;
        try(ZipOutputStream zip=new ZipOutputStream(new BufferedOutputStream(new DigestOutputStream(counted,digest),262144))) {
            zip.setLevel(level);byte[] buffer=new byte[65536];
            for(int i=0;i<ordered.size();i++) {
                ZipEntry entry=new ZipEntry(ordered.get(i));entry.setTime(0L);zip.putNextEntry(entry);
                File file=files.get(i);long expected=file.length(),read=0;
                try(InputStream in=new FileInputStream(file)) {
                    int n;while((n=in.read(buffer))!=-1) {
                        zip.write(buffer,0,n);source+=n;read+=n;
                        if(progress!=null)progress.update(source,counted.bytes);
                    }
                }
                if(read!=expected||file.length()!=expected)throw new IOException("Source changed during upload");
                zip.closeEntry();
            }
        }
        StringBuilder hex=new StringBuilder();for(byte b:digest.digest())hex.append(String.format(Locale.ROOT,"%02x",b&255));
        return new Result(source,counted.bytes,hex.toString());
    }
    private WarpUploadArchive() {}
}
