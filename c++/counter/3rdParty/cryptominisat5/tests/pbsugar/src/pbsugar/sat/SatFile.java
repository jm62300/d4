package pbsugar.sat;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

public class SatFile {
    public static int SAT_BUFFER_SIZE = 4*1024;
    
    private String satFileName;
    private FileChannel satFileChannel = null; 
    private ByteBuffer satByteBuffer = null;
    public int variablesCount = 0;
    public int clausesCount = 0;
    
    public SatFile(String satFileName) {
        this.satFileName = satFileName;
    }

    public void open() throws IOException {
        satFileChannel = (new FileOutputStream(satFileName)).getChannel();
        // satFileChannel = (new RandomAccessFile(satFileName, "rw")).getChannel();
        satByteBuffer = ByteBuffer.allocateDirect(SAT_BUFFER_SIZE);
        String header = getHeader(0, 0);
        write(header);
    }

    public void flush() throws IOException {
        satByteBuffer.flip();
        satFileChannel.write(satByteBuffer);
        satByteBuffer.clear();
    }

    public int newVar(int size) {
        int code = variablesCount + 1;
        variablesCount += size;
        return code;
    }
    
    public int newVar() {
        return newVar(1);
    }
    
    public void write(byte[] b) throws IOException {
        if (satFileChannel == null)
            open();
        int len = b.length;
        if (satByteBuffer.position() + len <= SAT_BUFFER_SIZE) {
            satByteBuffer.put(b);
        } else {
            flush();
            int p;
            for (p = 0; p + SAT_BUFFER_SIZE <= len; p += SAT_BUFFER_SIZE) {
                satByteBuffer.put(b, p, SAT_BUFFER_SIZE);
                flush();
            }
            if (p < len)
                satByteBuffer.put(b, p, len - p);
        }
    }
    
    public void write(String s) throws IOException {
        write(s.getBytes());
    }
    
    public String getHeader(int variablesCount, int clausesCount) {
        int n = 64;
        StringBuilder s = new StringBuilder();
        s.append("p cnf ");
        s.append(Integer.toString(variablesCount));
        s.append(" ");
        s.append(Integer.toString(clausesCount));
        while (s.length() < n - 1) {
            s.append(" ");
        }
        s.append("\n");
        return s.toString();
    }

    public void updateHeader() throws IOException {
        String header = getHeader(variablesCount, clausesCount);
        RandomAccessFile satFile1 = new RandomAccessFile(satFileName, "rw");
        satFile1.seek(0);
        satFile1.write(header.getBytes());
        satFile1.close();
    }

    public void close() throws IOException {
        flush();
        satFileChannel.close();
        satFileChannel = null;
        satByteBuffer = null;
        updateHeader();
    }
    
    public void writeComment(String comment) throws IOException {
        write("c " + comment + "\n");
    }
    
    public void writeComment(String[] msg) throws IOException {
        write("c");
        for (String s : msg) {
            write(" ");
            write(s);
        }
        write("\n");
    }
    
    public void writeClause(int[] clause) throws IOException {
        for (int code : clause) {
            if (code > 0 && code > variablesCount)
                variablesCount = code; 
            else if (-code > variablesCount)
                variablesCount = -code; 
        }
        for (int code : clause)
            write(Integer.toString(code) + " ");
        write("0\n");
        clausesCount++;
    }
}
