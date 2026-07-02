package pbsugar.pb;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.StreamTokenizer;
import java.io.UnsupportedEncodingException;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

public class PBParser {
    // private BufferedReader reader = null;
    private StreamTokenizer st = null;
    private String headerLine;
    public int variables;
    public int constraints;
    
    public PBParser(BufferedReader reader) throws IOException {
        // this.reader = reader;
        headerLine = reader.readLine();
        if (headerLine == null || ! headerLine.startsWith("* ") ||
                ! headerLine.matches("\\*\\s+#variable=\\s+\\d+\\s+#constraint=\\s+\\d+\\s*"))
            throw new IllegalArgumentException("Invalid header line : " + reader);
        String[] s = headerLine.split("\\s+");
        variables = Integer.parseInt(s[2]);
        constraints = Integer.parseInt(s[4]);
        st = new StreamTokenizer(reader);
        st.resetSyntax();
        st.whitespaceChars(0x0000, 0x0020);
        st.wordChars('A', 'Z');
        st.wordChars('a', 'z');
        st.wordChars('_', '_');
        st.wordChars('0', '9');
        st.commentChar('*');
        st.eolIsSignificant(false);
        st.nextToken();
    }

    public PBParser(String fileName) throws UnsupportedEncodingException, IOException {
        this(new BufferedReader(new InputStreamReader(new FileInputStream(fileName), "UTF-8")));
    }
    
    private BigInteger parseBigInteger() throws IOException {
        BigInteger a = BigInteger.ONE;  
        if (st.ttype == '-' || st.ttype == '+') {
            boolean negative = st.ttype == '-';
            st.nextToken();
            if (st.ttype != StreamTokenizer.TT_WORD || ! st.sval.matches("\\d+"))
                throw new IllegalArgumentException("Format error at line " + st.lineno());
            a = negative ? new BigInteger("-" + st.sval) : new BigInteger(st.sval);
            st.nextToken();
        } else if (st.ttype == StreamTokenizer.TT_WORD && st.sval.matches("\\d+")) {
            a = new BigInteger(st.sval);
            st.nextToken();
        }
        return a;
    }
    
    private PBLiteral parseLiteral() throws IOException {
        PBLiteral x = null;
        boolean negative = false;
        if (st.ttype == '~') {
            negative = true;
            st.nextToken();
        }
        if (st.ttype != StreamTokenizer.TT_WORD || st.sval.matches("\\d+"))
            throw new IllegalArgumentException("Format error at line " + st.lineno());
        x = new PBLiteral(st.sval, negative);
        st.nextToken();
        return x;
    }
    
    private String parseCmp() throws IOException {
        String cmp = null;
        int t = st.ttype;
        switch (t) {
        case '>':
        case '<':
            cmp = Character.toString((char)t);
            st.nextToken();
            if (st.ttype == '=') {
                cmp += "=";
                st.nextToken();
            }
            break;
        case '!':
            st.nextToken();
            if (st.ttype != '=')
                throw new IllegalArgumentException("Format error at line " + st.lineno());
            st.nextToken();
            break;
        case '=':
            cmp = "=";
            st.nextToken();
            break;
        }
        return cmp;
    }
    
    public PBConstraint getNext() throws IOException {
        if (st == null || st.ttype == StreamTokenizer.TT_EOF) {
            st = null;
            return null;
        }            
        List<BigInteger> as = new ArrayList<BigInteger>();
        List<PBLiteral> xs = new ArrayList<PBLiteral>();
        String cmp = null;
        while (st.ttype != StreamTokenizer.TT_EOF && cmp == null) {
            BigInteger a = parseBigInteger();
            PBLiteral x = parseLiteral();
            as.add(a);
            xs.add(x);
            cmp = parseCmp();
        }
        BigInteger b = parseBigInteger();
        if (st.ttype != ';')
            throw new IllegalArgumentException("Format error at line " + st.lineno());
        st.nextToken();
        PBConstraint pb = new PBConstraint(as, xs, cmp, b);
        return pb;
    }

}
