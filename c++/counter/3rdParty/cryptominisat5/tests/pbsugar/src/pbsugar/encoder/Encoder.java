package pbsugar.encoder;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

import pbsugar.sat.SatFile;

public class Encoder {
    public static int UNDEF = 0;
    public static int TRUE = Integer.MAX_VALUE;
    public static int FALSE = - TRUE;
    private static int auxVarCount = 0;
    
    public int debug = 0;
    private Map<String,Integer> varCodeMap = null;
    private Map<String,int[]> varDimMap = null;
    private SortedMap<Integer,String> codeVarMap = null;
    private String satFileName = null;
    public SatFile sat = null;

    public Encoder(String satFileName) {
        debug = 0;
        varCodeMap = new HashMap<String,Integer>();
        varDimMap = new HashMap<String,int[]>();
        codeVarMap = new TreeMap<Integer,String>();
        this.satFileName = satFileName;
    }

    public void open() throws IOException {
        sat = new SatFile(satFileName);
        sat.open();
    }
    
    public void close() throws IOException {
        if (sat.clausesCount == 0) {
            int code = sat.newVar();
            sat.writeClause(new int[] { code, -code });
        }
        sat.close();
    }

    public int newVar(String name, int... dim) {
        if (varCodeMap.containsKey(name))
            throw new IllegalArgumentException("Duplicate newVar " + name);
        int size = 1;
        if (dim != null) {
            for (int d : dim)
                size *= d;
        }
        int code = sat.newVar(size);
        varCodeMap.put(name, code);
        varDimMap.put(name, dim);
        codeVarMap.put(code, name);
        return code;
    }

    public String newAuxVar(String pre) {
        auxVarCount++;
        return pre + auxVarCount;
    }
    
    public int neg(int lit) {
        return - lit;
    }
    
    public int code(String name, int... is) {
        if (! varCodeMap.containsKey(name))
            throw new IllegalArgumentException("Variable not found " + name);
        int code = varCodeMap.get(name);
        int[] dim = varDimMap.get(name);
        if (is == null && dim == null)
            return code;
        if (is == null || dim == null || is.length != dim.length)
            throw new IllegalArgumentException("Illegal indices " + name);
        int k = 0;
        for (int i = 0; i < is.length; i++) {
            k *= dim[i];
            if (is[i] < 1 || is[i] > dim[i])
                throw new IllegalArgumentException("Illegal indices " + name);
            k += is[i] - 1;
        }
        return code + k;
    }
   
    public String decode(int lit) {
        if (lit == UNDEF)
            return "undef";
        if (lit == TRUE)
            return "true";
        if (lit == FALSE)
            return "false";
        String neg = "";
        if (lit < 0) {
            neg = "~";
            lit = - lit;
        }
        if (codeVarMap.headMap(lit + 1).isEmpty())
            throw new IllegalArgumentException("Illegal decode argument " + lit);
        int code0 = codeVarMap.headMap(lit + 1).lastKey();
        String name = codeVarMap.get(code0);
        int[] dim = varDimMap.get(name);
        name = neg + name;
        int k = lit - code0;
        if (dim == null || dim.length == 0) {
            if (k != 0)
                throw new IllegalArgumentException("Illegal decode argument " + lit);
            return name;
        }
        int[] is = new int[dim.length];
        for (int i = dim.length - 1; i >= 0; i--) {
            is[i] = k % dim[i] + 1;
            k /= dim[i];
        }
        for (int i : is) {
            name += "_" + i;
        }
        if (k != 0) {
            throw new IllegalArgumentException("Illegal decode argument " + lit + " : " + name);
        }
        return name;
    }
    
    public String toString(Clause clause) {
        StringBuilder sb = new StringBuilder();
        String delim = "";
        for (int lit : clause.getLiterals()) {
            sb.append(delim);
            sb.append(decode(lit));
            delim = " ";
        }
        return sb.toString();
    }
    
    public String toString(List<Clause> clauses) {
        StringBuilder sb = new StringBuilder();
        String delim = "";
        for (Clause clause : clauses) {
            sb.append(delim);
            sb.append(toString(clause));
            delim = " : ";
        }
        return sb.toString();
    }
    
    public void writeComment(String comment) throws IOException {
        sat.writeComment(comment);
    }
    
    public void writeComment(String[] msg) throws IOException {
        sat.writeComment(msg);
    }
    
    public void writeClause(Clause clause) throws IOException {
        if (debug > 0) {
            writeComment(toString(clause));
        }
        int[] literals = clause.getLiterals();
        boolean undef = false;
        int size = 0;
        for (int lit : literals) {
            if (lit == UNDEF) {
                undef = true;
                break;
            } else if (lit == TRUE) {
                size = -1;
                break;
            } else if (lit == FALSE) {
            } else {
                size++;
            }
        }
        if (undef) {
            // clause containing undef literal
        } else if (size < 0) {
            // true clause
        } else if (size == 0) {
            // false clause
            int code = sat.newVar();
            sat.writeClause(new int[] { code });
            sat.writeClause(new int[] { -code });
        } else {
            int[] dimacsClause = new int[size];
            int i = 0;
            for (int lit : literals) {
                if (lit == FALSE)
                    continue;
                dimacsClause[i++] = lit;
            }
            sat.writeClause(dimacsClause);
        }
    }
    
    public void writeClause(int... lits) throws IOException {
        writeClause(new Clause(lits));
    }
}
