package pbsugar.encoder;

import java.util.Arrays;
import java.util.List;

public class Clause {
    public static Clause FALSE_CLAUSE = new Clause();
    
    private int[] literals;
    
    public Clause() {
        literals = new int[0];
    }
    
    public Clause(int... lits) {
        this.literals = lits;
    }
    
    public Clause(List<Integer> lits) {
        literals = new int[lits.size()];
        int i = 0;
        for (int lit : lits)
            literals[i++] = lit;
    }
    
    public Clause or(int... lits) {
        int n = literals.length;
        int[] literals1 = Arrays.copyOf(literals, n + lits.length);
        for (int lit : lits)
            literals1[n++] = lit;
        return new Clause(literals1);
    }

    public Clause or(Clause c) {
        return or(c.literals);
    }
    
    public boolean isUnsat() {
        return literals.length == 0;
    }

    public int[] getLiterals() {
        return literals;
    }
    
    /*
    public String toString() {
        StringBuilder sb = new StringBuilder();
        String delim = "";
        for (int lit : literals) {
            sb.append(delim);
            sb.append(lit);
            delim = " ";
        }
        return sb.toString();
    }
    */
}
