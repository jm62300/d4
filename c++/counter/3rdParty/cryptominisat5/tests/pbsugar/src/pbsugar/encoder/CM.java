package pbsugar.encoder;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.List;

import pbsugar.PBSugar;
import pbsugar.pb.PBConstraint;

public abstract class CM {
    public static boolean ORDER_AXIOMS = true;
    public static boolean CHECK_DOMAIN = false;
    
    int n;
    int[] as;
    int[] xs;
    int m;
    int[] sumMax = null;
    PBEncoder encoder;
    private CMkey key = null;
    private CMkey sharableKey = null;
    String name = "CM";
    
    Domain[] domain;
    
    public CM(int[] as, int[] xs, int m, PBEncoder encoder) {
        assert(as.length == xs.length);
        this.n = as.length;
        this.as = as;
        this.xs = xs;
        this.m = m;
        this.encoder = encoder;
        sumMax = new int[n];
        int s = 0;
        for (int i = 0; i < n; i++) {
            s += this.as[i];
            sumMax[i] = s;
        }
        if (PBEncoder.USE_SPARSE_CM) {
            domain = new Domain[n];
            Domain d = new Domain(0);
            for (int i = 0; i < n; i++) {
                d = d.union(d.plus(as[i]));
                if (d.ub() >= m + 1) {
                    d = d.limit(m).union(new Domain(m));
                }
                domain[i] = d;
            }
            if (CHECK_DOMAIN) {
                BitSet set = new BitSet();
                set.set(0);
                for (int i = 0; i < n; i++) {
                    BitSet set0 = (BitSet)set.clone();
                    for (int j = 0; j >= 0; j = set0.nextSetBit(j + 1)) {
                        set.set(j + as[i]);
                    }
                    for (int j = 0; j <= domain[i].ub(); j++) {
                        if (set.get(j) != domain[i].contains(j)) {
                            PBSugar.info("ERROR Wrong domain " + domain[i] + " at " + i + " of " + this);
                        }
                    }
                }
            }
        }
        key = new CMkey(as, xs);
        int len = PBEncoder.SHARABLE_CM_LENGTH;
        if (len > 0)
            sharableKey = new CMkey(Arrays.copyOf(as, len), Arrays.copyOf(xs, len));
    }
    
    public int n() {
        return n;
    }

    public int m() {
        return m;
    }
    
    public int[] as() {
        return as;
    }

    public int[] xs() {
        return xs;
    }

    public CMkey key() {
        return key;
    }
    
    public CMkey sharableKey() {
        return sharableKey;
    }
    
    public int neg(int lit) {
        return encoder.neg(lit);
    }

    public abstract int elem0(int i, int j);
    
    public int elem(int i, int j) {
        if (j <= 0)
            return Encoder.TRUE;
        if (i <= 0)
            return Encoder.FALSE;
        if (i < 1 || i > n)
            throw new IllegalArgumentException("elem " + i + " " + j);
        if (j > sumMax[i-1])
            return Encoder.FALSE;
        /*
        if (PBEncoder.USE_DOMAIN) {
            Domain d = domain[i-1];
            if (! d.contains(j))
                throw new IllegalArgumentException("elem " + i + " " + j);
            // j = d.upper(j);
        }
        */
        return elem0(i, j);
    }

    public abstract void encode() throws IOException;
    
    public List<Clause> encodeCmp(String cmp, int b) {
        if (cmp.equals(PBConstraint.LE)) {
            if (b < 0) {
                return PBEncoder.FALSE_CNF;
            } else {
                return encoder.singletonCNF(new Clause(neg(elem(n, b+1))));
            }
        } else if (cmp.equals(PBConstraint.GE)) {
            if (b == 0) {
                return PBEncoder.TRUE_CNF;
            } else {
                return encoder.singletonCNF(new Clause(elem(n, b)));
            }
        } else if (cmp.equals(PBConstraint.EQ)) {
            if (b < 0) {
                return PBEncoder.FALSE_CNF;
            } else if (b == 0) {
                return encoder.singletonCNF(new Clause(neg(elem(n, b+1))));
            } else {
                List<Clause> clauses = new ArrayList<Clause>();
                clauses.add(new Clause(elem(n, b)));
                clauses.add(new Clause(neg(elem(n, b+1))));
                return clauses;
            }
        } else { // cmp.equals(PBConstraint.NE)
            if (b < 0) {
                return PBEncoder.TRUE_CNF;
            } else if (b == 0) {
                return encoder.singletonCNF(new Clause(elem(n, b+1)));
            } else {
                return encoder.singletonCNF(new Clause(neg(elem(n, b)), elem(n, b)));
            }
        }
    }
    
    public void encodeSum(String sum, Domain domain) throws IOException {
        int size = domain.size();
        int code0 = encoder.newVar(sum, size - 1);
        if (encoder.debug >= 2) {
            encoder.writeComment(sum + " " + code0 + " " + domain + "#" + domain.size());
        }
        if (ORDER_AXIOMS) {
            for (int index = 1; index < size - 1; index++) {
                encoder.writeClause(neg(code0 + index), (code0 + index - 1));
            }
        }
    }
    
    public int sumCode(String sum, Domain domain, int value) {
        if (value <= domain.lb())
            return Encoder.TRUE;
        if (value > domain.ub())
            return Encoder.FALSE;
        int index = 0;
        for (int[] r : domain.intervals) {
            if (value < r[0]) {
                break;
            } else if (value > r[1]) {
                index += r[1] - r[0] + 1;
            } else { // r[0] <= value <= r[1]
                index += value - r[0];
                break;
            }
        }
        if (index <= 0)
            return Encoder.TRUE;
        if (index >= domain.size())
            return Encoder.FALSE;
        int code0 = encoder.code(sum, 1);
        return code0 + index - 1;
    }
    
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(name + "(" + n + "," + m + ")" + " =");
        for (int i = 0; i < n; i++) {
            sb.append(" ");
            if (as[i] >= 0)
                sb.append("+");
            sb.append(Integer.toString(as[i]));
            sb.append(" ");
            sb.append(encoder.decode(xs[i]));
        }
        return sb.toString();
    }

}
