package pbsugar.encoder;

import java.io.IOException;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map.Entry;

import pbsugar.pb.PBConstraint;
import pbsugar.pb.PBLiteral;

public class PBEncoder extends Encoder {
    public static List<Clause> TRUE_CNF = Collections.emptyList();
    public static List<Clause> FALSE_CNF = Collections.singletonList(Clause.FALSE_CLAUSE);
    public static int countCM = 0;
    public static int DECOMPOSE_PB = 100;
    public static int countDecompose = 0;
    // public static BigInteger BASE = new BigInteger("120");
    public static BigInteger BASE = null;
    public static boolean ENCODE_CLAUSE_PART = true;
    public static int ENCODE_AS_CNF_LITERALS_SIZE = 3;
    public static boolean SUM_CARRIES = false;
    public static boolean USE_CM_CACHE = true;
    public static int CM_CACHE_SIZE = 1000;
    public static int countCMreused = 0;
    private static CMcache cmCache = new CMcache(CM_CACHE_SIZE, 0.75f, true);
    private static class CMcache extends LinkedHashMap<CMkey,CM> { 
        private static final long serialVersionUID = 8752083187832081579L;

        public CMcache(int capacity, float loadFactor, boolean accessOrder) {
            super(capacity, loadFactor, accessOrder);
        }

        protected boolean removeEldestEntry(Entry<CMkey,CM> eldest) {
            return size() > CM_CACHE_SIZE;
        }
    }
    public static int SHARABLE_CM_LENGTH = 4;
    public static int SHARABLE_CM_CACHE_SIZE = 1000;
    public static int countCMshared = 0;
    private static SharableCMcache sharableCmCache = new SharableCMcache(SHARABLE_CM_CACHE_SIZE, 0.75f, true); 
    private static class SharableCMcache extends LinkedHashMap<CMkey,CM> { 
        private static final long serialVersionUID = -7551421904672362743L;

        public SharableCMcache(int capacity, float loadFactor, boolean accessOrder) {
            super(capacity, loadFactor, accessOrder);
        }

        protected boolean removeEldestEntry(Entry<CMkey,CM> eldest) {
            return size() > SHARABLE_CM_CACHE_SIZE;
        }
    }
    public static boolean USE_SPARSE_CM = true;
    
    public PBEncoder(String satFileName) {
        super(satFileName);
    }
    
    public int pbLiteral(PBLiteral lit) {
        int code = code(lit.getVariable());
        return lit.isNegative() ? neg(code) : code;
    }
    
    public List<Clause> singletonCNF(Clause clause) {
        return Collections.singletonList(clause);
    }
    
    public List<Clause> or(List<Clause> cs, Clause c) {
        List<Clause> clauses = new ArrayList<Clause>();
        for (Clause clause : cs)
            clauses.add(clause.or(c));
        return clauses;
    }
    
    public List<Clause> and(List<Clause> cs, Clause c) {
        if (c.isUnsat())
            return FALSE_CNF;
        List<Clause> clauses = new ArrayList<Clause>(cs);
        clauses.add(c);
        return clauses;
    }

    public List<Clause> or(List<Clause> cs1, List<Clause> cs2) {
        List<Clause> clauses = new ArrayList<Clause>();
        if (cs1.size() > 0 && cs2.size() > 0) {
            for (Clause c1 : cs1)
                for (Clause c2 : cs2)
                    clauses.add(c1.or(c2));
        }
        return clauses;
    }
    
    public List<Clause> and(List<Clause> cs1, List<Clause> cs2) {
        List<Clause> clauses = new ArrayList<Clause>(cs1);
        clauses.addAll(cs2);
        return clauses;
    }
    
    private List<Clause> encodeAsCNF(PBConstraint pb, int i, BigInteger lhs) {
        boolean valid = false;
        boolean unsat = false;
        BigInteger lb = lhs;
        BigInteger ub = lhs;
        for (int j = i; j <= pb.size(); j++)
            ub = ub.add(pb.a(j));
        if (pb.cmp().equals(PBConstraint.LE)) {
            if (ub.compareTo(pb.b()) <= 0)
                valid = true;
            if (lb.compareTo(pb.b()) > 0)
                unsat = true;
        } else if (pb.cmp().equals(PBConstraint.GE)) {
            if (lb.compareTo(pb.b()) >= 0)
                valid = true;
            if (ub.compareTo(pb.b()) < 0)
                unsat = true;
        } else if (pb.cmp().equals(PBConstraint.EQ)) {
            if (lb.compareTo(pb.b()) == 0 && ub.compareTo(pb.b()) == 0)
                valid = true;
            if (lb.compareTo(pb.b()) > 0 || ub.compareTo(pb.b()) < 0)
                unsat = true;
        } else {
            if (lb.compareTo(pb.b()) > 0 || ub.compareTo(pb.b()) < 0)
                valid = true;
            if (lb.compareTo(pb.b()) == 0 && ub.compareTo(pb.b()) == 0)
                unsat = true;
        }
        if (valid)
            return TRUE_CNF;
        if (unsat)
            return FALSE_CNF;
        List<Clause> clauses1 = or(encodeAsCNF(pb, i+1, lhs), new Clause(pbLiteral(pb.x(i))));
        List<Clause> clauses2 = or(encodeAsCNF(pb, i+1, lhs.add(pb.a(i))), new Clause(pbLiteral(pb.x(i).negate())));
        return and(clauses1, clauses2);
    }

    private List<Clause> encodeGeAsCNF(PBConstraint pb) {
        if (pb.isUnsat())
            return FALSE_CNF;
        if (pb.isValid())
            return TRUE_CNF;
        List<BigInteger> as = pb.as().subList(1, pb.size());
        List<PBLiteral> xs = pb.xs().subList(1, pb.size());
        String cmp = pb.cmp();
        BigInteger b = pb.b();
        PBConstraint pb1 = new PBConstraint(as, xs, cmp, b.subtract(pb.a(1)));
        List<Clause> clauses1 = encodeGeAsCNF(pb1);
        Clause clause = new Clause(pbLiteral(pb.x(1)));
        PBConstraint pb2 = new PBConstraint(as, xs, cmp, b);
        List<Clause> clauses2 = or(encodeGeAsCNF(pb2), clause);
        return and(clauses1, clauses2);
    }

    public List<Clause> encodeAsCNF(PBConstraint pb) {
        if (pb.cmp().equals(PBConstraint.LE)) {
            BigInteger b = pb.ubLHS().subtract(pb.b());
            List<PBLiteral> xs = new ArrayList<PBLiteral>(pb.xs());
            for (int i = 0; i < xs.size(); i++)
                xs.set(i, xs.get(i).negate());
            PBConstraint pb1 = new PBConstraint(pb.as(), xs, PBConstraint.GE, b);
            return encodeGeAsCNF(pb1);
        } else if (pb.cmp().equals(PBConstraint.GE)) {
            return encodeGeAsCNF(pb);
        } else if (pb.cmp().equals(PBConstraint.EQ)) {
            PBConstraint pb1 = new PBConstraint(pb.as(), pb.xs(), PBConstraint.LE, pb.b());
            PBConstraint pb2 = new PBConstraint(pb.as(), pb.xs(), PBConstraint.GE, pb.b());
            return and(encodeAsCNF(pb1), encodeAsCNF(pb2));
        } else { // pb.cmp().equals(PBConstraint.NE)
            PBConstraint pb1 = new PBConstraint(pb.as(), pb.xs(), PBConstraint.LE, pb.b().subtract(BigInteger.ONE));
            PBConstraint pb2 = new PBConstraint(pb.as(), pb.xs(), PBConstraint.GE, pb.b().add(BigInteger.ONE));
            return or(encodeAsCNF(pb1), encodeAsCNF(pb2));
        }
    }

    private int[] asInt(PBConstraint pb) {
        int[] as = new int[pb.as().size()];
        for (int i = 0; i < as.length; i++)
            as[i] = pb.a(i+1).intValue();
        return as;
    }
    
    private int[] xsInt(PBConstraint pb) {
        int[] xs = new int[pb.xs().size()];
        for (int i = 0; i < xs.length; i++)
            xs[i] = pbLiteral(pb.x(i+1));
        return xs;
    }
    
    private CM encodeByCounterMatrix(PBConstraint pb) throws IOException {
        int[] as = asInt(pb);
        int[] xs = xsInt(pb);
        int m = pb.b().intValue() + 1;
        CM z = new SimpleCM(as, xs, m, this);
        if (USE_CM_CACHE) {
            CM z1 = cmCache.get(z.key());
            if (z1 != null && m <= z1.m()) {
                countCMreused++;
                if (debug >= 1)
                    writeComment("# CM reused");
                return z1;
            }
        }
        if (SHARABLE_CM_LENGTH > 0 && z.n() >= SHARABLE_CM_LENGTH) {
            CM z1 = sharableCmCache.get(z.sharableKey()); 
            if (z1 != null && (! PBEncoder.USE_SPARSE_CM || m <= z1.m())) {
                countCMshared++;
                if (debug >= 1)
                    writeComment("# CM shared");
                z = new SharedCM(as, xs, m, z1, this);
            } else {
                sharableCmCache.put(z.sharableKey(), z);
            }
        }
        if (debug >= 1)
            writeComment("# CM : " + z.toString());
        countCM++;
        z.encode();
        if (USE_CM_CACHE)
            cmCache.put(z.key(), z);
        return z;
    }
    
    public PBConstraint encodeClausePart(PBConstraint pb, List<Integer> literals, List<Clause> clauses) throws IOException {
        boolean change = true;
        while (change && ! pb.isValid() && ! pb.isUnsat()) {
            change = false;
            List<BigInteger> as = new ArrayList<BigInteger>();
            List<PBLiteral> xs = new ArrayList<PBLiteral>();
            BigInteger b = pb.b();
            for (int i = 1; i <= pb.size(); i++) {
                if (pb.isUnsatWhen(i, 0)) {
                    if (debug >= 2)
                        writeComment("# Unsat when "+ pb.x(i) + "=0");
                    change = true;
                    clauses.add((new Clause(literals)).or(pbLiteral(pb.x(i))));
                    b = b.subtract(pb.a(i));
                } else if (pb.isUnsatWhen(i, 1)) {
                    if (debug >= 2)
                        writeComment("# Unsat when "+ pb.x(i) + "=1");
                    change = true;
                    clauses.add((new Clause(literals)).or(pbLiteral(pb.x(i).negate())));
                } else {
                    as.add(pb.a(i));
                    xs.add(pb.x(i));
                }
            }
            pb = new PBConstraint(as, xs, pb.cmp(), b);
            pb.normalize();
            if (debug >= 2)
                writeComment("# Remaining PB "+ pb.toString());
            as = new ArrayList<BigInteger>();
            xs = new ArrayList<PBLiteral>();
            b = pb.b();
            for (int i = 1; i <= pb.size(); i++) {
                if (pb.isValidWhen(i, 0)) {
                    if (debug >= 2)
                        writeComment("# Valid when "+ pb.x(i) + "=0");
                    change = true;
                    literals.add(pbLiteral(pb.x(i).negate()));                
                    b = b.subtract(pb.a(i));
                } else if (pb.isValidWhen(i, 1)) {
                    if (debug >= 2)
                        writeComment("# Valid when "+ pb.x(i) + "=1");
                    change = true;
                    literals.add(pbLiteral(pb.x(i)));                
                } else {
                    as.add(pb.a(i));
                    xs.add(pb.x(i));
                }
            }
            pb = new PBConstraint(as, xs, pb.cmp(), b);
            pb.normalize();
            if (debug >= 2)
                writeComment("# Remaining PB "+ pb.toString());
        }
        return pb;
    }

    /*
    private List<PBLiteral> list2literals(List<String> list) {
        List<PBLiteral> literals = new ArrayList<PBLiteral>();
        for (String x : list)
            literals.add(new PBLiteral(x));
        return literals;
    }
    */
    
    private List<Clause> encodePB_(PBConstraint pb) throws IOException {
        BigInteger base = BASE;
        if (base == null || base.compareTo(BigInteger.ONE) <= 0 || pb.b().compareTo(base) < 0) {
            if (debug >= 1)
                writeComment("# CM for " + pb.toString());
            CM z = encodeByCounterMatrix(pb);
            List<Clause> clauses = z.encodeCmp(pb.cmp(), pb.b().intValue());
            if (debug >= 1)
                writeComment("# CM result " + toString(clauses));
            return clauses;
        } else {
            // TODO
            throw new IllegalArgumentException("Not implemented");
        }
    }
    
    private List<Clause> decomposePB(PBConstraint pb) throws IOException {
        BigInteger base = new BigInteger("2");
        for (int ibase = base.intValue(); ibase <= DECOMPOSE_PB; ibase++) {
            PBConstraint[] pbs = pb.divideAndRemainder(base);
            PBConstraint pbH = pbs[0]; 
            PBConstraint pbL = pbs[1]; 
            PBConstraint pbH1 = null;
            if (pb.cmp().equals(PBConstraint.LE)) {
                BigInteger b = pbH.b().subtract(BigInteger.ONE);
                pbH1 = new PBConstraint(pbH.as(), pbH.xs(), pbH.cmp(), b);
                pbH1.normalize();
            } else if (pb.cmp().equals(PBConstraint.GE)) {
                BigInteger b = pbH.b().add(BigInteger.ONE);
                pbH1 = new PBConstraint(pbH.as(), pbH.xs(), pbH.cmp(), b);
                pbH1.normalize();
            } else { // pb.cmp().equals(PBConstraint.EQ)
            }
            pbH.normalize();
            pbL.normalize();
            if (pbH.size() == 0)
                break;
            if (pbL.ubLHS().compareTo(base) < 0) {
                writeComment("Found decomposable PB with " + base + " for " + pb);
                countDecompose++;
                if (pb.cmp().equals(PBConstraint.EQ))
                    return and(encodePB(pbH), encodePB(pbL));
                if (pbL.isValid())
                    return encodePB(pbH);
                if (pbL.isUnsat())
                    return encodePB(pbH1);
                return and(encodePB(pbH), or(encodePB(pbH1), encodePB(pbL)));
            }
            base = base.add(BigInteger.ONE);
        }
        return null;
    }
    
    public List<Clause> encodePB(PBConstraint pb) throws IOException {
        if (debug >= 1)
            writeComment("# Encoding PB " + pb.toString());
        List<Integer> literals1 = new ArrayList<Integer>();
        List<Clause> clauses1 = new ArrayList<Clause>();
        PBConstraint pb1 = pb;
        if (ENCODE_CLAUSE_PART) {
            pb1 = encodeClausePart(pb, literals1, clauses1);
        }
        // pb <==> (pb1 or literals1) and clauses1
        List<Clause> clauses = null;
        if (DECOMPOSE_PB > 0) {
            clauses = decomposePB(pb1);
        }
        if (clauses == null) {
            if (pb1.isValid()) {
                clauses = TRUE_CNF;
            } else if (pb1.isUnsat()) {
                clauses = FALSE_CNF;
            } else if (pb1.size() <= ENCODE_AS_CNF_LITERALS_SIZE) {
                clauses = encodeAsCNF(pb1);
            } else {
                clauses = encodePB_(pb1);
            }
        }
        clauses = and(clauses1, or(clauses, new Clause(literals1)));
        if (debug >= 1)
            writeComment("# Result PB " + toString(clauses));
        return clauses;
    }
    
    public void encode(PBConstraint pb) throws IOException {
        pb.normalize();
        if (debug >= 1)
            writeComment("# Input PB " + pb.toString());
        List<Clause> clauses = encodePB(pb);
        if (debug >= 1)
            writeComment("# Output PB " + pb.toString());
        for (Clause clause : clauses) {
            writeClause(clause);
        }
    }
}
