package pbsugar.pb;

import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

public class PBConstraint {
    public static int SORT_COEF = 3;
    public static String LE = "<=";
    public static String GE = ">=";
    public static String EQ = "=";
    public static String NE = "!=";
    // public static PBConstraint FALSE_PB = new PBConstraint(null, null, GE, BigInteger.ONE);
    // public static PBConstraint TRUE_PB = new PBConstraint(null, null, GE, BigInteger.ZERO);
    
    private List<BigInteger> as;
    private List<PBLiteral> xs;
    private String cmp;
    private BigInteger b;
    
    public PBConstraint(List<BigInteger> as, List<PBLiteral> xs, String cmp, BigInteger b) {
        assert(as == null || as.size() == xs.size());
        assert(cmp.equals(LE) || cmp.equals(GE) || cmp.equals(EQ) || cmp.equals(NE));
        if (as == null) {
            as = new ArrayList<BigInteger>();
            for (int i = 0; i < xs.size(); i++)
                as.add(BigInteger.ONE);
        }            
        this.as = as;
        this.xs = xs;
        this.cmp = cmp;
        this.b = b;
    }
    
    public void normalize() {
        normalizeVariables();
        normalizeNegative();
        normalizeGCD();
        normalizeCmp();
        sortByCoef();
    }

    /**
     * Multiple occurrences of variables are replaced with a single occurrence.
     * +2*x -3*x ==> -1*x 
     */
    private void normalizeVariables() {
        List<BigInteger> as = new ArrayList<BigInteger>();
        List<PBLiteral> xs = new ArrayList<PBLiteral>();
        BigInteger b = this.b;
        int n = size();
        for (int i = 0; i < n; i++) {
            BigInteger a = this.as.get(i);
            PBLiteral x = this.xs.get(i);
            if (x.isNegative()) {
                a = a.negate();
                x = x.negate();
                b = b.add(a);
            }
            boolean found = false;
            for (int j = 0; j < xs.size(); j++) {
                if (x.equals(xs.get(j))) {
                    found = true;
                    as.set(j, as.get(j).add(a));
                }
            }
            if (! found) {
                as.add(a);
                xs.add(x);
            }
        }
        this.as = as;
        this.xs = xs;
        this.b = b;
    }

    /**
     * Negative coefficients are replaced with positive coefficients.
     * -2*x -3*y <= -2 ==> +2*!x +3*!y <= 3 
     */
    private void normalizeNegative() {
        List<BigInteger> as = new ArrayList<BigInteger>();
        List<PBLiteral> xs = new ArrayList<PBLiteral>();
        BigInteger b = this.b;
        int n = size();
        for (int i = 0; i < n; i++) {
            if (this.as.get(i).signum() < 0) { 
                as.add(this.as.get(i).negate());
                xs.add(this.xs.get(i).negate());
                b = b.subtract(this.as.get(i));
            } else if (this.as.get(i).signum() > 0) {
                as.add(this.as.get(i));
                xs.add(this.xs.get(i));
            }
        }
        this.as = as;
        this.xs = xs;
        this.b = b;
    }

    /**
     * Coefficients are divided by GCD.
     */
    private void normalizeGCD() {
        if (isValid() || isUnsat())
            return;
        if (b.signum() == 0)
            return;
        int n = size();
        assert(n > 0);
        assert(b.signum() > 0);
        BigInteger gcd = this.as.get(0);
        for (int i = 1; i < n; i++) {
            gcd = this.as.get(i).gcd(gcd);
            if (gcd.equals(BigInteger.ONE))
                return;
        }
        if (gcd.equals(BigInteger.ONE))
            return;
        if (cmp.equals(LE)) {
            for (int i = 0; i < n; i++) {
                as.set(i, this.as.get(i).divide(gcd));
            }
            b = b.divide(gcd);
        } else if (cmp.equals(GE)) {
            for (int i = 0; i < n; i++) {
                as.set(i, this.as.get(i).divide(gcd));
            }
            b = b.add(gcd).subtract(BigInteger.ONE).divide(gcd);
        } else if (cmp.equals(EQ)) {
            BigInteger gcd1 = b.abs().gcd(gcd);
            if (gcd1.equals(BigInteger.ONE)) {
                // False
                as = Collections.emptyList();
                xs = Collections.emptyList();
                b = BigInteger.ONE;
            } else {
                for (int i = 0; i < n; i++) {
                    as.set(i, this.as.get(i).divide(gcd1));
                }
                b = b.divide(gcd1);
            }
        } else { // cmp.equals(NE)
            BigInteger gcd1 = b.abs().gcd(gcd);
            if (gcd1.equals(BigInteger.ONE)) {
                // True
                as = Collections.emptyList();
                xs = Collections.emptyList();
                b = BigInteger.ONE;
            } else {
                for (int i = 0; i < n; i++) {
                    as.set(i, this.as.get(i).divide(gcd1));
                }
                b = b.divide(gcd1);
            }
        }
    }

    /**
     * Choosing smaller RHS.
     */
    private void normalizeCmp() {
        if (isValid() || isUnsat())
            return;
        BigInteger b1 = ubLHS().subtract(b);
        int n = size();
        if (b1.compareTo(b) < 0) {
            for (int i = 0; i < n; i++) {
                xs.set(i, this.xs.get(i).negate());
            }
            b = b1;
            if (cmp.equals(LE)) {
                cmp = GE;
            } else if (cmp.equals(GE)) {
                cmp = LE;
            }
        }
    }
    
    private void sortByCoef() {
        int n = size();
        if (SORT_COEF > 0 && n > 0) {
            Object[][] axs = new Object[n][];
            for (int i = 0; i < n; i++) {
                axs[i] = new Object[] { as.get(i), xs.get(i) };
            }
            if (SORT_COEF == 1) {
                // sort by literal name
                Arrays.sort(axs, new Comparator<Object[]>() {
                    @Override
                    public int compare(Object[] arg0, Object[] arg1) {
                        String x0 = ((PBLiteral)arg0[1]).toString();
                        String x1 = ((PBLiteral)arg1[1]).toString();
                        return x0.compareTo(x1);
                    }
                });
            } else if (SORT_COEF == 2) {
                // sort by coefficient in ascending order
                Arrays.sort(axs, new Comparator<Object[]>() {
                    @Override
                    public int compare(Object[] arg0, Object[] arg1) {
                        BigInteger a0 = (BigInteger)arg0[0];
                        BigInteger a1 = (BigInteger)arg1[0];
                        int c = a0.compareTo(a1);
                        String x0 = ((PBLiteral)arg0[1]).toString();
                        String x1 = ((PBLiteral)arg1[1]).toString();
                        return c != 0 ? c : x0.compareTo(x1);
                    }
                });
            } else if (SORT_COEF == 3) {
                // sort by coefficient in descending order
                Arrays.sort(axs, new Comparator<Object[]>() {
                    @Override
                    public int compare(Object[] arg0, Object[] arg1) {
                        BigInteger a0 = (BigInteger)arg0[0];
                        BigInteger a1 = (BigInteger)arg1[0];
                        int c = a0.compareTo(a1);
                        String x0 = ((PBLiteral)arg0[1]).toString();
                        String x1 = ((PBLiteral)arg1[1]).toString();
                        return c != 0 ? - c : x0.compareTo(x1);
                    }
                });
            }
            List<BigInteger> as = new ArrayList<BigInteger>(n);
            List<PBLiteral> xs = new ArrayList<PBLiteral>(n);
            for (int i = 0; i < n; i++) {
                as.add((BigInteger)axs[i][0]);
                xs.add((PBLiteral)axs[i][1]);
            }
            this.as = as;
            this.xs = xs;
        }
    }
    
    public int size() {
        return as.size();
    }
    
    public List<BigInteger> as() {
        return as;
    }

    public List<PBLiteral> xs() {
        return xs;
    }

    public BigInteger a(int i) {
        assert(i >= 1 && i <= size());
        return as.get(i-1);
    }

    public PBLiteral x(int i) {
        assert(i >= 1 && i <= size());
        return xs.get(i-1);
    }

    public String cmp() {
        return cmp;
    }

    public BigInteger b() {
        return b;
    }

    public BigInteger ubLHS() {
        BigInteger s = BigInteger.ZERO;
        for (BigInteger a : as) {
            s = s.add(a);
        }
        return s;
    }

    private boolean isValidUnder(BigInteger lb, BigInteger ub) {
        if (cmp.equals(LE)) {
            return ub.compareTo(b) <= 0;
        } else if (cmp.equals(GE)) {
            return lb.compareTo(b) >= 0;
        } else if (cmp.equals(EQ)){
            return ub.compareTo(b) <= 0 && lb.compareTo(b) >= 0;
        } else { // cmp.equals(NE)
            return lb.compareTo(b) > 0 || ub.compareTo(b) < 0;
        }
    }
    
    public boolean isValid() {
        return isValidUnder(BigInteger.ZERO, ubLHS());
    }
    
    public boolean isValidWhen(int i, int value) {
        assert(value == 0 || value == 1);
        BigInteger lb = BigInteger.ZERO;
        BigInteger ub = ubLHS();
        if (value == 0) {
            ub = ub.subtract(a(i));
        } else {
            lb = lb.add(a(i));
        }
        return isValidUnder(lb, ub);
    }
    
    private boolean isUnsatUnder(BigInteger lb, BigInteger ub) {
        if (cmp.equals(LE)) {
            return lb.compareTo(b) > 0;
        } else if (cmp.equals(GE)) {
            return ub.compareTo(b) < 0;
        } else if (cmp.equals(EQ)){
            return lb.compareTo(b) > 0 || ub.compareTo(b) < 0; 
        } else { // cmp.equals(NE)
            return ub.compareTo(b) <= 0 && lb.compareTo(b) >= 0;
        }
    }
    
    public boolean isUnsat() {
        return isUnsatUnder(BigInteger.ZERO, ubLHS());
    }

    public boolean isUnsatWhen(int i, int value) {
        assert(value == 0 || value == 1);
        BigInteger lb = BigInteger.ZERO;
        BigInteger ub = ubLHS();
        if (value == 0) {
            ub = ub.subtract(a(i));
        } else {
            lb = lb.add(a(i));
        }
        return isUnsatUnder(lb, ub);
    }
    
    public BigInteger lhs(Set<String> pbSolution) {
        BigInteger lhs = BigInteger.ZERO;
        int n = size();
        for (int i = 0; i < n; i++) {
            PBLiteral x = xs.get(i); 
            if (pbSolution.contains(x.getVariable()) ^ x.isNegative()) {
                lhs = lhs.add(as.get(i));
            }
        }
        return lhs;
    }
    
    public boolean isSatisfied(Set<String> pbSolution) {
        BigInteger lhs = lhs(pbSolution);
        if (cmp.equals(LE)) {
            return lhs.compareTo(b) <= 0;
        } else if (cmp.equals(GE)) {
            return lhs.compareTo(b) >= 0;
        } else if (cmp.equals(EQ)) {
            return lhs.compareTo(b) == 0;
        } else { // cmp.equals(NE)
            return lhs.compareTo(b) != 0;
        }
    }
    
    public PBConstraint[] divideAndRemainder(BigInteger p) {
        List<BigInteger> as0 = new ArrayList<BigInteger>();
        List<PBLiteral> xs0 = new ArrayList<PBLiteral>();
        List<BigInteger> as1 = new ArrayList<BigInteger>();
        List<PBLiteral> xs1 = new ArrayList<PBLiteral>();
        int n = size();
        for (int i = 0; i < n; i++) {
            BigInteger[] dr = as.get(i).divideAndRemainder(p);
            if (dr[0].signum() > 0) {
                as0.add(dr[0]);
                xs0.add(xs.get(i));
            }
            if (dr[1].signum() > 0) {
                as1.add(dr[1]);
                xs1.add(xs.get(i));
            }
        }
        BigInteger[] dr = b.divideAndRemainder(p);
        BigInteger b0 = dr[0];
        BigInteger b1 = dr[1];
        PBConstraint pb0 = new PBConstraint(as0, xs0, cmp, b0);
        PBConstraint pb1 = new PBConstraint(as1, xs1, cmp, b1);
        return new PBConstraint[] { pb0, pb1 };
    }

    public PBConstraint addXs(List<String> xs1) {
        List<BigInteger> as = new ArrayList<BigInteger>(this.as);
        List<PBLiteral> xs = new ArrayList<PBLiteral>(this.xs);
        for (int i = 0; i < xs1.size(); i++) {
            as.add(BigInteger.ONE);
            xs.add(new PBLiteral(xs1.get(i)));
        }
        PBConstraint pb = new PBConstraint(as, xs, cmp, b);
        return pb;
    }
    
    
    public PBConstraint addB(BigInteger b1) {
        PBConstraint pb = new PBConstraint(as, xs, cmp, b.add(b1));
        return pb;
    }
    
    public String toString() {
        StringBuilder sb = new StringBuilder();
        int n = size();
        for (int i = 0; i < n; i++) {
            if (as.get(i).signum() >= 0)
                sb.append("+");
            sb.append(as.get(i).toString());
            sb.append(" ");
            sb.append(xs.get(i).toString());
            sb.append(" ");
        }
        sb.append(cmp + " ");
        sb.append(b.toString());
        return sb.toString();
    }

}
