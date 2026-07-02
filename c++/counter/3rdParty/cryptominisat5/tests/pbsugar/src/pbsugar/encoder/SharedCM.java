package pbsugar.encoder;

import java.io.IOException;

public class SharedCM extends CM {
    private CM z0;
    private int n0;
    private int m0;
    private String matrix1 = null;
    private String matrix2 = null;
    private String[] sumVar = null;
    
    public SharedCM(int[] as, int[] xs, int m, CM z0, PBEncoder encoder) {
        super(as, xs, m, encoder);
        this.z0 = z0;
        for (n0 = 0; n0 < n && n0 < z0.n; n0++) {
            if (as[n0] != z0.as[n0] || xs[n0] != z0.xs[n0])
                break;
        }
        m0 = z0.m;
    }

    @Override
    public int elem0(int i, int j) {
        int code;
        if (sumVar == null) {
            if (j < 1 || j > m)
                throw new IllegalArgumentException("elem " + i + " " + j);
            if (i <= n0 && j <= m0)
                code = z0.elem(i, j);
            else if (i <= n0)
                code = encoder.code(matrix1, i, j - m0);
            else
                code = encoder.code(matrix2, i - n0, j);
        } else {
            if (i <= n0)
                code = z0.elem(i, j);
            else
                code = sumCode(sumVar[i-n0-1], domain[i-1], j);
        }
        return code;
    }

    private void encode1() throws IOException {
        if (m0 < m) {
            matrix1 = encoder.newAuxVar("z");
            encoder.newVar(matrix1, n0, m-m0);
        }
        if (n0 < n) {
            matrix2 = encoder.newAuxVar("z");
            encoder.newVar(matrix2, n-n0, m);
        }
        if (ORDER_AXIOMS) {
            for (int i = 1; i <= n; i++) {
                for (int j = 1; j <= m; j++) {
                    if (i <= n0 && j <= m0) continue;
                    // (z(i, j) === 1) ==> (z(i, j-1) === 1)
                    if (j-1 >= 1)
                        encoder.writeClause(neg(elem(i, j)), elem(i, j-1));
                }
            }
        }
        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= m; j++) {
                if (i <= n0 && j <= m0) continue;
                // (z(i-1, j) === 1) ==> (z(i, j) === 1)                                                                                                            
                if (i-1 >= 1)
                    encoder.writeClause(neg(elem(i-1, j)), elem(i, j));
          }
        }
        for (int i = 1; i <= n; i++) {
            int a = as[i-1];
            for (int j = 1; j <= m; j++) {
                if (i <= n0 && j <= m0) continue;
                // (z(i, j) === 1) ==> (z(i-1, j-a) === 1)                                                                                                          
                if (i-1 >= 1 && j-a >= 1)
                    encoder.writeClause(neg(elem(i, j)), elem(i-1, j-a));
                if (i == 1 && j-a == 1)
                    encoder.writeClause(neg(elem(i, j)));
            }
        }
        for (int i = 1; i <= n; i++) {
            int x = xs[i-1];
            for (int j = 1; j <= m; j++) {
                if (i <= n0 && j <= m0) continue;
                // (x === 0) ==> (z(i, j) === 1) ==> (z(i-1, j) === 1)                                                                                              
                if (i-1 >= 1)
                    encoder.writeClause(x, neg(elem(i, j)), elem(i-1, j));
                if (i == 1 && j == 1)
                    encoder.writeClause(x, neg(elem(i, j)));
            }
        }
        for (int i = 1; i <= n; i++) {
            int a = as[i-1];
            int x = xs[i-1];
            for (int j = 1; j <= m; j++) {
                if (i <= n0 && j <= m0) continue;
                // (x === 1) ==> (z(i-1, j-a) === 1) ==> (z(i, j) === 1)                                                                                            
                if (i-1 >= 1 && j-a >= 1)
                    encoder.writeClause(neg(x), neg(elem(i-1, j-a)), elem(i, j));
                if (j-a < 1) // TODO                                                                                                                                
                    encoder.writeClause(neg(x), elem(i, j));
            }
        }
    }

    private void encode2b() throws IOException {
        if (m0 < m) {
            throw new IllegalArgumentException("Can not be shared");
        }
        if (n0 < n) {
            sumVar = new String[n - n0];
            for (int i = n0 + 1; i <= n; i++) {
                sumVar[i-n0-1] = encoder.newAuxVar("s");
                encodeSum(sumVar[i-n0-1], domain[i-1]);
            }
        }
        // x(1) || ! s(1) >= a(1)
        // encoder.writeClause(xs[0], neg(elem(1, as[0])));
        // ! x(1) || (s(1) >= a(1))
        // encoder.writeClause(neg(xs[0]), elem(1, as[0]));
        for (int i = n0+1; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // s(i-1) >= j+1 || ! s(i) >= j+a(i)+1
                    int lit1 = elem(i-1, j+1);
                    int lit2 = neg(elem(i, j+as[i-1]+1));
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = n0+1; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // x(i) || s(i-1) >= j+1 || ! s(i) >= j+1
                    int lit1 = elem(i-1, j+1);
                    int lit2 = neg(elem(i, j+1));
                    encoder.writeClause(xs[i-1], lit1, lit2);
                }
            }
        }
        for (int i = n0+1; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! s(i-1) >= j || s(i) >= j
                    int lit1 = neg(elem(i-1, j));
                    int lit2 = elem(i, j);
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = n0+1; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! x(i) || ! s(i-1) >= j || s(i) >= j+a(i)
                    int lit1 = neg(elem(i-1, j));
                    int lit2 = elem(i, j+as[i-1]);
                    encoder.writeClause(neg(xs[i-1]), lit1, lit2);
                }
            }
        }
    }
    
    private void encode2() throws IOException {
        if (m0 < m) {
            throw new IllegalArgumentException("Can not be shared");
        }
        if (n0 < n) {
            sumVar = new String[n - n0];
            for (int i = n0 + 1; i <= n; i++) {
                sumVar[i-n0-1] = encoder.newAuxVar("s");
                encodeSum(sumVar[i-n0-1], domain[i-1]);
            }
        }
        // x(1) || ! s(1) >= a(1)
        // encoder.writeClause(xs[0], neg(elem(1, as[0])));
        // ! x(1) || (s(1) >= a(1))
        // encoder.writeClause(neg(xs[0]), elem(1, as[0]));
        for (int i = n0+1; i <= n; i++) {
            for (int[] r : domain[i - 1].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! (s(i) >= j) || (s(i-1) >= j - a) 
                    int lit1 = neg(elem(i, j));
                    int lit2 = elem(i-1, j-as[i-1]);
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = n0+1; i <= n; i++) {
            for (int[] r : domain[i - 1].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // x(i) || ! (s(i) >= j) || s(i-1) >= j
                    int lit1 = neg(elem(i, j));
                    int lit2 = elem(i-1, j);
                    encoder.writeClause(xs[i-1], lit1, lit2);
                }
            }
        }
        for (int i = n0+1; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! s(i-1) >= j || s(i) >= j
                    int lit1 = neg(elem(i-1, j));
                    int lit2 = elem(i, j);
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = n0+1; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! x(i) || ! s(i-1) >= j || s(i) >= j+a(i)
                    int lit1 = neg(elem(i-1, j));
                    int lit2 = elem(i, Math.min(j+as[i-1], m));
                    encoder.writeClause(neg(xs[i-1]), lit1, lit2);
                }
            }
        }
    }

    @Override
    public void encode() throws IOException {
        if (PBEncoder.USE_SPARSE_CM) {
            encode2();
        } else {
            encode1();
        }
    }
}
