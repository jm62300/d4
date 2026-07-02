package pbsugar.encoder;

import java.io.IOException;

public class SimpleCM extends CM {
    private String matrix = null;
    private String[] sumVar = null;
    
    public SimpleCM(int[] as, int[] xs, int m, PBEncoder encoder) {
        super(as, xs, m, encoder);
        name = "SimpleCM";
    }

    @Override
    public int elem0(int i, int j) {
        int code;
        if (sumVar == null) {
            if (j < 1 || j > m)
                throw new IllegalArgumentException("elem " + i + " " + j);
            code = encoder.code(matrix, i, j);
        } else {
            /*
            if (j <= domain[i-1].lb())
                return Encoder.TRUE;
            if (j > domain[i-1].ub())
                return Encoder.FALSE;
            if (j > m)
                return Encoder.UNDEF;
             */
            code = sumCode(sumVar[i-1], domain[i-1], j);
        }
        return code;
    }

    private void encode1() throws IOException {
        matrix = encoder.newAuxVar("z");
        encoder.newVar(matrix, n, m);
        // z(i, j): i = 1..n, j = 1..m                                                                                                                      
        // z(i, j) == 1 <==> a(1)*x(1) + ... + a(i)*x(i) >= j                                                                                                 
        // z(i, j) = 1 (when j <= 0)                                                                                                                          
        // z(0, j) = 0 (when j = 1..m)                                                                                                                      
        if (ORDER_AXIOMS) {
            for (int i = 1; i <= n; i++) {
                for (int j = 1; j <= m; j++) {
                    // (z(i, j) === 1) ==> (z(i, j-1) === 1)
                    if (j-1 >= 1)
                        encoder.writeClause(neg(elem(i, j)), elem(i, j-1));
                }
            }
        }
        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= m; j++) {
                // (z(i-1, j) === 1) ==> (z(i, j) === 1)                                                                                                            
                if (i-1 >= 1)
                    encoder.writeClause(neg(elem(i-1, j)), elem(i, j));
          }
        }
        for (int i = 1; i <= n; i++) {
            int a = as[i-1];
            for (int j = 1; j <= m; j++) {
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
                // (x === 1) ==> (z(i-1, j-a) === 1) ==> (z(i, j) === 1)                                                                                            
                if (i-1 >= 1 && j-a >= 1)
                    encoder.writeClause(neg(x), neg(elem(i-1, j-a)), elem(i, j));
                if (j-a < 1) // TODO                                                                                                                                
                    encoder.writeClause(neg(x), elem(i, j));
            }
        }
    }
    
    private void encode2a() throws IOException {
        matrix = encoder.newAuxVar("z");
        encoder.newVar(matrix, n, m);
        // z(i, j): i = 1..n, j = 1..m                                                                                                                      
        // z(i, j) == 1 <==> a(1)*x(1) + ... + a(i)*x(i) >= j                                                                                                 
        // z(i, j) = 1 (when j <= 0)                                                                                                                          
        // z(0, j) = 0 (when j = 1..m)                                                                                                                      
        if (ORDER_AXIOMS) {
            for (int i = 1; i <= n; i++) {
                int j0 = 0;
                for (int[] r : domain[i - 1].intervals) {
                    for (int j = r[0]; j <= r[1]; j++) {
                        if (j == 0) continue;
                        // (z(i, j) === 1) ==> (z(i, j-1) === 1)
                        if (j0 > 0)
                            encoder.writeClause(neg(elem(i, j)), elem(i, j0));
                        j0 = j;
                    }
                }
            }
        }
        // x(1) || ! s(1) >= a(1)
        encoder.writeClause(xs[0], neg(elem(1, as[0])));
        // ! x(1) || (s(1) >= a(1))
        encoder.writeClause(neg(xs[0]), elem(1, as[0]));
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // s(i-1) >= j+1 || ! s(i) >= j+a(i)+1
                    if (j + as[i-1] + 1 > domain[i-1].ub()) continue;
                    int j2 = domain[i-1].upper(j + as[i-1] + 1);
                    if (j + 1 > domain[i-2].ub()) {
                        encoder.writeClause(neg(elem(i, j2)));
                    } else {
                        int j1 = domain[i-2].upper(j + 1);
                        encoder.writeClause(elem(i-1, j1), neg(elem(i, j2)));
                    }
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // x(i) || s(i-1) >= j+1 || ! s(i) >= j+1
                    if (j + 1 > domain[i-1].ub()) continue;
                    int j2 = domain[i-1].upper(j + 1);
                    if (j + 1 > domain[i-2].ub()) {
                        encoder.writeClause(xs[i-1], neg(elem(i, j2)));
                    } else {
                        int j1 = domain[i-2].upper(j + 1);
                        encoder.writeClause(xs[i-1], elem(i-1, j1), neg(elem(i, j2)));
                    }
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! s(i-1) >= j || s(i) >= j
                    encoder.writeClause(neg(elem(i-1, j)), elem(i, j));
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! x(i) || ! s(i-1) >= j || s(i) >= j+a(i)
                    if (j + as[i-1] > m) continue;
                    encoder.writeClause(neg(xs[i-1]), neg(elem(i-1, j)), elem(i, j+as[i-1]));
                }
            }
        }
    }

    private void encode2b() throws IOException {
        sumVar = new String[n];
        for (int i = 1; i <= n; i++) {
            sumVar[i-1] = encoder.newAuxVar("s");
            encodeSum(sumVar[i-1], domain[i-1]);
        }
        // x(1) || ! s(1) >= a(1)
        encoder.writeClause(xs[0], neg(elem(1, as[0])));
        // ! x(1) || (s(1) >= a(1))
        encoder.writeClause(neg(xs[0]), elem(1, as[0]));
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // s(i-1) >= j+1 || ! s(i) >= j+a(i)+1
                    int lit1 = elem(i-1, j+1);
                    int lit2 = neg(elem(i, j+as[i-1]+1));
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // x(i) || s(i-1) >= j+1 || ! s(i) >= j+1
                    int lit1 = elem(i-1, j+1);
                    int lit2 = neg(elem(i, j+1));
                    encoder.writeClause(xs[i-1], lit1, lit2);
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! s(i-1) >= j || s(i) >= j
                    int lit1 = neg(elem(i-1, j));
                    int lit2 = elem(i, j);
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = 2; i <= n; i++) {
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
    
    private void encode2c() throws IOException {
        sumVar = new String[n];
        for (int i = 1; i <= n; i++) {
            sumVar[i-1] = encoder.newAuxVar("s");
            encodeSum(sumVar[i-1], domain[i-1]);
        }
        // x(1) || ! s(1) >= a(1)
        encoder.writeClause(xs[0], neg(elem(1, as[0])));
        // ! x(1) || (s(1) >= a(1))
        encoder.writeClause(neg(xs[0]), elem(1, as[0]));
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 1].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! (s(i) >= j) || (s(i-1) >= j - a) 
                    int lit1 = neg(elem(i, j));
                    int lit2 = elem(i-1, j-as[i-1]);
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 1].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // x(i) || ! (s(i) >= j) || s(i-1) >= j
                    int lit1 = neg(elem(i, j));
                    int lit2 = elem(i-1, j);
                    encoder.writeClause(xs[i-1], lit1, lit2);
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 1].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // s(i) >= j+1 || ! s(i-1) >= j+1
                    int lit1 = elem(i, j+1);
                    int lit2 = neg(elem(i-1, j+1));
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 1].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! x(i) || s(i) >= j+1 || ! s(i-1) >= j+1-a(i)
                    int lit1 = elem(i, j+1);
                    int lit2 = neg(elem(i-1, j+1-as[i-1]));
                    encoder.writeClause(neg(xs[i-1]), lit1, lit2);
                }
            }
        }
    }
    
    private void encode2() throws IOException {
        sumVar = new String[n];
        for (int i = 1; i <= n; i++) {
            sumVar[i-1] = encoder.newAuxVar("s");
            encodeSum(sumVar[i-1], domain[i-1]);
        }
        // x(1) || ! s(1) >= a(1)
        encoder.writeClause(xs[0], neg(elem(1, as[0])));
        // ! x(1) || (s(1) >= a(1))
        encoder.writeClause(neg(xs[0]), elem(1, as[0]));
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 1].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! (s(i) >= j) || (s(i-1) >= j - a) 
                    int lit1 = neg(elem(i, j));
                    int lit2 = elem(i-1, j-as[i-1]);
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 1].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // x(i) || ! (s(i) >= j) || s(i-1) >= j
                    int lit1 = neg(elem(i, j));
                    int lit2 = elem(i-1, j);
                    encoder.writeClause(xs[i-1], lit1, lit2);
                }
            }
        }
        for (int i = 2; i <= n; i++) {
            for (int[] r : domain[i - 2].intervals) {
                for (int j = r[0]; j <= r[1]; j++) {
                    // ! s(i-1) >= j || s(i) >= j
                    int lit1 = neg(elem(i-1, j));
                    int lit2 = elem(i, j);
                    encoder.writeClause(lit1, lit2);
                }
            }
        }
        for (int i = 2; i <= n; i++) {
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
