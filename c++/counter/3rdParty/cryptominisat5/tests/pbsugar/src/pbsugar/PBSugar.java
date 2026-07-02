package pbsugar;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedSet;
import java.util.StringTokenizer;
import java.util.TreeSet;
import java.util.regex.Pattern;

import pbsugar.encoder.CM;
import pbsugar.encoder.PBEncoder;
import pbsugar.pb.PBConstraint;
import pbsugar.pb.PBLiteral;
import pbsugar.pb.PBParser;
import pbsugar.sat.Solver;

public class PBSugar {
    public static int verboseLevel = 0;
    private static final long MEGA = 1024*1024;
    
    public static void info(String message) {
        if (verboseLevel >= 1) {
            System.out.println("c " + message);
        }
    }
    
    public static void fine(String message) {
        if (verboseLevel >= 2) {
            System.out.println("c " + message);
        }
    }

    public static void status() {
        MemoryMXBean mbean = ManagementFactory.getMemoryMXBean();
        MemoryUsage heapUsage = mbean.getHeapMemoryUsage();
//      long heapInit = heapUsage.getInit() / MEGA;
        long heapUsed = heapUsage.getUsed() / MEGA;
//      long heapCommitted = heapUsage.getCommitted() / MEGA;
        long heapMax = heapUsage.getMax() / MEGA;
        MemoryUsage nonHeapUsage = mbean.getNonHeapMemoryUsage();
//      long nonHeapInit = nonHeapUsage.getInit() / MEGA;
        long nonHeapUsed = nonHeapUsage.getUsed() / MEGA;
//      long nonHeapCommitted = nonHeapUsage.getCommitted() / MEGA;
        long nonHeapMax = nonHeapUsage.getMax() / MEGA;
        fine(
                "Heap : " + heapUsed + " MiB used (max " + heapMax + " MiB), " +
                "NonHeap : " + nonHeapUsed + " MiB used (max " + nonHeapMax + " MiB)"
        );
    }

    public String pbFileName = null;
    public SortedSet<String> varsSet = null;
    public PBEncoder encoder = null;
    public String satSolverName = null;
    public boolean miniSat = false;
    public String satFileName = null;
    public String mapFileName = null;
    public String outFileName = null;
    public boolean verify = false;
    public int debug = 0;
    
    public int encode() throws IOException {
        if (satFileName == null) {
            File satFile = File.createTempFile("pbsugar", ".cnf");
            satFile.deleteOnExit();
            satFileName = satFile.getAbsolutePath();
        }
        PBSugar.info("Parsing and Encoding");
        encoder = new PBEncoder(satFileName);
        encoder.debug = debug;
        encoder.open();
        varsSet = new TreeSet<String>(new Comparator<String>() {
            Pattern p = Pattern.compile("x\\d+");

            @Override
            public int compare(String s0, String s1) {
                if (p.matcher(s0).matches() && p.matcher(s1).matches()) {
                    int c0 = Integer.parseInt(s0.substring(1));
                    int c1 = Integer.parseInt(s1.substring(1));
                    return c0 < c1 ? -1 : (c0 == c1 ? 0 : 1);
                }
                return s0.compareTo(s1);
            }
        });
        BufferedReader reader = new BufferedReader(new FileReader(pbFileName));
        PBParser parser = new PBParser(reader);
        int count = 0;
        while (true) {
            PBConstraint pb = parser.getNext();
            if (pb == null)
                break;
            count++;
            for (PBLiteral lit : pb.xs()) {
                String name = lit.getVariable();
                if (! varsSet.contains(name)) {
                    varsSet.add(name);
                    encoder.newVar(name);
                }
            }
            encoder.encode(pb);
            if (count % 100000 == 0) {
                PBSugar.info("Parsing and Encoding " + count + " constraints " + varsSet.size() + " variables");        
            }
        }
        PBSugar.info("Parsed and Encoded " + count + " constraints " + varsSet.size() + " variables");        
        reader.close();
        encoder.close();
        PBSugar.info("CM count " + PBEncoder.countCM);
        PBSugar.info("CM reused " + PBEncoder.countCMreused);
        PBSugar.info("CM shared " + PBEncoder.countCMshared);
        PBSugar.info("PB decomposed " + PBEncoder.countDecompose);
        PBSugar.info("Encoded to SAT with " +
                encoder.sat.variablesCount + " variables " +
                encoder.sat.clausesCount + " clauses");
        return parser.variables;
    }

    public void saveMap(int variables) throws IOException {
        BufferedWriter writer = new BufferedWriter(new FileWriter(mapFileName));
        writer.write(Integer.toString(variables) + "\n");
        for (String v : varsSet) {
            int code = encoder.code(v);
            writer.write(v + " " + code + "\n");
        }
        writer.close();
    }

    public Set<String> solve(int variables) throws IOException, InterruptedException {
        PBSugar.info("Solving by " + satSolverName);
        if (miniSat && outFileName == null) {
            File outFile = File.createTempFile("pbsugar", ".out");
            outFile.deleteOnExit();
            outFileName = outFile.getAbsolutePath();
        }
        Solver solver = new Solver(satSolverName, satFileName, outFileName);
        BitSet satSolution = solver.solve();
        Boolean satResult = solver.result;
        Set<String> pbSolution = null;
        if (satResult != null && satResult) {
            pbSolution = new HashSet<String>();
            for (String v : varsSet) {
                int code = encoder.code(v);
                if (satSolution.get(code))
                    pbSolution.add(v);
            }
        }
        if (satResult == null) {
            System.out.println("s UNKNOWN");
        } else if (satResult) {
            System.out.println("s SATISFIABLE");
            System.out.print("v");
            for (String v : varsSet) {
                if (pbSolution.contains(v))
                    System.out.print(" " + v);
                else
                    System.out.print(" -" + v);
            }
            System.out.println();
        } else {
            System.out.println("s UNSATISFIABLE");
        }
        PBSugar.info("Finished");
        return pbSolution;
    }
    
    private Boolean satSolution(String outFileName, BitSet satSolution) throws IOException {
        Boolean satResult = null;
        BufferedReader reader = new BufferedReader(
                new InputStreamReader(new FileInputStream(outFileName), "UTF-8"));
        String line = reader.readLine();
        if (line == null) {
        } else if (line.startsWith("SAT")) {
            satResult = true;
            while (true) {
                line = reader.readLine();
                if (line == null)
                    break;
                StringTokenizer st = new StringTokenizer(line);
                while (st.hasMoreTokens()) {
                    int code = Integer.parseInt(st.nextToken());
                    if (code > 0) {
                        satSolution.set(code, true);
                    } else {
                        satSolution.set(-code, false);
                    }
                }
            }
        } else if (line.startsWith("UNSAT")) {
            satResult = false;
        } else {
            while (line != null) {
                if (line.startsWith("s SAT")) {
                    satResult = true;
                } else if (line.startsWith("s UNSAT")) {
                    satResult = false;
                } else if (line.startsWith("v ")) {
                    StringTokenizer st = new StringTokenizer(line);
                    st.nextToken();
                    while (st.hasMoreTokens()) {
                        int code = Integer.parseInt(st.nextToken());
                        if (code > 0) {
                            satSolution.set(code, true);
                        } else {
                            satSolution.set(-code, false);
                        }
                    }
                }
                line = reader.readLine();
            }
        }
        reader.close();
        return satResult;
    }

    public Set<String> decode() throws IOException {
        PBSugar.info("Decoding");
        Set<String> pbSolution = null;
        List<String> vars = new ArrayList<String>();
        Map<String,Integer> varsMap = new HashMap<String,Integer>();
        BufferedReader rd = new BufferedReader(
                new InputStreamReader(new FileInputStream(mapFileName), "UTF-8"));
        int variables = Integer.parseInt(rd.readLine());
        while (true) {
            String line = rd.readLine();
            if (line == null)
                break;
            String[] s = line.split("\\s+");
            if (s.length == 2) {
                String v = s[0];
                int code = Integer.parseInt(s[1]);
                vars.add(v);
                varsMap.put(v, code);
            }
        }
        rd.close();
        BitSet satSolution = new BitSet();
        Boolean satResult = satSolution(outFileName, satSolution);
        if (satResult != null && satResult) {
            pbSolution = new HashSet<String>();
            for (String v : vars) {
                int code = varsMap.get(v);
                if (satSolution.get(code))
                    pbSolution.add(v);
            }
        }
        if (satResult == null) {
            System.out.println("s UNKNOWN");
        } else if (satResult) {
            System.out.println("s SATISFIABLE");
            System.out.print("v");
            int count = 0;
            for (String v : vars) {
                if (pbSolution.contains(v))
                    System.out.print(" " + v);
                else
                    System.out.print(" -" + v);
                count++;
                if (count % 64 == 0)
                    System.out.print("\nv");
            }
            System.out.println();
        } else {
            System.out.println("s UNSATISFIABLE");
            pbSolution = null;
        }
        PBSugar.info("Finished");
        return pbSolution;
    }
    
    public void verify(Set<String> pbSolution) throws IOException {
        if (pbSolution == null)
            return;
        PBSugar.info("Verifying");
        int violation = 0;
        BufferedReader reader = new BufferedReader(new FileReader(pbFileName));
        PBParser parser = new PBParser(reader);
        while (true) {
            PBConstraint pb = parser.getNext();
            if (pb == null)
                break;
            if (! pb.isSatisfied(pbSolution)) {
                violation++;
                PBSugar.info("ERROR Violation LHS=" + pb.lhs(pbSolution) + " at " + pb);
            }
        }
        reader.close();
        if (violation == 0)
            PBSugar.info("Verified");
        else
            PBSugar.info("ERROR " + violation + " violations");
    }
    
    public void verifyPbOutput(String pbFileName, String pbOutFileName) throws IOException {
        PBSugar.info("Verifying");
        Boolean result = null;
        Set<String> pbSolution = new HashSet<String>();
        BufferedReader rd1 = new BufferedReader(new FileReader(pbOutFileName));
        while (true) {
            String line = rd1.readLine();
            if (line == null)
                break;
            if (line.startsWith("s SAT")) {
                System.out.println(line);
                result = true;
            } else if (line.startsWith("s UNSAT")) {
                System.out.println(line);
                result = false;
            } else if (line.startsWith("v ")) {
                StringTokenizer st = new StringTokenizer(line);
                st.nextToken();
                while (st.hasMoreTokens()) {
                    String s = st.nextToken();
                    if (! s.startsWith("-"))
                        pbSolution.add(s);
                }
            }
        }
        rd1.close();
        if (result == null) {
            System.out.println("s UNKNOWN");
            System.out.println("s NOT VERIFIED");
            return;
        } else if (! result) {
            System.out.println("s NOT VERIFIED");
            return;
        }
        BufferedReader rd2 = new BufferedReader(new FileReader(pbFileName));
        PBParser parser = new PBParser(rd2);
        int violation = 0;
        while (true) {
            PBConstraint pb = parser.getNext();
            if (pb == null)
                break;
            if (! pb.isSatisfied(pbSolution)) {
                violation++;
                PBSugar.info("Found violation LHS=" + pb.lhs(pbSolution) + " at " + pb);
            }
        }
        rd2.close();
        if (violation == 0) {
            System.out.println("s VERIFIED");
        } else {
            System.out.println("s NOT VERIFIED");
            PBSugar.info("ERROR " + violation + " violations");
        }
    }

    private boolean setOption(String opt) {
        if (opt.matches("(no_)?verify")) {
            verify = ! opt.startsWith("no_");
        } else if (opt.matches("s1=(.+)")) {
            int n = opt.indexOf('=') + 1;
            satSolverName = opt.substring(n);
            miniSat = false;
        } else if (opt.matches("s2=(.+)")) {
            int n = opt.indexOf('=') + 1;
            satSolverName = opt.substring(n);
            miniSat = true;
        } else if (opt.matches("(no_)?axioms")) {
            CM.ORDER_AXIOMS = ! opt.startsWith("no_");
        } else if (opt.matches("sort=(n(o)?|l(it)?|a(sc)?|d(es)?)")) {
            if (opt.startsWith("sort=n"))
                PBConstraint.SORT_COEF = 0;
            else if (opt.startsWith("sort=l"))
                PBConstraint.SORT_COEF = 1;
            else if (opt.startsWith("sort=a"))
                PBConstraint.SORT_COEF = 2;
            else if (opt.startsWith("sort=d"))
                PBConstraint.SORT_COEF = 3;
        } else if (opt.matches("(no_)?sum(_carries)?")) {
            PBEncoder.SUM_CARRIES = ! opt.startsWith("no_");
        } else if (opt.matches("(no_)?clause")) {
            PBEncoder.ENCODE_CLAUSE_PART = ! opt.startsWith("no_");
        } else if (opt.matches("(no_)?cache")) {
            PBEncoder.USE_CM_CACHE = ! opt.startsWith("no_");
        } else if (opt.matches("share=(\\d+)")) {
            int n = opt.indexOf('=') + 1;
            PBEncoder.SHARABLE_CM_LENGTH = Integer.parseInt(opt.substring(n));
        } else if (opt.matches("decomp=(\\d+)")) {
            int n = opt.indexOf('=') + 1;
            PBEncoder.DECOMPOSE_PB = Integer.parseInt(opt.substring(n));
        } else if (opt.matches("cnf=(\\d+)")) {
            int n = opt.indexOf('=') + 1;
            PBEncoder.ENCODE_AS_CNF_LITERALS_SIZE = Integer.parseInt(opt.substring(n));
        } else if (opt.matches("(no_)?sparse")) {
            PBEncoder.USE_SPARSE_CM = ! opt.startsWith("no_");
        } else if (opt.matches("(no_)?dense")) {
            PBEncoder.USE_SPARSE_CM = opt.startsWith("no_");
        } else if (opt.matches("base=(\\d+)")) {
            // future work
            int n = opt.indexOf('=') + 1;
            PBEncoder.BASE = new BigInteger(opt.substring(n));
         } else {
            return false;
        }
        return true;
    }
    
    private void setDefaultOptions() {
        verify = false;
        satSolverName = "minisat";
        miniSat = true;
        CM.ORDER_AXIOMS = true;
        PBConstraint.SORT_COEF = 3;
        PBEncoder.ENCODE_CLAUSE_PART = true;
        PBEncoder.USE_CM_CACHE = true;
        PBEncoder.SHARABLE_CM_LENGTH = 4;
        PBEncoder.DECOMPOSE_PB = 100;
        PBEncoder.ENCODE_AS_CNF_LITERALS_SIZE = 3;
        PBEncoder.USE_SPARSE_CM = true;
        // PBEncoder.BASE = new BigInteger("120");
        PBEncoder.BASE = null;
        PBEncoder.SUM_CARRIES = false;
    }
    
    public static void main(String[] args) {
        try {
            PBSugar pbSugar = new PBSugar();
            pbSugar.setDefaultOptions();
            int i = 0;
            while (i < args.length) {
                if (args[i].equals("-s1") && i + 1 < args.length) {
                    pbSugar.satSolverName = args[i+1];
                    pbSugar.miniSat = false;
                    i++;
                } else if (args[i].equals("-s2") && i + 1 < args.length) {
                    pbSugar.satSolverName = args[i+1];
                    pbSugar.miniSat = true;
                    i++;
                } else if (args[i].equals("-debug") && i + 1 < args.length) {
                    pbSugar.debug = Integer.parseInt(args[i+1]);
                    i++;
                } else if (args[i].equals("-v") || args[i].equals("-verbose")) {
                    PBSugar.verboseLevel++;
                } else if (args[i].equals("-option") && i + 1 < args.length) {
                    String[] opts = args[i+1].split(",");
                    for (String opt : opts) {
                        if (! pbSugar.setOption(opt)) {
                            System.out.println("Unknown option");
                            System.exit(1);
                        }
                    }
                    i++;
                } else {
                    break;
                }
                i++;
            }
            if (i+1 == args.length) {
                pbSugar.pbFileName = args[i];
                int variables = pbSugar.encode();
                Set<String> pbSolution = pbSugar.solve(variables);
                if (pbSugar.verify)
                    pbSugar.verify(pbSolution);
            } else if (i+4 == args.length && args[i].equals("-encode")) {
                pbSugar.pbFileName = args[i+1];
                pbSugar.satFileName = args[i+2];
                pbSugar.mapFileName = args[i+3];
                int variables = pbSugar.encode();
                pbSugar.saveMap(variables);
            } else if (i+4 == args.length && args[i].equals("-decode")) {
                pbSugar.pbFileName = args[i+1];
                pbSugar.outFileName = args[i+2];
                pbSugar.mapFileName = args[i+3];
                Set<String> pbSolution = pbSugar.decode();
                if (pbSugar.verify)
                    pbSugar.verify(pbSolution);
            } else if (i+3 == args.length && args[i].equals("-verify")) {
                String pbFileName = args[i+1];
                String pbOutFileName = args[i+2];
                pbSugar.verifyPbOutput(pbFileName, pbOutFileName);
            } else {
                System.out.println("Unknown command line");
                System.exit(1);
            }
            status();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
