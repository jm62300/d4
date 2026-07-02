package pbsugar.sat;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.BitSet;
import java.util.StringTokenizer;

import pbsugar.PBSugar;

public class Solver {
    public String satSolverName;
    public String satFileName;
    public String outFileName;
    public Boolean result = null;
    public BitSet solution = null;

    public Solver(String satSolverName, String satFileName, String outFileName) {
        this.satSolverName = satSolverName;
        this.satFileName = satFileName;
        this.outFileName = outFileName;
    }

    public Solver(String satSolverName, String satFileName) {
        this(satSolverName, satFileName, null);
    }

    private void decode(BufferedReader rd) throws IOException {
        while (true) {
            String line = rd.readLine();
            if (line == null)
                break;
            if (line.startsWith("s SAT")) {
                result = true;
            } else if (line.startsWith("s UNSAT")) {
                result = false;
            } else if (line.startsWith("v ")) {
                StringTokenizer st = new StringTokenizer(line);
                st.nextToken();
                while (st.hasMoreTokens()) {
                    int code = Integer.parseInt(st.nextToken());
                    if (code > 0) {
                        solution.set(code, true);
                    } else {
                        solution.set(-code, false);
                    }
                }
            } else {
                PBSugar.info("c " + line);
            }
        }
    }
    
    private void decodeOut(BufferedReader rd) throws IOException {
        while (true) {
            String line = rd.readLine();
            if (line == null)
                break;
            if (line.startsWith("SAT")) {
                result = true;
            } else if (line.startsWith("UNSAT")) {
                result = false;
            } else {
                StringTokenizer st = new StringTokenizer(line);
                while (st.hasMoreTokens()) {
                    int code = Integer.parseInt(st.nextToken());
                    if (code > 0) {
                        solution.set(code, true);
                    } else {
                        solution.set(-code, false);
                    }
                }
            }
        }
    }
    
    public BitSet solve() throws IOException, InterruptedException {
        String[] command = { satSolverName, satFileName };
        if (outFileName != null)
            command = new String[] { satSolverName, satFileName, outFileName };
        Process process = Runtime.getRuntime().exec(command);
        BufferedReader stdout = new BufferedReader(
                new InputStreamReader(process.getInputStream()));
        BufferedReader stderr = new BufferedReader(
                new InputStreamReader(process.getErrorStream()));
        solution = new BitSet(); 
        decode(stderr);
        stderr.close();
        decode(stdout);
        stdout.close();
        process.waitFor();
        if (outFileName != null) {
            BufferedReader out = new BufferedReader(new FileReader(outFileName));
            decodeOut(out);
            out.close();
        }
        if (result == null || ! result)
            solution = null;
        return solution;
    }
}
