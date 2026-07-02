package pbsugar.encoder;

import java.util.Arrays;

public class CMkey {
    private int[] as;
    private int[] xs;
    
    public CMkey(int[] as, int[] xs) {
        this.as = as;
        this.xs = xs;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + Arrays.hashCode(as);
        result = prime * result + Arrays.hashCode(xs);
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        CMkey other = (CMkey) obj;
        if (!Arrays.equals(as, other.as))
            return false;
        if (!Arrays.equals(xs, other.xs))
            return false;
        return true;
    }
}
