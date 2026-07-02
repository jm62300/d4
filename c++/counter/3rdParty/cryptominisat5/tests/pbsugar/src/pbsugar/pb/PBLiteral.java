package pbsugar.pb;

public class PBLiteral {
    private String x;
    private boolean negative;

    public PBLiteral(String x, boolean negative) {
        this.x = x;
        this.negative = negative;
    }

    public PBLiteral(String x) {
        this(x, false);
    }

    public String getVariable() {
        return x;
    }
    
    public boolean isNegative() {
        return negative;
    }

    public PBLiteral negate() {
        return new PBLiteral(x, ! negative);
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + (negative ? 1231 : 1237);
        result = prime * result + ((x == null) ? 0 : x.hashCode());
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
        PBLiteral other = (PBLiteral) obj;
        if (negative != other.negative)
            return false;
        if (x == null) {
            if (other.x != null)
                return false;
        } else if (!x.equals(other.x))
            return false;
        return true;
    }

    @Override
    public String toString() {
        return negative ? "~" + x : x;
    }
}
