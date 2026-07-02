package pbsugar.encoder;

import java.util.Comparator;
import java.util.Iterator;
import java.util.TreeSet;

public class Domain {
    TreeSet<int[]> intervals;
    int limit = Integer.MAX_VALUE;
    private int size = -1;

    private Domain() {
        intervals = new TreeSet<int[]>(new Comparator<int[]>() {
            public int compare(int[] r1, int[] r2) {
                if (r1[1] != r2[1])
                    return r1[1] < r2[1] ? -1 : 1;
                if (r1[0] != r2[0])
                    return r1[0] < r2[0] ? -1 : 1;
                return 0;
            }
        });
    }
    
    public Domain(int value) {
        this();
        if (value <= limit)
            intervals.add(new int[] { value, value });
    }

    public int lb() {
        return intervals.first()[0];
    }
    
    public int ub() {
        return intervals.last()[1];
    }

    public int size() {
        if (size < 0) {
            size = 0;
            for (int[] r : intervals)
                size += r[1] - r[0] + 1;
        }
        return size;
    }
    
    public boolean contains(int value) {
        int[] r = intervals.higher(new int[] { Integer.MIN_VALUE, value });
        return r != null && r[0] <= value;
    }
    
    public int upper(int value) {
        int[] r = intervals.higher(new int[] { Integer.MIN_VALUE, value });
        if (r == null)
            return value;
        return r[0] <= value ? value : r[0];
    }

    public Domain limit(int limit) {
        if (limit >= ub())
            return this;
        // limit = upper(limit);
        Domain domain = new Domain();
        domain.limit = limit;
        for (int[] r : intervals) {
            if (limit < r[0]) {
                break;
            } else if (limit >= r[1]) {
                domain.intervals.add(r);
            } else { // r[0] <= limit < r[1]
                domain.intervals.add(new int[] { r[0], limit });
                break;
            }
        }
        return domain;
    }
    
    public Domain plus(int value) {
        Domain domain = new Domain();
        for (int[] r : intervals) {
            int l = r[0] + value;
            int u = r[1] + value;
            if (limit < l)
                break;
            if (limit < u)
                u = limit;
            domain.intervals.add(new int[] { l, u });
        }
        return domain.limit(limit);
    }
    
    public Domain union(Domain that) {
        Domain domain = new Domain();
        Iterator<int[]> iter1 = this.intervals.iterator();
        Iterator<int[]> iter2 = that.intervals.iterator();
        int[] r1 = null;
        int[] r2 = null;
        while (true) {
            if (r1 == null && iter1.hasNext())
                r1 = iter1.next();
            if (r2 == null && iter2.hasNext())
                r2 = iter2.next();
            if (r1 == null || r2 == null)
                break;
            if (r1[1] + 1 < r2[0]) {
                domain.intervals.add(r1);
                r1 = null;
            } else if (r2[1] + 1 < r1[0]) {
                domain.intervals.add(r2);
                r2 = null;
            } else if (r1[1] < r2[1]) {
                r2 = new int[] { Math.min(r1[0], r2[0]), r2[1] };
                r1 = null;
            } else {
                r1 = new int[] { Math.min(r1[0], r2[0]), r1[1] };
                r2 = null;
            }
        }
        if (r1 != null) {
            domain.intervals.add(r1);
        }
        if (r2 != null) {
            domain.intervals.add(r2);
        }
        while (iter1.hasNext()) {
            domain.intervals.add(iter1.next());
        }
        while (iter2.hasNext()) {
            domain.intervals.add(iter2.next());
        }
        return domain.limit(Math.max(this.limit, that.limit));
    }
    
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Domain(");
        String delim = "";
        for (int[] r : intervals) {
            if (r[0] == r[1])
                sb.append(delim + r[0]);
            else
                sb.append(delim + r[0] + ".." + r[1]);
            delim = ", ";
        }
        sb.append(")");
        return sb.toString();
    }
}
