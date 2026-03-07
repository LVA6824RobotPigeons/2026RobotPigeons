package org.py.cmm5;

public class DPoint {

    public double x;
    public double y;

    public DPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "[" + x + ", " + y + "]";
    }

}
