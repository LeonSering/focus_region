package focusmap;

public class Point { //implements Comparable<Object> {
    private double x;
    private double y;
    
    public Point(double myX, double myY) {
        x = myX;
        y = myY;        
    }
    
    public double getX() {
        return x;
    }
    
    public double getY() {
        return y;
    }
    public void setX(double myX) {
        x = myX;
    }
    
    public void setY(double myY) {
        y = myY;
    }
    
    public double getSquareDistance(Point p) {
        double dx = p.x - x;
        double dy = p.y - y;
        return dx * dx + dy * dy;
    }

    /**
     * Computes the point on edge e2 = (p,q) closest to this
     * @param e2Points the vertices of e2
     * @return either a new point on e2 or a vertex of e2
     */
    public Point getClosestPoint(Point[] e2Points) {
        double dxE = e2Points[1].x - e2Points[0].x;
        double dyE = e2Points[1].y - e2Points[0].y;
        
        double dxP = x - e2Points[0].x;
        double dyP = y - e2Points[0].y;
        
        double l2E = dxE * dxE + dyE * dyE;
        double s = dxE * dxP + dyE * dyP;
        
        if (0 < s && s < l2E && l2E > 0) {
            //closest point is not a vertex of e2
            double xNew = e2Points[0].x + (s / l2E) * dxE;
            double yNew = e2Points[0].y + (s / l2E) * dyE;
            return new Point(xNew, yNew);
            
        } else {            
            //closest point is a vertex of e2
            double dxQ = x - e2Points[1].x;
            double dyQ = y - e2Points[1].y;
            
            if (dxP * dxP + dyP * dyP < dxQ * dxQ + dyQ * dyQ) {
                return e2Points[0];
            } else {
                return e2Points[1];
            }           
        }
    }
    
   /* public int compareTo(Object o) {
        if(o instanceof Point) {
                Point arg0 = (Point) o;
                if(x > arg0.x || (x == arg0.x && y > arg0.y)) return 1;
                if(x == arg0.x && y == arg0.y) return 0;
                return -1; 
        }
        return 0;
    }

    public int hashCode() {
        return Integer.parseInt("" + (int) (x / 1000.0) + (int) (y / 1000.0));
    }

    public boolean equals(Object o) {
        Point p0 = (Point) o;
        return (x == p0.x && y == p0.y);
    }*/
}
