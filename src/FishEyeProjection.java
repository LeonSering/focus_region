package focusmap;

import java.util.Vector;

import com.vividsolutions.jts.geom.Envelope;

public class FishEyeProjection {
    private Point center;
    private Envelope env;
    private double s;
    private double r0;
    private Point B0, B1, B2, B3;
    private double r1;
    
    private int n;
    private double Px[]; // Die x-Koordinate an den stellen k/n der Bezier-Kurve (k = 0, 1, ..., n)
    private double Py[];
    
    
    public FishEyeProjection(Point p, Envelope eMapExtent, double m, double r) {
        center = p;
        env = eMapExtent;
        s = m;
        r0 = r;
        
        double xMid = center.getX();
        double yMid = center.getY();
        
        r1 = env.getMaxX() - xMid;
        if (env.getMaxY() - yMid < r1) r1 = env.getMaxY() - yMid; 
        if (xMid - env.getMinX() < r1) r1 = xMid - env.getMinX();
        if (yMid - env.getMinY() < r1) r1 = yMid - env.getMinY();
        

        //System.out.println("r1 = " + r1);
        
        B0 = new Point(r0, r0*s);
        B1 = new Point((s*r0 + r1) / (s*2.0), (s*r0 + r1) *0.5);
        B2 = new Point((s*r0 + r1) *0.5, (s*r0 + r1) *0.5);
        B3 = new Point(r1, r1);
        
        n=10;
        
        this.Px = new double[n+1];
        this.Py = new double[n+1];
        for(int k = 0; k <= n; k++) {
        	Px[k] = computePx(k/(double)n);
        	Py[k] = computePy(k/(double)n);
        }
    }
    
    public Point projectPoint(Point p) {
                
    	double xMid = center.getX();
        double yMid = center.getY();
        
        double dx = p.getX() - xMid;
        double dy = p.getY() - yMid;
        double rOld = Math.sqrt(dx * dx + dy * dy);
        double rNew;
        if (rOld <= r0) {
            rNew = s * rOld;
        } else if (r0 < rOld && rOld <= r1) {
            //rNew = s * r0 + (rOld - r0) * (r1 - s * r0) / (r1 - r0);
        	rNew = computeBezier(rOld);
        } else {
            rNew = rOld;
        }
                
        double dxNew = dx * rNew / rOld;
        double dyNew = dy * rNew / rOld;
        return new Point(xMid + dxNew, yMid + dyNew);
            
    }
    
    public Point projectPointHarrie(Point p) {
        
        double xMid = center.getX();
        double yMid = center.getY();
        
        double r0 = 1.0;
        double r1 = 6.0;
                
        double sL = 0.1;
        double sS = 0.01;
        
        double dx = p.getX() - xMid;
        double dy = p.getY() - yMid;
        double rOld = Math.sqrt(dx * dx + dy * dy);
        double rNew;
        if (rOld <= r0) {
            rNew = rOld;
        } else if (r0 < rOld && rOld <= r1) {
            rNew = rOld + (rOld - r0) * (rOld - r0) * (sS - sL) / (2 * sL * (r1 - r0));
        } else {
            rNew = r1 + (r1 - r0) * (sS - sL) / (2 * sL) + (rOld - r1) * sS / sL;
        }
                
        double dxNew = dx * rNew / rOld;
        double dyNew = dy * rNew / rOld;
        return new Point(xMid + dxNew, yMid + dyNew);
    }
    
    public RoadNetwork projectRoadNetwork(RoadNetwork rn) {
        RoadNetwork rnNew = new RoadNetwork();
        
        for(Road r : rn.getRoads()) {
            Vector<Point> points = r.getVertices();
            Vector<Point> pointsNew = new Vector<Point>(); 
            for (Point p : points) {
                pointsNew.add(projectPoint(p));
            }
            Road rNew = new Road(pointsNew, r.getType(), r.getId());
            rNew.setSymbol(r.getSymbol());
            rnNew.add(rNew);
        }
        return rnNew;
    }
    
    public double computePx(double t) {
    	return  Math.pow((1 - t),3) * B0.getX() + 3 *t* Math.pow((1 - t),2) * B1.getX() + 3 *Math.pow(t,2)* (1 - t)*B2.getX() + Math.pow(t,3) * B3.getX(); 
    }
    
    public double computePy(double t) {
    	return  Math.pow((1 - t),3) * B0.getY() + 3 *t* Math.pow((1 - t),2) * B1.getY() + 3 *Math.pow(t,2)* (1 - t)*B2.getY() + Math.pow(t,3) * B3.getY(); 
    }
    
    public double computeBezier(double r) {
    	int k = 0;
    	for (int i = 0; i < n; i++) {
    		if (r >= Px[i])
    			k = i;
    	}
    	
    	return (Py[k+1] - Py[k]) / (Px[k+1] - Px[k]) * (r-Px[k]) + Py[k];
    	
    }
    
}
