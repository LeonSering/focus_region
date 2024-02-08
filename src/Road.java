package focusmap;
import java.util.Vector;

import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.impl.CoordinateArraySequence;

public class Road {
    private Vector<Point> vertices;
    private String type;
    private RoadSymbol symbol;
    private int id;
    
    public Road(Vector<Point> v, String s, int i) {
        vertices = v;
        type = s;
        symbol = RoadSymbol.DEFAULT;
        id = i;
    }
    
    public int getId() {
        return id;
    }
    
    public Vector<Point> getVertices() {
        return vertices;
    }
    
    public String getType() {
        return type;
    }
          
    public LineString getAsLineString() {
        CoordinateArraySequence points = new CoordinateArraySequence(vertices.size());
        for (int i = 0; i < vertices.size(); i++) {
            Point p = (Point) vertices.get(i);
            points.setOrdinate(i, 0, p.getX());
            points.setOrdinate(i, 1, p.getY());
        }
        GeometryFactory gf = new GeometryFactory();
        LineString ls = new LineString(points, gf);
        return ls;
    }
    
    public int compareByLayer(Road r) {
        if (this.symbol.getLayer() > r.symbol.getLayer()) return 1;
        if (this.symbol.getLayer() == r.symbol.getLayer()) return 0;
        return -1;
    }
      
    public double getLength() {
    	double length = 0.0;
    	
    	for(int i = 0; i < vertices.size() - 1; i++) {
    		double x1 = vertices.get(i).getX();
    		double y1 = vertices.get(i).getY();
    		double x2 = vertices.get(i+1).getX();
    		double y2 = vertices.get(i+1).getY();
    		
    		length += Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    	}
    	
    	
    	return length;
    }

    public void setSymbol(RoadSymbol symbol) {
        this.symbol = symbol;
    }

    public RoadSymbol getSymbol() {
        return symbol;
    }
    /**
     * Method to compute the angle between two roads at a given vertex 
     * that is assumed to be either the first or last vertex of both roads
     * @param r2 the second road
     * @param v the vertex
     * @return the angle that this road forms with road r2 at vertex v 
     * or NaN if both roads do not share vertex v.
     */
    public double getAngleWith(Road r2, Point v) {
        
        if (this.getVertices().size() < 2 || r2.getVertices().size() < 2) {
            return Double.NaN;
        }
        
        //get line segment of first road at v
        Point l1_v2 = null;
        if (this.getVertices().firstElement() == v) {
            l1_v2 = this.getVertices().get(1);
        } else if (this.getVertices().lastElement() == v) {            
            l1_v2 = this.getVertices().get(this.getVertices().size() - 2);
        } else {
            System.out.println("First road not incident to v.");
            return Double.NaN;
        }
        double dx1 = l1_v2.getX() - v.getX();
        double dy1 = l1_v2.getY() - v.getY();
        
        //get line segment of second road at v
        Point l2_v2 = null;
        if (r2.getVertices().firstElement() == v) {
            l2_v2 = r2.getVertices().get(1);
        } else if (r2.getVertices().lastElement() == v) {            
            l2_v2 = r2.getVertices().get(r2.getVertices().size() - 2);
        } else {
            System.out.println("Second road not incident to v.");
            return Double.NaN;
        }
        double dx2 = l2_v2.getX() - v.getX();
        double dy2 = l2_v2.getY() - v.getY();
        
        double a1 = Math.atan2(dy1, dx1);
        double a2 = Math.atan2(dy2, dx2);
        double da = Math.abs(a1 - a2); // angle difference   0 <= da <= 2pi
        da = Math.PI - Math.min(da, 2 * Math.PI - da);
        //System.out.println(this.id + " " + r2.id + " " + da * 180.0 / Math.PI);
        return da;
       
    }
    
    /**
     * Method to merge two roads 
     * @param r2 the road that is to be merged with this
     * @param point a vertex shared by this and r2
     * @return the road resulting from the merge of this and r2
     */
    public Road merge(Road r2, Point v) {
        Vector<Point> points = new Vector<Point>();
        
        //add vertices of road this
        if (this.getVertices().firstElement() == v) {
            for (int i = this.getVertices().size() - 1; i >= 0; i--) points.add(this.getVertices().get(i));
        } else if (this.getVertices().lastElement() == v) {            
            for (int i = 0; i < this.getVertices().size(); i++) points.add(this.getVertices().get(i));
        } else {
            return null;
        }
        
        //add vertices of road r2
        if (r2.getVertices().firstElement() == v) {
            for (int i = 1; i < r2.getVertices().size(); i++) points.add(r2.getVertices().get(i));
        } else if (r2.getVertices().lastElement() == v) {
            for (int i = r2.getVertices().size() - 2; i >= 0; i--) points.add(r2.getVertices().get(i));
        } else {
            return null;
        }
        
        return new Road(points, this.getType(), this.id);
    }

    public void addPoint(Point v3Snapped, Point v3) {

        Vector<Point> newVertices = new Vector<Point>();
        for (int i = 0; i < vertices.size() - 1; i++) {
            Point v1 = vertices.get(i);
            Point v2 = vertices.get(i + 1);

            newVertices.add(v1);

            double dx12 = v2.getX() - v1.getX();
            double dy12 = v2.getY() - v1.getY();
            double dx13 = v3.getX() - v1.getX();
            double dy13 = v3.getY() - v1.getY();
            
            double dQ = Math.abs(dx12 * dy13 - dy12 * dx13);
            double dL = dx12 * dx13 + dy12 * dy13;
            if (dQ < 0.000001 && 0 < dL && dL < dx12 * dx12 + dy12 * dy12) {
                newVertices.add(v3Snapped);
            }
        }
        newVertices.add(vertices.get(vertices.size() - 1));
        vertices = newVertices;
    }
    
    /**
     * Gibt die laenge der Strasse road in der Scheibe mit Mittelpunkt m und Radius r zurueck.
     */
    public double roadLength(Point m, double r) {
        double laenge = 0;
        
        for (int i = 0; i< this.getVertices().size()-1; i++) {
                Point p1 = this.getVertices().get(i);
                Point p2 = this.getVertices().get(i+1);
                
                double a1 = p1.getX();
                double a2 = p1.getY();
                double b1 = p2.getX();
                double b2 = p2.getY();
                double v1 = b1 - a1;
                double v2 = b2 - a2;
                double m1 = m.getX();
                double m2 = m.getY();
                
                double p = (2*v1*(a1-m1)+2*v2*(a2-m2))/(v1*v1+v2*v2);
                double q = ((a1-m1)*(a1-m1) + (a2-m2)*(a2-m2) - r*r)/(v1*v1+v2*v2);
                double o = (p*p/4.0)-q;
                
                double t1 = -1.0;
                double t2 = -1.0;
                
                if(o >= 0) {
                        t1 = -p/2.0 - Math.sqrt(o);
                        t2 = -p/2.0 + Math.sqrt(o);
                }
                
                Point anfang = p1;
                Point ende = p2;

                if((a1-m1)*(a1-m1) + (a2-m2)*(a2-m2) < r*r) { // p1 befindet sich im Kreis
                        if(t2 <= 1) { // p2 befindet sich ausserhalb
                                ende = new Point(a1+v1*t2, a2+v2*t2);
                        }
                }
                else if ((b1-m1)*(b1-m1) + (b2-m2)*(b2-m2) < r*r){// p2 befindet sich im Kreis (p1 ausserhalb)
                        anfang = new Point(a1+v1*t1, a2+v2*t1);
                }
                else { // p1 und p2 ausserhalb
                        if (0 <= t1 && t1 <= 1 && 0 <= t2 && t2 <= 1) {
                                anfang = new Point(a1+v1*t1, a2+v2*t1);
                                ende = new Point(a1+v1*t2, a2+v2*t2);
                        }
                        else { // stecke befindet sich ueberhaupt nicht (oder nur in einem Punkt) im Kreis
                                continue;
                        }
                                
                }
                double dx = ende.getX() - anfang.getX();
                double dy = ende.getY() - anfang.getY();
                laenge += Math.sqrt(dx*dx + dy*dy);
                
                
                
        }
        
        return laenge;
    
    }
}
