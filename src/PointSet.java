package focusmap;
import java.awt.Color;
import java.awt.Graphics2D;
import java.util.List;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.index.quadtree.*;

public class PointSet implements MapObject {
    Quadtree points;
    Envelope env;
    PointSet() {
        points = new Quadtree();
        env = new Envelope();
    }
    
    void addPoint(Point p) {
        points.insert(new Envelope(p.getX(), p.getX(), p.getY(), p.getY()), p);
        env.expandToInclude(p.getX(), p.getY());
    }
  
    Point getNearestNeighbor(Point p, double searchRadius) {
        Envelope env = new Envelope(p.getX() - searchRadius, p.getX() + searchRadius, p.getY() - searchRadius, p.getY() + searchRadius);
        List<Object> l = points.query(env);
        
        Point nearest = null;
        Double d2_Min = Double.POSITIVE_INFINITY;
        
        for (Object o : l) {
            Point p2 = (Point) o;
            double dx = p2.getX() - p.getX();
            double dy = p2.getY() - p.getY();
            double s2 = dx * dx + dy * dy;
            if (s2 <= searchRadius * searchRadius && s2 < d2_Min) {
                nearest = p2;
                d2_Min = s2; 
            }
        }
        
        return nearest;
    }

    @Override
    public void drawBackground(Graphics2D g, Transformation t) {
                
    }

    @Override
    public void drawForeground(Graphics2D g, Transformation t) {
        for (Object o : points.queryAll()) {
            Point p = (Point) o;
            int col = t.getColumn(p.getX());
            int row = t.getRow(p.getY());
            g.setColor(Color.RED);
            g.drawLine(col, row, col + 5, row + 5);
            g.drawLine(col, row, col + 5, row - 5);
            g.drawLine(col, row, col - 5, row + 5);
            g.drawLine(col, row, col - 5, row - 5);
        }        
    }

    @Override
    public Envelope getBoundingBox() {
        return env;
    }
    
    public int getSize() {
        return points.size();
    }
    
    public List<Point> getAsList() {
    	return points.queryAll();
    }
}
