package focusmap;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.BasicStroke;
import com.vividsolutions.jts.geom.Envelope;

/**
 * This class represents a LineObject that implements
 * the MapObject interface, i.e., a LineObject can be
 * added to a MapDisplay.
 * @author Jan-Henrik Haunert
 */
public class LineObject implements MapObject {

    /**
     * The x coordinate of this Lines first Point.
     */
    private Point p1World;

    private Point p2World;

    private float foregroundSymbolWidth;

    private float backgroundSymbolWidth;

    /**
     * The Color of the stroke representing this LineObject in a map.
     */
    private Color backgroundColor;

    /**
     * The Color used to fill the symbol representing this LineObject in a map.
     */
    private Color foregroundColor;

    /**
     * Generates a new LineObject.
     * @param p1 the first Point
     * @param p2 the second Point
     * @param mySymbolWidth the width of this Line's symbol
     * @param myStrokeWidth the widths of the strokes delineating this symbol
     * @param myBackgroundColor the Color of the stroke representing this
     * LineObject in a map
     * @param myForegroundColor the Color used to fill
     * the symbol representing this LineObject in a map
     */
    public LineObject(Point p1, Point p2,
            float mySymbolWidth, float myStrokeWidth,
            Color myBackgroundColor, Color myForegroundColor) {
        p1World = p1;
        p2World = p2;
        
        foregroundSymbolWidth = mySymbolWidth - 2 * myStrokeWidth;
        backgroundSymbolWidth = mySymbolWidth;
        backgroundColor = myBackgroundColor;
        foregroundColor = myForegroundColor;
    }

    /**
     * Draws the background content of this LineSymbol into the
     * specified Graphics object.
     * @param g the specified Graphics object
     * @param t the Transformation that maps world coordinates
     * to image coordinates
     */
    @Override
    public void drawBackground(Graphics2D g, Transformation t) {
        int x1Image = t.getColumn(p1World.getX());
        int y1Image = t.getRow(p1World.getY());
        int x2Image = t.getColumn(p2World.getX());
        int y2Image = t.getRow(p2World.getY());
        BasicStroke s = new BasicStroke(backgroundSymbolWidth);
        g.setStroke(s);
        g.setColor(backgroundColor);
        g.drawLine(x1Image, y1Image, x2Image, y2Image);
    }

    /**
     * Draws the foreground content of this LineSymbol into the
     * specified Graphics object.
     * @param g the specified Graphics object
     * @param t the Transformation that maps world coordinates
     * to image coordinates
     */
    @Override
    public void drawForeground(Graphics2D g, Transformation t) {
        if (foregroundSymbolWidth <= 0) {
            return;
        }
        int x1Image = t.getColumn(p1World.getX());
        int y1Image = t.getRow(p1World.getY());
        int x2Image = t.getColumn(p2World.getX());
        int y2Image = t.getRow(p2World.getY());
        BasicStroke s = new BasicStroke(foregroundSymbolWidth);
        g.setStroke(s);
        g.setColor(foregroundColor);
        g.drawLine(x1Image, y1Image, x2Image, y2Image);
    }

    /**
     * Returns the bounding box of this LineSymbol.
     * @return the points representing the lower left corner
     * and the upper right corner of the bounding box.
     */
    @Override
    public Envelope getBoundingBox() {
        double xMin = p1World.getX();
        double xMax = p2World.getX();
        if (xMin > xMax) {
            xMin = xMax;
            xMax = p1World.getX();
        }
        double yMin = p1World.getY();
        double yMax = p2World.getY();
        if (yMin > yMax) {
            yMin = yMax;
            yMax = p1World.getY();
        }
        return new Envelope(xMin, xMax, yMin, yMax);
    }
    
    public Point getP1() {
        return p1World;
    }
    public Point getP2() {
        return p2World;
    }
    
}
