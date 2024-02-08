package focusmap;
import java.util.Iterator;
import java.util.LinkedList;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LinearRing;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.impl.CoordinateArraySequence;
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.BasicFeature;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureSchema;
import com.vividsolutions.jump.io.DriverProperties;
import com.vividsolutions.jump.io.ShapefileReader;
import com.vividsolutions.jump.io.ShapefileWriter;

public class Statement {
	
	
	public static final double ABSTAND_OUT_OUTEST = 0.1;
	
	private double abstand_in_out = 2.0;

	
	/* Inneres Rechteck (Knoten die direkt von dem Fisheye beeinflusst werden)*/
	//private Point inPoint1, inPoint2;
	private LinkedList<Envelope> inEnvs;
	
	// All Envelopes which are Circles are in this Set.
	private LinkedList<Envelope> isCircle;
	
	/* Aeusseres Rechteck (Knoten die indirekt vom Fisheye beeinflusst werden)*/
	//private Point outPoint1, outPoint2;
	private Envelope outEnv;
	
	/* Ausserstes Rechteck (Knoten die NICHT vom Fisheye beeinflusst werden, aber in die Triangulierung
	 * und lineare Programmierung einbezogen werden.
	 */
	//private Point outestPoint1, outestPoint2;
	private Envelope outestEnv;
	
	/* der Vergroesserungsfaktor, die im inneren Rechteck wirken soll. */
	private double lowerBoundScale;
	
	public void setDefaultOut() {
		Point inPoint1 = getInPoint1();
		Point inPoint2 = getInPoint2();
		
		
		double dx = inPoint2.getX() - inPoint1.getX();
		double dy = inPoint2.getY() - inPoint1.getY();
		// da das innere Rechteck gueltig ist (inPoint1 besitzt kleinere Koordinaten als inPoint2)
		// sind dx und dy positiv.
		
		double dmax = (dx > dy) ? dx : dy;
		
		Point outPoint1 = new Point(inPoint1.getX() - dmax * abstand_in_out, inPoint1.getY() - dmax * abstand_in_out);
		Point outPoint2 = new Point(inPoint2.getX() + dmax * abstand_in_out, inPoint2.getY() + dmax * abstand_in_out);
		
		outEnv = PointsToEnv(outPoint1, outPoint2);
		
		validateOutOutest();
	}
	
	public void setDefaultOutest()  {
		Point outPoint1 = getOutPoint1();
		Point outPoint2 = getOutPoint2();
		double dx = outPoint2.getX() - outPoint1.getX();
		double dy = outPoint2.getY() - outPoint1.getY();
		// da das aeusseres Rechteck gueltig ist (inPoint1 besitzt kleinere Koordinaten als inPoint2)
		// sind dx und dy positiv.
		
		double dmax = (dx > dy) ? dx : dy;
		
		Point outestPoint1 = new Point(outPoint1.getX() - dmax * ABSTAND_OUT_OUTEST, outPoint1.getY() - dmax * ABSTAND_OUT_OUTEST);
		Point outestPoint2 = new Point(outPoint2.getX() + dmax * ABSTAND_OUT_OUTEST, outPoint2.getY() + dmax * ABSTAND_OUT_OUTEST);
		outestEnv = PointsToEnv(outestPoint1, outestPoint2);
	}
	
	// Leerer Konstruktor nur zum importieren aus shp-files
	private Statement() {
		inEnvs = new LinkedList<Envelope>();
		isCircle = new LinkedList<Envelope>();
	}
	
	public Statement(double lowerBoundScale, Point inPoint1, Point inPoint2, boolean circle) {
		setLowerBoundScale(lowerBoundScale);
		setInEnv(PointsToEnv(inPoint1, inPoint2), circle);
		setDefaultOut();
		setDefaultOutest();
	}
	
	public Statement(double lowerBoundScale, Envelope inEnv, boolean circle) {
		setLowerBoundScale(lowerBoundScale);
		setInEnv(inEnv, circle);
		setDefaultOut();
		setDefaultOutest();
	}
	
	
	
	
	public Statement(double lowerBoundScale, Point inPoint1, Point inPoint2, boolean circle, Point outPoint1, Point outPoint2) {
		setLowerBoundScale(lowerBoundScale);
		setInEnv(PointsToEnv(inPoint1, inPoint2), circle);
		setOutEnv(PointsToEnv(outPoint1, outPoint2));
		setDefaultOutest();
	}
	
	public Statement(double lowerBoundScale, Envelope inEnv, boolean circle, Envelope outEnv) {
		setLowerBoundScale(lowerBoundScale);
		setInEnv(inEnv, circle);
		setOutEnv(outEnv);
		setDefaultOutest();
	}

	public void setInEnv(Envelope inEnv, boolean circle) {
		
		inEnvs = new LinkedList<Envelope>();
		isCircle = new LinkedList<Envelope>();
		inEnvs.add(inEnv);
		if (circle == true)
	    	isCircle.add(inEnv);
		/*double xMin, xMax, yMin, yMax;
		if(inPoint1.getX() < inPoint2.getX()) {
			xMin = inPoint1.getX();
			xMax = inPoint2.getX();
		}
		else {
			xMin = inPoint2.getX();
			xMax = inPoint1.getX();			
		}
		if(inPoint1.getY() < inPoint2.getY()) {
			yMin = inPoint1.getY();
			yMax = inPoint2.getY();
		}
		else {
			yMin = inPoint2.getY();
			yMax = inPoint1.getY();			
		}
		this.inPoint1 = new Point(xMin, yMin);
		this.inPoint2 = new Point(xMax, yMax);*/
		validateInOut();
	}
	
	public void addInEnv(Envelope inEnv, boolean circle) {
		if (inEnv == null) {
		    this.setInEnv(inEnv, circle);
		} else {
		    inEnvs.add(inEnv);  
		    if (circle == true)
		    	isCircle.add(inEnv);
		}	        
		validateInOut();
	}


	public void setOutEnv(Envelope env) {
		
		outEnv = env;
		
		/*
		double xMin, xMax, yMin, yMax;
		if(outPoint1.getX() < outPoint2.getX()) {
			xMin = outPoint1.getX();
			xMax = outPoint2.getX();
		}
		else {
			xMin = outPoint2.getX();
			xMax = outPoint1.getX();			
		}
		if(outPoint1.getY() < outPoint2.getY()) {
			yMin = outPoint1.getY();
			yMax = outPoint2.getY();
		}
		else {
			yMin = outPoint2.getY();
			yMax = outPoint1.getY();			
		}
		this.outPoint1 = new Point(xMin, yMin);
		this.outPoint2 = new Point(xMax, yMax);*/
		validate();
	}

	public Point getOutPoint1() {
		
		return new Point(outEnv.getMinX(), outEnv.getMinY());
	}

	public Point getOutPoint2() {
		return new Point(outEnv.getMaxX(), outEnv.getMaxY());
	}

	public void setOutestRectangle(Envelope env) {
		outestEnv = env;
		/*
		double xMin, xMax, yMin, yMax;
		if(outestPoint1.getX() < outestPoint2.getX()) {
			xMin = outestPoint1.getX();
			xMax = outestPoint2.getX();
		}
		else {
			xMin = outestPoint2.getX();
			xMax = outestPoint1.getX();			
		}
		if(outestPoint1.getY() < outestPoint2.getY()) {
			yMin = outestPoint1.getY();
			yMax = outestPoint2.getY();
		}
		else {
			yMin = outestPoint2.getY();
			yMax = outestPoint1.getY();			
		}
		this.outestPoint1 = new Point(xMin, yMin);
		this.outestPoint2 = new Point(xMax, yMax);*/
		
		validateOutOutest();
	}

	public Point getOutestPoint1() {
		
		return new Point(outestEnv.getMinX(), outestEnv.getMinY());
	}

	public Point getOutestPoint2() {
		return new Point(outestEnv.getMaxX(), outestEnv.getMaxY());
	}

	public void setLowerBoundScale(double lowerBoundScale) {
		if (lowerBoundScale < 1)
			this.lowerBoundScale = 1;
		else
			this.lowerBoundScale = lowerBoundScale;
	}

	public double getLowerBoundScale() {
		return lowerBoundScale;
	}
	
	/* Ueberprueft ob die Rahmen ineinander geschaltelt sind. */
	private void validate() {
		validateInOut();
		validateOutOutest();
	}
	
	/* Ueberprueft ob der aeussere Rahmen um den inneren Rahmen liegt. Falls nicht, wird der aeussere auf default gesetzt. */
	private void validateInOut() {
		if(outEnv == null) {
			setDefaultOut();
			return;
		}
		Point inPoint1 = getInPoint1();
		Point inPoint2 = getInPoint2();
		Point outPoint1 = getOutPoint1();
		Point outPoint2 = getOutPoint2();
		
		if ((outPoint1 == null || outPoint2 == null) 
				|| (outPoint1.getX() > inPoint1.getX() || outPoint1.getY() > inPoint1.getY())
				|| (outPoint2.getX() < inPoint2.getX() || outPoint2.getY() < inPoint2.getY())) {
				setDefaultOut();
			}
		
	}
	
	/* Ueberprueft ob der aeusserste Rahmen um den aeusseren Rahmen liegt. Falls nicht, wird der aesserste auf default gesetzt. */
	private void validateOutOutest() {
		if(outEnv == null) {
			setDefaultOut();
		}
		if(outestEnv == null) {
			setDefaultOutest();
			return;
		}
		Point outPoint1 = getOutPoint1();
		Point outPoint2 = getOutPoint2();
		
		Point outestPoint1 = getOutestPoint1();
		Point outestPoint2 = getOutestPoint2();
		
		if ((outestPoint1 == null || outestPoint2 == null)
				|| (outestPoint1.getX() > outPoint1.getX() || outestPoint1.getY() > outPoint1.getY())
				|| (outestPoint2.getX() < outPoint2.getX() || outestPoint2.getY() < outPoint2.getY())) {
				setDefaultOutest();
			}
	}
	
	public LinkedList<Envelope> getInEnvs() {
		return inEnvs;
	}
	
	public LinkedList<Envelope> getCircles(){
		return isCircle;
	}
	
	public boolean isCircle(Envelope env) {
		return isCircle.contains(env);
	}
	
	public Envelope getOutEnv() {
		return outEnv;
		}
	
	public Envelope getOutestEnv() {
		return outestEnv;
	}
	
	
	public void setAbstand_in_out(double value) {
		abstand_in_out = value;
		System.out.println("Abstand in_out: " + value);
		this.setDefaultOut();
	}
	
	public double getAbstand_in_out() {
		return abstand_in_out;
	}

    public static Statement importFromShapefile(String filename) {
        if(filename.endsWith(".shp")) {
            System.out.println("Statement wird aus folgendem Shapefile importiert: " + filename);
            ShapefileReader shp_input = new ShapefileReader();
            DriverProperties dp = new DriverProperties(filename);
            FeatureCollection myFeatureCollection = null;
            try {
                myFeatureCollection = shp_input.read(dp);  
                Iterator<?> i = myFeatureCollection.iterator();
                
                Statement st = new Statement();
                
                while(i.hasNext()) {
	                Feature myFeature = (Feature) i.next(); 
	                        
	                Geometry myGeometry = myFeature.getGeometry();
	                Envelope env = myGeometry.getEnvelopeInternal();
	                
	                double scale = (Double) myFeature.getAttribute("scale");
	                st.setLowerBoundScale(scale);
	                try {
		                double inout = (Double) myFeature.getAttribute("inout");
		                st.setAbstand_in_out(inout);
	                } catch (Exception e) {
	                	
	                }
	                boolean isCircle = false;
	                try {
		                int isCircleInt = (Integer) myFeature.getAttribute("isCircle");
		                if (isCircleInt != 0)
		                	isCircle = true;
	                } catch (Exception e) {
	                	
	                }
	                st.addInEnv(env, isCircle);
	                
	               
	            }
                return st;
            } catch (Exception e) {
            }
        }
        return null;
    }

    public void exportToShapefile(String filename) {
        if(filename.endsWith(".shp")) {
            ShapefileWriter shp_output = new ShapefileWriter();
            DriverProperties dpw = new DriverProperties(filename);
            LinkedList<BasicFeature> myList = new LinkedList<BasicFeature>();
            try{
                FeatureSchema fs = new FeatureSchema();
                fs.addAttribute("SHAPE", AttributeType.GEOMETRY);
                fs.addAttribute("scale", AttributeType.DOUBLE); 
                fs.addAttribute("inout", AttributeType.DOUBLE);
                fs.addAttribute("isCircle", AttributeType.INTEGER);
                
                for (Envelope env: inEnvs) {
	                BasicFeature bf = new BasicFeature(fs);
	                                
	                Coordinate[] coords = new Coordinate[5];
	                
	                Point inPoint1 = new Point(env.getMinX(), env.getMinY());
	                Point inPoint2 = new Point(env.getMaxX(), env.getMaxY());
	                
	                coords[0] = new Coordinate(inPoint1.getX(), inPoint1.getY()); 
	                coords[1] = new Coordinate(inPoint1.getX(), inPoint2.getY());
	                coords[2] = new Coordinate(inPoint2.getX(), inPoint2.getY());
	                coords[3] = new Coordinate(inPoint2.getX(), inPoint1.getY());
	                coords[4] = coords[0];
	                
	                GeometryFactory gf = new GeometryFactory();
	                LinearRing rectangle = new LinearRing(new CoordinateArraySequence(coords), gf);
	              
	                Polygon p = new Polygon(rectangle , null, gf);
	                bf.setGeometry(p);
	                bf.setAttribute("scale", lowerBoundScale);
	                bf.setAttribute("inout", abstand_in_out);
	                if(isCircle.contains(env))
	                	bf.setAttribute("isCircle", 1);
	                else
	                	bf.setAttribute("isCircle", 0);
	                
	                
	                myList.add(bf);
                }
	                
	                
                FeatureCollection myFeatureCollection = new FeatureDataset(myList , fs);
                System.out.println("Shape written to " + filename);
                shp_output.write(myFeatureCollection, dpw);
                
            } catch(Exception ex) { 
                System.out.println("shp_write: " + ex);
            }
        }   
    }
    
    public Envelope getInEnvBoundingBox() {
    	Point minPoint = new Point(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    	Point maxPoint = new Point(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
    	
    	for(Envelope env: inEnvs) {
    		if (env.getMaxX() > maxPoint.getX())
    			maxPoint.setX(env.getMaxX());
    		if (env.getMaxY() > maxPoint.getY())
    			maxPoint.setY(env.getMaxY());
    		
    		if (env.getMinX() < minPoint.getX())
    			minPoint.setX(env.getMinX());
    		if (env.getMinY() < minPoint.getY())
    			minPoint.setY(env.getMinY());
    	}
    	
    	return PointsToEnv(minPoint, maxPoint);
    }
    
    // gibt den Punkt links unten der InEnvBoundingBox zurueck.
    public Point getInPoint1() {
    	Envelope boundingBox = getInEnvBoundingBox();
    	return new Point(boundingBox.getMinX(), boundingBox.getMinY());
    }
    
 // gibt den Punkt rechts oben der InEnvBoundingBox zurueck.
    public Point getInPoint2() {
    	Envelope boundingBox = getInEnvBoundingBox();
    	return new Point(boundingBox.getMaxX(), boundingBox.getMaxY());
    }
    
    public static Envelope PointsToEnv(Point p1, Point p2) {
    	return new Envelope(p1.getX(), p2.getX(), p1.getY(), p2.getY());
    }

    public Point getCenter() { // Dann Center von der InEnvBoundingBox
    	return getCenter(this.getInEnvBoundingBox());
    }
    
    public static Point getCenter(Envelope env) {
    	double xC = (env.getMaxX() + env.getMinX()) * 0.5;
        double yC = (env.getMaxY() + env.getMinY()) * 0.5;
        return new Point(xC, yC);
    }
    
    public double getRadius() { // Dann der Radius von der InEnvBoundingBox
    	return getRadius(this.getInEnvBoundingBox());
    }
    
    public static double getRadius(Envelope env) {
    	return Math.min(env.getHeight(), env.getWidth());
    	
    }
    
    public boolean inContains(Point p) { // ueberprueft ob der Punkt in einem der in Envelopes ist.
    	for(Envelope env : inEnvs) {
    		if (isCircle(env)) { // es handelt sich um einen Kreis.
    			Point c = getCenter(env);
    			double r = getRadius(env);
    			if ( (c.getX() - p.getX())*(c.getX() - p.getX()) + (c.getY() - p.getY())*(c.getY() - p.getY()) <= r*r)
    				return true;
    			
    		} else { // es handelt sich um ein Rechteck
	    		if (env.contains(p.getX(), p.getY()))
	    			return true;
    		}
    	}
    	return false;
    }

}
