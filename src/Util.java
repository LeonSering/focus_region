package focusmap;


import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geomgraph.Edge;
//import com.vividsolutions.jts.geom.Point; // Die Klasse wird verwendet, aber immer mit Pfadangabe.
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.BasicFeature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureSchema;
import com.vividsolutions.jump.io.DriverProperties;
import com.vividsolutions.jump.io.ShapefileWriter;

public class Util {
	public static int min(int x, int y) {
		int ret = x;
		if (y < ret)
			ret = y;
		return ret;
	}
	
	public static int abs(int x) {
		if (x < 0)
			x = -x;
		return x;
	}


	
	
	public static void ExportScalePointMap(List<Point> list, HashMap<Point, Double> scales) {
        String filename = "G:\\workspace_eclipse\\FocusMapSVN\\GIS_data\\ScalePointMap.shp";
        ShapefileWriter shp_output = new ShapefileWriter();
        DriverProperties dpw = new DriverProperties(filename);
        try{
            FeatureSchema fs = new FeatureSchema();
            fs.addAttribute("SHAPE", AttributeType.GEOMETRY);
            fs.addAttribute("scale", AttributeType.DOUBLE);
            LinkedList<BasicFeature> myList = new LinkedList<BasicFeature>();
            
            GeometryFactory gf = new GeometryFactory();
            
            for (Point p : list) {
                BasicFeature bf = new BasicFeature(fs);
                
                com.vividsolutions.jts.geom.Point jtsPoint = gf.createPoint(new Coordinate(p.getX(), p.getY()));
                bf.setGeometry(jtsPoint);
                bf.setAttribute("scale", scales.get(p));
                myList.push(bf);
            }
            
            
            FeatureCollection myFeatureCollection = new FeatureDataset(myList, fs);
            System.out.println("ScalePointMap written to " + filename);
            shp_output.write(myFeatureCollection, dpw);
            
        } catch(Exception ex) { 
            System.out.println("shp_write: " + ex);
        }
	}

	public static void ExportResiduen(LinkedList<LineString> residuenLines, HashMap<LineString, Double> residuen) {
		String filename = "G:\\workspace_eclipse\\FocusMapSVN\\GIS_data\\Residuen.shp";
        ShapefileWriter shp_output = new ShapefileWriter();
        DriverProperties dpw = new DriverProperties(filename);
        try{
            FeatureSchema fs = new FeatureSchema();
            fs.addAttribute("SHAPE", AttributeType.GEOMETRY);
            fs.addAttribute("residuum", AttributeType.DOUBLE);
            LinkedList<BasicFeature> myList = new LinkedList<BasicFeature>();
            
            
            for (LineString line : residuenLines) {
                BasicFeature bf = new BasicFeature(fs);

                bf.setGeometry(line);
                bf.setAttribute("residuum", residuen.get(line));
                myList.push(bf);
            }
            
            
            FeatureCollection myFeatureCollection = new FeatureDataset(myList, fs);
            System.out.println("Residuen written to " + filename);
            shp_output.write(myFeatureCollection, dpw);
            
        } catch(Exception ex) { 
            System.out.println("shp_write: " + ex);
        }
		
	}
	
	
	public static double det(Point p1, Point p2, Point p3) {
		// e1: p1->p2
		// e2: p1->p3
		double e1x = p2.getX() - p1.getX();
		double e1y = p2.getY() - p1.getY();
		double e2x = p3.getX() - p1.getX();
		double e2y = p3.getY() - p1.getY();
		
		return e1x*e2y - e2x*e1y;
	}
	// gibt den Richtungsvektor der Seitensenkrechten von p1->p2 normiert zurueck.
	// Dabei wird der Vektor p1->p2 um 90 Grad im Urzeigersinn gedreht.
	public static double[] berechneSeitensenkrechte(Point p1, Point p2) {
		// Bedingung zwischen p1 und p2:
        double x1 = p1.getX();
        double x2 = p2.getX();
        double y1 = p1.getY();
        double y2 = p2.getY();
        
        double [] ret = new double[2];
        // Vektor p1->p2 um 90 Grad drehen:
        //  0 1
        // -1 0
        ret[0] = y2 - y1;
        ret[1] = -x2 + x1;
        
        // normieren:
        double laenge_2 = ret[0] * ret[0] + ret[1] * ret[1];
        if (laenge_2 == 0) { // es handelt sich um den 0 Vektor
        	return ret;
        }
        
        ret[0] = ret[0] / Math.sqrt(laenge_2);
        ret[1] = ret[1] / Math.sqrt(laenge_2);
        return ret;
	}
	
	public static double[] berechneWinkelhalbierende(Point q, Point t1, Point t2) {
		Point[] t = new Point[2];
		t[0] = t1;
		t[1] = t2;
		
		
		double [] vX = new double[2];
		double [] vY = new double[2];
		
		double [] ret = new double[2];
		
		for(int i = 0; i < 2; i++) {
		
			vX[i] = t[i].getX() - q.getX();
			vY[i] = t[i].getY() - q.getY();
			// v[i]: q -> t[i]
			
			// normieren:
			double vLaenge_2 = (vX[i] * vX[i] + vY[i]*vY[i]);
			
			if (vLaenge_2 == 0) { // v ist nicht Nullvektor
				ret[0] = 0.0;
				ret[1] = 0.0;
				return ret;
			
			}
				
				
			vX[i] = vX[i] / Math.sqrt(vLaenge_2);
			vY[i] = vY[i] / Math.sqrt(vLaenge_2);
			
			
		}
		// beide normierten Vektoren addieren:
		ret[0] = vX[0] + vX[1];
		ret[1] = vY[0] + vY[1];
		
		// normieren:
		double Laenge_2 = (ret[0] * ret[0] + ret[1]*ret[1]);
		
		if (Laenge_2 == 0) { // v ist nicht Nullvektor
			return berechneSeitensenkrechte(q, t1);
		}
			
			
		ret[0] = ret[0] / Math.sqrt(Laenge_2);
		ret[1] = ret[1] / Math.sqrt(Laenge_2);
		
		return ret;
	}

	public static HashMap<Coordinate, LinkedList<Edge>> computeIntersections(LinkedList<Edge> edges, HashMap<Point, Point> newPointOf) {

        
        //compute intersections
        SegmentIntersectionSweep myIntersector = new SegmentIntersectionSweep();
        HashMap<Coordinate, LinkedList<Edge>> myIntersections = myIntersector.segmentIntersection(edges);
        //System.out.println(myIntersections.size() + " intersections found.");
        return myIntersections;
	}
	
	public static void addIntersectionConstraints(IloCplex cplex, IloNumVar[] x, IloNumVar[] y, HashMap<Coordinate, LinkedList<Edge>> myIntersections, HashMap<Edge, Point[]> oldPointsOfEdge, HashMap<Point, Integer> vertexIndices, Envelope outEnv, double epsilon) throws IloException {
		for(Map.Entry<Coordinate, LinkedList<Edge>> intersections : myIntersections.entrySet()) {
            Object[] intersectingEdges = intersections.getValue().toArray();
            for (int i = 0; i < intersectingEdges.length; i++) {
                Edge e1 = (Edge) intersectingEdges[i];
                
                Point[] e1Points = oldPointsOfEdge.get(e1);
                /*if (vertexIndices.get(e1Points[0]) == null) {
                    System.out.println("null");
                } */
                int e1_index1 = vertexIndices.get(e1Points[0]);
                int e1_index2 = vertexIndices.get(e1Points[1]);
                
                
                for (int j = i + 1; j < intersectingEdges.length; j++) {
                    Edge e2 = (Edge) intersectingEdges[j];
                    Point[] e2Points = oldPointsOfEdge.get(e2);
                    int e2_index1 = vertexIndices.get(e2Points[0]);
                    int e2_index2 = vertexIndices.get(e2Points[1]);
                    
                    Point[] supportVector = getSupportVector(e1Points, e2Points);
                    
                    double x0SV = supportVector[0].getX();
                    double y0SV = supportVector[0].getY();
                    //int sv_index = vertexIndices.get(supportVector[0]);
                    double dxSV = supportVector[1].getX() - x0SV;
                    double dySV = supportVector[1].getY() - y0SV;
                    
                    IloNumVar xL = cplex.numVar(outEnv.getMinX(), outEnv.getMaxX());
                    IloNumVar yL = cplex.numVar(outEnv.getMinY(), outEnv.getMaxY());
                            
                    
                    
                    //add constraint for point 1 of edge 1
                    // if (supportVector[0] != e1Points[0]) {
                        double dxP = e1Points[0].getX() - x0SV;
                        double dyP = e1Points[0].getY() - y0SV;
                        
                        IloLinearNumExpr expr = cplex.linearNumExpr();
                        expr.addTerm(x[e1_index1], dySV);
                        
                        //expr.addTerm(x[sv_index], -dySV);
                        expr.addTerm(xL, -dySV);
                        
                        expr.addTerm(y[e1_index1], -dxSV);
                        
                        //expr.addTerm(y[sv_index], dxSV);
                        expr.addTerm(yL, dxSV);
                        
                        if (dxP * dySV - dyP * dxSV > 0.0) {                        
                            cplex.addGe(expr, epsilon);
                        } else {
                            cplex.addLe(expr, -epsilon);
                        }
                    //}
                    
                    //add constraint for point 2 of edge 1
                    //if (supportVector[0] != e1Points[1]) {
                        /*double*/ dxP = e1Points[1].getX() - x0SV;
                        /*double*/ dyP = e1Points[1].getY() - y0SV;
                        
                        /*IloLinearNumExpr*/ expr = cplex.linearNumExpr();
                        expr.addTerm(x[e1_index2], dySV);
                        
                        //expr.addTerm(x[sv_index], -dySV);
                        expr.addTerm(xL, -dySV);
                        
                        expr.addTerm(y[e1_index2], -dxSV);
                       
                        //expr.addTerm(y[sv_index], dxSV);
                        expr.addTerm(yL, dxSV);
                        
                        
                        
                        if (dxP * dySV - dyP * dxSV > 0.0) {                        
                            cplex.addGe(expr, epsilon);
                        } else {
                            cplex.addLe(expr, -epsilon);
                        }
                    //}
                    
                    //add constraint for point 1 of edge 2
                    //if (supportVector[0] != e2Points[0]) {
                        /*double*/ dxP = e2Points[0].getX() - x0SV;
                        /*double*/ dyP = e2Points[0].getY() - y0SV;
                        
                        /*IloLinearNumExpr*/ expr = cplex.linearNumExpr();
                        expr.addTerm(x[e2_index1], dySV);
                        
                        //expr.addTerm(x[sv_index], -dySV);
                        expr.addTerm(xL, -dySV);
                        
                        
                        expr.addTerm(y[e2_index1], -dxSV);
                        
                        //expr.addTerm(y[sv_index], dxSV);
                        expr.addTerm(yL, dxSV);
                        
                        if (dxP * dySV - dyP * dxSV > 0.0) {                        
                            cplex.addGe(expr, epsilon);
                        } else {
                            cplex.addLe(expr, -epsilon);
                        }
                    //}
                    
                    //add constraint for point 2 of edge 2
                    //if (supportVector[0] != e2Points[1]) {
                        /*double*/ dxP = e2Points[1].getX() - x0SV;
                        /*double*/ dyP = e2Points[1].getY() - y0SV;
                        
                        /*IloLinearNumExpr*/ expr = cplex.linearNumExpr();
                        expr.addTerm(x[e2_index2], dySV);
                        
                        //expr.addTerm(x[sv_index], -dySV);
                        expr.addTerm(xL, -dySV);
                        
                        expr.addTerm(y[e2_index2], -dxSV);
                        
                        //expr.addTerm(y[sv_index], dxSV);
                        expr.addTerm(yL, dxSV);
                        
                        if (dxP * dySV - dyP * dxSV > 0.0) {                        
                            cplex.addGe(expr, epsilon);
                        } else {
                            cplex.addLe(expr, -epsilon);
                        }
                    //}
                }
            }             
        }
	}
	
    public static Point[] getSupportVector(Point[] e1Points, Point[] e2Points) {
        //compute distance between vertices of edge e1 and straight line supporting e2        
        Point p1_1 = e1Points[0];
        Point p1_1c = p1_1.getClosestPoint(e2Points);
        double d1_1 = p1_1.getSquareDistance(p1_1c);
        
        Point p1_2 = e1Points[1];
        Point p1_2c = p1_2.getClosestPoint(e2Points);
        double d1_2 = p1_2.getSquareDistance(p1_2c);
        
        double d1 = d1_1;
        Point p1 = p1_1;
        Point p1c = p1_1c;
        if (d1_2 < d1) {
            d1 = d1_2;
            p1 = p1_2;
            p1c = p1_2c;
        }
        
        //compute distance between vertices of edge e2 and straight line supporting e1        
        Point p2_1 = e2Points[0];
        Point p2_1c = p2_1.getClosestPoint(e1Points);
        double d2_1 = p2_1.getSquareDistance(p2_1c);
        
        Point p2_2 = e2Points[1];
        Point p2_2c = p2_2.getClosestPoint(e1Points);
        double d2_2 = p2_2.getSquareDistance(p2_2c);
        
        double d2 = d2_1;
        Point p2 = p2_1;
        Point p2c = p2_1c;
        if (d2_2 < d2) {
            d2 = d2_2;
            p2 = p2_2;
            p2c = p2_2c;
        }
        
        //construct support vector
        Point[] supportVector = new Point[2];
        if (d1 < d2) {
            supportVector[0] = p1;
            double dx = p1c.getX() - p1.getX(); 
            double dy = p1c.getY() - p1.getY();
            double l = Math.sqrt(dx * dx + dy * dy);
            double x = p1.getX() - dy / l;
            double y = p1.getY() + dx / l;
            supportVector[1] = new Point(x, y);         
        } else {
            supportVector[0] = p2;
            double dx = p2c.getX() - p2.getX(); 
            double dy = p2c.getY() - p2.getY();
            double l = Math.sqrt(dx * dx + dy * dy);
            double x = p2.getX() - dy / l;
            double y = p2.getY() + dx / l;
            supportVector[1] = new Point(x, y);
        }
        return supportVector;
    }
	
    public static int orientationConstraints(IloCplex cplex, IloNumVar[] x, IloNumVar[] y, SimpleGraph<Point, DefaultEdge> gTri, HashMap<Point, Integer> vertexIndices, HashMap<Point, Point> newPointOf, double epsilon2, boolean countOnly, boolean all) throws IloException {
    	int orientationCount = 0;
    	LinkedList<DefaultEdge> verbleibendeKanten = new LinkedList<DefaultEdge>(gTri.edgeSet());
    	Point[] p = new Point[3];
    	while(!verbleibendeKanten.isEmpty()) {
    		DefaultEdge e1 = verbleibendeKanten.iterator().next(); // Hauptkante
    		verbleibendeKanten.remove(e1);
    		p[0] = gTri.getEdgeSource(e1); // Alte Knoten
    		p[1] = gTri.getEdgeTarget(e1);
    		Set<DefaultEdge> kandidaten = gTri.edgesOf(p[0]);
    		for(DefaultEdge e2: kandidaten) {
    			if(!verbleibendeKanten.contains(e2)) 
    				continue; // Nur falls e2 noch nicht als Hauptkante betrachtet wurde
    			
    			p[2] = gTri.getEdgeTarget(e2);
    			if (p[2] == p[0]) 
    				p[2] = gTri.getEdgeSource(e2);
    			
    			if (!gTri.containsEdge(p[1], p[2]))
    				continue;
    			DefaultEdge e3 = gTri.getEdge(p[1], p[2]);
    			if(!verbleibendeKanten.contains(e3)) 
    				continue; // Nur falls e3 noch nicht als Hauptkante betrachtet wurde
    			
    			// Dreieck wurde gefunden:
    			
    			// Ueberpruefen, ob Orientierung umgedreht wurde:
    			
    			
    			double detOld = Util.det(p[0], p[1], p[2]);
    			double detNew = Util.det(newPointOf.get(p[0]), newPointOf.get(p[1]), newPointOf.get(p[2]));
    			
    			if (detOld * detNew >= 0 && !all) // Es hat keine Orientierungsaenderung stattgefunden
    				continue;
    			
    		
    			orientationCount++;
    			
    			if(countOnly)
    				continue;
    			
    			// Punkt mit dem groessten Winkel (der laengsten Steite gegenueber) auswaehlen, um dort die Bedingung anzusetzen.
    			// beliebig initialisiert.
    			Point q = p[0];
    			Point t[] = new Point[2];
    			t[0] = p[1];
    			t[1] = p[2];
    			double laengsteSeite = 0.0; // laengste Seite^2
    			for (int i = 0; i < 3; i++) {
    				Point p0 = p[i];
    				Point p1 = p[(i+1) % 3];
    				Point p2 = p[(i+2) % 3];
    				
    				double seite = (p1.getX() - p2.getX())*(p1.getX() - p2.getX()) + (p1.getY() - p2.getY())*(p1.getY() - p2.getY()); 
    				if (seite > laengsteSeite) {
    					laengsteSeite = seite;
    					q = p0;
    					t[0] = p1;
    					t[1] = p2;
    				}
    			}
    			
    			
    			
    			int qIndex = vertexIndices.get(q);
    			//System.out.println("Fuege Bedingung bei Dreieck (" + qIndex + ", " + vertexIndices.get(t[0]) + ", " + vertexIndices.get(t[1]) + ") an!");
    			
    			// Der Richtungsvektor der Seitenhalbierenden.
    			double [] result = Util.berechneWinkelhalbierende(q, t[0], t[1]);
    			
                double richtX = result[0];
                double richtY = result[1];
                if (richtX == 0 && richtY == 0) {
                	System.out.println("FEHLER!!!");
                }
                
           
                double richtSenkX = richtY;
                double richtSenkY = -richtX;
                
                /*if(!firstTime) {
                	System.out.println("detOld: " + detOld + " | detNew: " + detNew);
                    System.out.println("q = " + q.getX() + ", " + q.getY() + " | t0 = " + t[0].getX() + ", " + t[0].getY() +" | t1 = "+ t[1].getX() + ", " + t[1].getY());
        			System.out.println("richtX = " + richtX + " | richtY = " + richtY + " | richtSenkX = " + richtSenkX + " | richtSenkY = " + richtSenkY);
        			Point qNew = newPointOf.get(q); 
        			Point [] tNew = new Point[2]; tNew[0] = newPointOf.get(t[0]); tNew[1] = newPointOf.get(t[1]);
        			System.out.println("q = " + qNew.getX() + ", " + qNew.getY() + " | t0 = " + tNew[0].getX() + ", " + tNew[0].getY() +" | t1 = "+ tNew[1].getX() + ", " + tNew[1].getY());
        			System.out.println();
                }*/
                
                
    			for (int i = 0; i < 2; i++) {
    				int tIndex = vertexIndices.get(t[i]);
    				// Seitenhalbierende
    				IloLinearNumExpr expr1 = cplex.linearNumExpr();
                    expr1.addTerm(y[tIndex], richtX);
                    expr1.addTerm(y[qIndex], -richtX);
                    expr1.addTerm(x[tIndex], -richtY);
                    expr1.addTerm(x[qIndex], richtY);
                    
                    if ((t[i].getY() - q.getY())*richtX - (t[i].getX() - q.getX())*richtY <= 0) {
                    	cplex.addLe(expr1, -epsilon2); // (yT - yQ)*dx - (xT - xQ)*dy <= 0
                    //	System.out.println("Links!");
                    } else {
                    	cplex.addGe(expr1, epsilon2); // (yT - yQ)*dx - (xT - xQ)*dy >= 0
                    //	System.out.println("Rechts!");
                    }
                    
                    // Senkrechte der Seitenhalbierenden
                    IloLinearNumExpr expr2 = cplex.linearNumExpr();
                    expr2.addTerm(y[tIndex], richtSenkX);
                    expr2.addTerm(y[qIndex], -richtSenkX);
                    expr2.addTerm(x[tIndex], -richtSenkY);
                    expr2.addTerm(x[qIndex], richtSenkY);
                    
                    cplex.addGe(expr2, epsilon2); // (yT - yQ)*dx - (xT - xQ)*dy >= 0
                    if ((t[i].getY() - q.getY())*richtSenkX - (t[i].getX() - q.getX())*richtSenkY >= 0) {
                    	//cplex.addGe(expr2, epsilon2); // (yT - yQ)*dx - (xT - xQ)*dy >= 0
                    	//System.out.println("Wie gedacht!");
                    } else {
                    	//cplex.addLe(expr2, epsilon2); // (yT - yQ)*dx - (xT - xQ)*dy <= 0
                    	//System.out.println("detOld = " +  detOld + " | detNew = " + detNew);
                    	System.out.println("Nicht wie gedacht!");
                    }
    			}
    			
    		}
    	}
    	return orientationCount;
    }
    
}
