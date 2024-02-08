package focusmap;
import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

/*import java.util.Collections;
import java.util.Comparator;*/
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Set;
import java.util.Vector;
import java.util.HashMap;
//import java.util.Set;
import java.util.List;
import javax.swing.*;
import java.util.Iterator;
import javax.swing.filechooser.FileFilter;

import java.io.*;

import com.vividsolutions.jump.feature.*;
import com.vividsolutions.jump.io.*;
import com.vividsolutions.jts.geom.*;
import com.vividsolutions.jts.geomgraph.Edge;

import org.jgrapht.alg.ConnectivityInspector;
import org.jgrapht.graph.*;
//import org.jgrapht.alg.ConnectivityInspector;

// Triangulierung:
import org.poly2tri.Poly2Tri;
import org.poly2tri.triangulation.TriangulationPoint;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;
import org.poly2tri.triangulation.point.TPoint;
import org.poly2tri.triangulation.sets.ConstrainedPointSet;

public class RoadNetwork {
        
    private static final double COORD_ERROR_TOLERANCE = 0.1;
    private LinkedList<Road> roads;
    private LinkedList<Road> triRoads;
    private double minX;
    private double minY;
    private double maxX;
    private double maxY;
    private RoadSymbol[] symbolSet;
    
    private String name;
    private int maxRoadID;
    
    public RoadNetwork() {
        roads = new LinkedList<Road>();
        minX = Double.POSITIVE_INFINITY;
        minY = Double.POSITIVE_INFINITY;
        maxX = Double.NEGATIVE_INFINITY;
        maxY = Double.NEGATIVE_INFINITY;
        name = "Empty RoadMap";
        symbolSet = null;
    }
    
    public boolean add(Road r) {
        if (r.getId() > maxRoadID) maxRoadID = r.getId();
        return roads.add(r);
    }
    
    public LinkedList<Road> getRoads() {
        return roads;
    }
    
    public LinkedList<Road> getTriRoads() {
    	if (triRoads == null) {
    		// addTriRoads();  // Wegen Stackoverflow rausgenommen
    	}
        return triRoads;
    }
    
    public static class SHPFileFilter extends FileFilter
    {
	    public boolean accept(File f)
	        {
	        	return (f.isDirectory() || f.getPath().endsWith(".shp"));
	        }
	    public String getDescription()
        {
	    	return "ESRI Shapefiles";
        }
    }
    
    public void exportToShapeFile() {
        JFileChooser chooser = new JFileChooser();
        File dir = new File("GIS_data");
        chooser.setCurrentDirectory(dir );
        
        SHPFileFilter filter = new SHPFileFilter();
        chooser.setFileFilter(filter);
        int returnVal = chooser.showSaveDialog(null);
        if(returnVal == JFileChooser.APPROVE_OPTION) {
            String filename = chooser.getSelectedFile().getPath();
            if(filename.endsWith(".shp")) 
            {
            ShapefileWriter shp_output = new ShapefileWriter();
            DriverProperties dpw = new DriverProperties(filename);
            try{
                FeatureSchema fs = new FeatureSchema();
                fs.addAttribute("SHAPE", AttributeType.GEOMETRY);
                fs.addAttribute("type", AttributeType.STRING);
                LinkedList<BasicFeature> myList = new LinkedList<BasicFeature>();
                
                for (Road r : roads) {
                    BasicFeature bf = new BasicFeature(fs);
                    LineString ls = r.getAsLineString();
                    bf.setGeometry(ls);
                    bf.setAttribute("type", r.getType());
                    myList.push(bf);
                }
                
                
                FeatureCollection myFeatureCollection = new FeatureDataset(myList, fs);
                System.out.println("Shape written to " + filename);
                shp_output.write(myFeatureCollection, dpw);
                
            } catch(Exception ex) { 
                System.out.println("shp_write: " + ex);}
            }
        }
    }
    
    public static RoadNetwork importFromShapefile() {
    	RoadNetwork myRoadNetwork = new RoadNetwork();
        JFileChooser chooser = new JFileChooser();
        
        File dir = new File("GIS_data");
        chooser.setCurrentDirectory(dir );
        SHPFileFilter filter = new SHPFileFilter();
        chooser.setFileFilter(filter);
        int returnVal = chooser.showOpenDialog(null);
        if(returnVal == JFileChooser.APPROVE_OPTION) 
        {
            String filename = chooser.getSelectedFile().getPath();
            myRoadNetwork = importFromShapefile(filename);
        }
        return myRoadNetwork;
    }
    
    public static RoadNetwork importFromShapefile(String filename) {
    	
    	RoadNetwork myRoadNetwork = new RoadNetwork();
    	PointSet points = new PointSet();
    	
    	if(filename.endsWith(".shp")) 
        {
            System.out.println("Roadnetwork wird aus folgendem Shapefile importiert: " + filename);
	        ShapefileReader shp_input = new ShapefileReader();
	        DriverProperties dp = new DriverProperties(filename);
	        FeatureCollection myFeatureCollection = null;
	        try {
	            myFeatureCollection = shp_input.read(dp);
	            
	            int counter = 0;
	            
	            String attributeColumn = null;
	            
	            
	            if (myFeatureCollection.getFeatureSchema().hasAttribute("type")) {
	                //use symbols for open street map data
	                System.out.println("Darstellung fuer OSM-Daten wird verwendet.");
	                attributeColumn = "type";
	                myRoadNetwork.symbolSet = RoadSymbol.OSM_SYMBOLS;
	            } else if (myFeatureCollection.getFeatureSchema().hasAttribute("CLASS")) {
	                System.out.println("Darstellung fuer MassGIS-Daten wird verwendet.");
	                attributeColumn = "CLASS";
	                myRoadNetwork.symbolSet = RoadSymbol.MASS_GIS_SYMBOLS;
	            }
	            
	            for (Iterator<?> i = myFeatureCollection.iterator(); i.hasNext(); counter++) {
	                Feature myFeature = (Feature) i.next();
	                String type = "default";
	                if (attributeColumn != null) {
	                    Object o = myFeature.getAttribute(attributeColumn);
	                    if (o instanceof String) type = (String) o;
	                    else if (o instanceof Integer) type = "_" + (Integer) o + "_";
	                }
	                
	                  
	                
	                Geometry myGeometry = myFeature.getGeometry();
	                if (myGeometry.getGeometryType() == "LineString") {
	                    Vector<Point> PunktVektor = new Vector<Point>();
	                    Coordinate[] xyz = myGeometry.getCoordinates();
	                    for(int j = 0; j < xyz.length; j++) {
	                        Point p = new Point(xyz[j].x, xyz[j].y);
	                        
	                        Point neighbor = points.getNearestNeighbor(p, COORD_ERROR_TOLERANCE);
                                if (neighbor != null) {
                                    //System.out.println("Snap");
                                    p = neighbor;
                                } else {
                                    points.addPoint(p);
                                    if (p.getX() < myRoadNetwork.getMinX()) myRoadNetwork.minX = p.getX();
                                    if (p.getY() < myRoadNetwork.getMinY()) myRoadNetwork.minY = p.getY();
                                    if (p.getX() > myRoadNetwork.getMaxX()) myRoadNetwork.maxX = p.getX();
                                    if (p.getY() > myRoadNetwork.getMaxY()) myRoadNetwork.maxY = p.getY();
                                }
	                        PunktVektor.add(p);
	                        }                
	                    
	                    Road myRoad = new Road(PunktVektor, type, myRoadNetwork.maxRoadID + 1);
	                    RoadSymbol rs = RoadSymbol.getSymbolForClass(myRoadNetwork.symbolSet, type);
	                    myRoad.setSymbol(rs);
	                    myRoadNetwork.add(myRoad);
	                }
	                else {
	                    System.out.println("The shapefile does not contain a polyline but a " + myGeometry.getGeometryType());                                 
	                }
	            }
	            int n = filename.lastIndexOf('/') + 1;
	            myRoadNetwork.setName(filename.substring(n,filename.length() - 4)); // Den Pfad und die ".shp" Endung abschneiden.
	        } catch(Exception ex){System.out.println("shp_read: " + ex);}
	        
	    }
    	int n1 = points.getSize();
    	myRoadNetwork.addIntersectionPoints(points);
    	int n2 = points.getSize();
    	System.out.println((n2 - n1) + " points added");
    	
    	myRoadNetwork.addTriRoads();
    	  
    	return myRoadNetwork;
    }


    private SimpleGraph<Point, DefaultEdge> getAsGraph() { // gibt das gesamte Roadnetwork als Graph zurueck.
    	
    	return getAsGraph(this.getEnv());
    }
    private SimpleGraph<Point, DefaultEdge> getAsGraph(Envelope env) { // Gibt das Roadnetwork als Graph im Envelope aus.
        SimpleGraph<Point, DefaultEdge> g = new SimpleGraph<Point, DefaultEdge>(DefaultEdge.class);
        //Fuege alle Strassen des Strassennetzes ein
        for(Road r : roads) {
        	
        	if (!r.getType().startsWith("triedge")) { // TriEdges waren nur Hilfsstrassen zur Visualisierung der Triangulierung
        											  // und sollen nun keine Rolle mehr spielen.
        	
	            Vector<Point> points = r.getVertices();
	            Point lastPoint = points.elementAt(0);
	            if(env == null || env.contains(lastPoint.getX(), lastPoint.getY()))
	            	g.addVertex(lastPoint);
	            
	            //Fuege alle Ecken der Strasse ein
	            for(int i = 1; i < points.size(); i++) {
	                Point nextPoint = points.elementAt(i);
	                if (lastPoint != nextPoint) {
	                    if(env == null || env.contains(nextPoint.getX(), nextPoint.getY())) {
	                        g.addVertex(nextPoint);
	                	if(env == null || env.contains(lastPoint.getX(), lastPoint.getY()))
	                	    g.addEdge(lastPoint, nextPoint);
	                    }
	                lastPoint = nextPoint;
	                }
	            }
            }
        }
        return g;
    }
    
    /**
     * Trianguliert das RoadNetwork (sich selbst) und fuegt triRoads hinzu.
     * @return liste von hinzugefuegten Kanten als Roads
     */
    public static LinkedList<Road> triangulate(SimpleGraph<Point, DefaultEdge> g) {
        // Punkte auslesen.
        int n = g.vertexSet().size();
        int m = g.edgeSet().size();
        
        if (n < 3) {
            return new LinkedList<Road>();
        }
        
        Object[] myObjects = g.vertexSet().toArray();
        Point[] vertices = new Point[myObjects.length];
        for (int i = 0; i < myObjects.length; i++) {
            vertices[i] = (Point) myObjects[i];
        }
        
        //Zuordnung Punkt->Variable
        HashMap<Point, Integer> vertexIndices = new HashMap<Point, Integer>();
        for (int i = 0; i < n; i++) {
            vertexIndices.put(vertices[i], i);
        }
        
        // triG muss nur berechnet werden, wenn trianguliert werden soll oder die orientierung beibehalten werden soll.
    	System.out.println("m: " + m + " | n: " + n); 
        // Punkte aus dem Graphen umwandeln.
        List<TriangulationPoint> triPoints = new LinkedList<TriangulationPoint>();
        
        HashMap<TPoint, Point> triMap = new HashMap<TPoint, Point>(); // um spaeter zum TPoint entspechenden Point zu finden
        
        for(int i = 0; i < n; i++) {
        	Point p = vertices[i];
        	TPoint triPoint = new TPoint(p.getX(), p.getY());
        	triPoints.add(i, triPoint);
        	triMap.put(triPoint, p);
        }
        
        int[] constrainedIndex = new int[2 * m]; // TEMP (Kante zwischen dem i. und dem i+1. Eintrag fuer i = 2k)S
        
        Iterator<DefaultEdge> itEdges = g.edgeSet().iterator();
        
        for(int i = 0; i < 2 * m && itEdges.hasNext(); i += 2) {
        	DefaultEdge e = itEdges.next();
        	Point source = g.getEdgeSource(e);
        	Point target = g.getEdgeTarget(e);
        	constrainedIndex[i] = vertexIndices.get(source);
        	constrainedIndex[i + 1] = vertexIndices.get(target);
        }
        
        // Punkte in eine Menge tun.
        ConstrainedPointSet triPointSet = new ConstrainedPointSet(triPoints, constrainedIndex);
        
        // Menge triangulieren
        try {
        	Poly2Tri.triangulate(triPointSet);
        }
        catch(Exception e) {
        	System.out.println("Triangulierung Fehlgeschlagen");
        	return new LinkedList<Road>();
        }
        
        
        //// Auslesen!
        
        // Zunaechst die Triangles:
        LinkedList<Road> triRoads = new LinkedList<Road>(); // fuer die Rueckgabe
        if (triPointSet.getTriangles() != null) {
            Iterator<DelaunayTriangle> trianglesIterator = triPointSet.getTriangles().iterator();
	        
            System.out.println("Anzahl Triangles: " + triPointSet.getTriangles().size());
	        
	        // Alle Triangles durchgehen:
	        
	        
	    while(trianglesIterator.hasNext()) {
	        DelaunayTriangle triangle = trianglesIterator.next();
	        	
	        // Zum TPoint den passenden Point finden (ueber die Map)
	        Point[] point = new Point[3];
	        for (int i = 0; i < 3; i++) {
	            point[i] = triMap.get(triangle.points[i]);
	        }
	        	
	        // Die drei Edges in das RoadNetwork einfuegen.
	        for (int i = 0; i < 3; i++) {
	            if (point[i].hashCode() <= point[(i+1) % 3].hashCode() 
	        	&& !point[i].equals(point[(i+1) % 3]) 
	        	&& !g.containsEdge(point[i], point[(i+1) % 3]) ) {
	        			
	        	// Kante in Graphen einfuegen.        			
	        	g.addEdge(point[i], point[(i+1) % 3]);
	        				        			
	        	// triedge ins TriRoadNetwork einfuegen (Nur zur Visualisierung)
	        	Vector<Point> pointsNew = new Vector<Point>();
	        	pointsNew.add(point[i]);
	        	pointsNew.add(point[(i+1) % 3]);
	                Road newRoad = new Road(pointsNew, "triedge", -1);
	                newRoad.setSymbol(RoadSymbol.TRI_EDGE);
	                triRoads.add(newRoad);
	        	}
	            }
	        }
            } 
        return triRoads;
    }
    
    public void addTriRoads() {
    	triRoads = triangulate(this.getAsGraph(null));

    }
    
    
    /**
     * Adds vertices to intersecting roads 
     */
    public PointSet addIntersectionPoints(PointSet snapPoints) {
        
        //generate edge list
        LinkedList<Edge> edges = new LinkedList<Edge>();      
        HashMap<Edge, Road> roadByEdge = new HashMap<Edge, Road>();
        
        for (Road r : roads) {
            Point pLast = r.getVertices().get(0);
            for (int i = 1; i < r.getVertices().size(); i++) {
                Point pNext = r.getVertices().get(i);
                Coordinate[] c = new Coordinate[2];
                c[0] = new Coordinate(pLast.getX(), pLast.getY());
                c[1] = new Coordinate(pNext.getX(), pNext.getY());
                Edge e = new Edge(c);
                
                edges.add(e);
                roadByEdge.put(e, r);
                
                pLast = pNext;
            }
        }
        
        //compute intersections
        SegmentIntersectionSweep myIntersector = new SegmentIntersectionSweep();
        HashMap<Coordinate, LinkedList<Edge>> myIntersections = myIntersector.segmentIntersection(edges);
                
        //iterate the intersections
        for (Map.Entry<Coordinate, LinkedList<Edge>> i : myIntersections.entrySet()) {
            Coordinate c = i.getKey();
            LinkedList<Edge> l = i.getValue();
            for (Edge e : l) {
                Road r = roadByEdge.get(e);
                
                //add a vertex to the road
                Point pNew = new Point(c.x, c.y);
                Point pOld = snapPoints.getNearestNeighbor(pNew, COORD_ERROR_TOLERANCE);
                Point p;
                if (pOld != null) {
                    p = pOld;
                } else {
                    p = pNew;
                    snapPoints.addPoint(p);
                }
                r.addPoint(p, pNew);
            }
        }
        return snapPoints;
    }
    
    
    /**
     * Method to generate a new RoadNetwork containing the roads of this RoadNetwork grouped into so-called strokes; 
     * a stroke is a sequence of roads in which for each two consecutive roads r1 and r2 it holds that
     * r1 and r2 end (or start) with the same vertex v and the angle alpha formed by r1 and r2 at v is close to pi  
     * @param angleTolerance the tolerance for the angle alpha: alpha has to be within pi - angleTolerance and pi + angleTolerance
     * @return the new road network
     */
    public RoadNetwork buildStrokes(double angleTolerance) {

        HashMap<Road, Road> newRoadByOldRoad = new HashMap<Road, Road>();
        HashMap<Road, LinkedList<Road>> oldRoadsByNewRoad = new HashMap<Road, LinkedList<Road>>();
        
        HashMap<Point, Vector<Road>> roadsByVertices = new HashMap<Point, Vector<Road>>();
        
        //iterate all roads
        for (Road r : roads) {
            
            //initially, define the new road equal to the old road
            newRoadByOldRoad.put(r, r);
            LinkedList<Road> l = new LinkedList<Road>();
            l.add(r);
            oldRoadsByNewRoad.put(r, l);
            
            
            //add map entry for first vertex of road 
            Point firstVertex = r.getVertices().firstElement();
            Vector<Road> roadsAtStart = roadsByVertices.get(firstVertex);
            if (roadsAtStart == null) {
                roadsAtStart = new Vector<Road>();
                roadsByVertices.put(firstVertex, roadsAtStart);
            }
            roadsAtStart.add(r);
            
            //add map entry for last vertex of road
            Point lastVertex = r.getVertices().lastElement();
            Vector<Road> roadsAtEnd = roadsByVertices.get(lastVertex);
            if (roadsAtEnd == null) {
                roadsAtEnd = new Vector<Road>();
                roadsByVertices.put(lastVertex, roadsAtEnd);
            }
            roadsAtEnd.add(r);
            
        }
        
        //iterate each vertex v of the road network
        //int counter = 0;
        for (Map.Entry<Point, Vector<Road>> e : roadsByVertices.entrySet()) {
                        
            Point v = e.getKey();
            //System.out.println("Vertex" + v.getX() + " " + v.getY());
            
            HashMap<Road, Road> bestMatches = new HashMap<Road, Road>();
            Vector<Road> myRoads = e.getValue();
            
            //test each pair of road segments incident to v to form a stroke
            for (int i = 0; i < myRoads.size(); i++) {
                Road r1 = myRoads.get(i);
                
                double minAngle = Double.POSITIVE_INFINITY;
                Road bestMatch = null;

                for (int j = 0; j < myRoads.size(); j++) {
                    if (i != j) {
                        Road r2 = myRoads.get(j);
                        if (r1.getType().equals(r2.getType())) {
                            double angle = r1.getAngleWith(r2, v);
                            if (angle < minAngle && angle <= angleTolerance) {
                                minAngle = angle;
                                bestMatch = r2;
                            }
                        }
                    }
                }
                bestMatches.put(r1, bestMatch);
            }
            
            //iterate pairs that were matched
            for(Map.Entry<Road, Road> match : bestMatches.entrySet()) {
                Road r1 = match.getKey();
                Road r2 = match.getValue();
                if(r1 != null && r2 != null && r1.getId() < r2.getId() && bestMatches.get(r2) == r1) {
                    Road newR1 = newRoadByOldRoad.get(r1);
                    Road newR2 = newRoadByOldRoad.get(r2);
                    
                    //merge matches
                    if (newR1 != newR2) {
                        //System.out.println("merge " + newR1 + " " + newR2);
                        Road mergedRoad = newR1.merge(newR2, v);
                        if (mergedRoad == null) {
                            System.out.println("error");
                            System.exit(0);
                        }
                        
                        LinkedList<Road> l = new LinkedList<Road>();  
                        for (Road r1_old : oldRoadsByNewRoad.get(newR1)) {
                            newRoadByOldRoad.put(r1_old, mergedRoad);
                            l.add(r1_old);
                        }
                        for (Road r2_old : oldRoadsByNewRoad.get(newR2)) {
                            newRoadByOldRoad.put(r2_old, mergedRoad);
                            l.add(r2_old);
                        }
                        oldRoadsByNewRoad.put(mergedRoad, l);
                    }
                }
            }
        }
        
        //create a set without duplicates
        HashSet<Road> mergedRoads = new HashSet<Road>();        
        for (Road r : newRoadByOldRoad.values()) if (r != null) mergedRoads.add(r);
        
        //create a new road network
        RoadNetwork merged = new RoadNetwork();
        merged.symbolSet = this.symbolSet;
        for (Road r : mergedRoads) {
            merged.add(r);
            r.setSymbol(RoadSymbol.getSymbolForClass(symbolSet, r.getType()));
        }
        return merged;
    } 

    
    public void reduceToLargestComponent() {
        SimpleGraph<Point, DefaultEdge> g = this.getAsGraph(this.getEnv());
        
      //Reduce the graph to the largest connected subgraph
        ConnectivityInspector<Point, DefaultEdge> myCI = new ConnectivityInspector<Point, DefaultEdge>(g);
        List<Set<Point>> myConnectedComponents = myCI.connectedSets();
        if (myConnectedComponents.size() <= 1)
        	return;
        System.out.println("Der Graph wird von " + myConnectedComponents.size() + " Zusammenhangskomponenten auf 1 reduziert.");
        int maxSize = 0; //the size of the largest component
        Set<Point> maxConnectedComponent = myConnectedComponents.get(0);
        for(Set<Point> myConnectedComponent : myConnectedComponents) {
            if (myConnectedComponent.size() > maxSize) {
                maxSize = myConnectedComponent.size();
                maxConnectedComponent = myConnectedComponent;
            }
        }
        for(Set<Point> myConnectedComponent : myConnectedComponents) {
            if (myConnectedComponent != maxConnectedComponent) {
                g.removeAllVertices(myConnectedComponent);
            }
        }
      
        Set<Point> pointSet = g.vertexSet();
        LinkedList<Road> roadsForRemoving = new LinkedList<Road>();
        
        for(Road r: roads) {
        	Point p = r.getVertices().firstElement();
        	if (!pointSet.contains(p))
        		roadsForRemoving.add(r);
        }
        roads.removeAll(roadsForRemoving);
        
        
    }
    
    /**
     * 
     * @param outEnv
     * @param outestEnv
     * @param pointsWithScaleConstraints
     * @param scaleLB
     * @param scaleUB
     * @param gewichtung
     * @param triangulation
     * @param orientierung
     * @param auswahl
     * @return
     */
    public RoadNetwork fit(Envelope outEnv, Envelope outestEnv, Point[] pointsWithScaleConstraints, double[] scaleLB, double[] scaleUB, boolean gewichtung, boolean edgeCrossing, boolean triangulation, boolean orientierung, boolean allOrientations, boolean auswahl, Statistic myStatistic) {
        return fit(outEnv, outestEnv, pointsWithScaleConstraints, scaleLB, scaleUB, gewichtung, edgeCrossing, triangulation, orientierung, allOrientations, auswahl, null);
    }
    
    public RoadNetwork fit(Envelope outEnv, Envelope outestEnv, Point[] pointsWithScaleConstraints, double[] scaleLB, double[] scaleUB, boolean gewichtung, boolean edgeCrossing, boolean triangulation, boolean orientierung, boolean allOrientations, boolean auswahl, Statistic myStatistic, FishEyeProjection fep) {
        
    	if (scaleLB != null && scaleLB.length > 0)
    		myStatistic.scaleValue = scaleLB[0];
    	else {
    		myStatistic.scaleValue = -1;
    	}
        HashSet<Point> focusRegion = new HashSet<Point>();
        for (Point p : pointsWithScaleConstraints) focusRegion.add(p);
        
        // Abstand fuer Intersection-Gerade:
        double epsilon = 0.1;
        // Abstand fuer Orientierungsbedingungen:
        double epsilon2 = 0.1;
        
    	// fuer auswahl:
    	double dicht = 200.0; // globale Dichtekonstante (TODOO: als Parameter uebernehmen)
    	double radius = 100.0;
    	
    	
    	// RoadNetwork in Graph umwandeln:
        SimpleGraph<Point, DefaultEdge> g = this.getAsGraph();
        final SimpleGraph<Point, DefaultEdge> gTri = this.getAsGraph();
        
        /*
      //Reduce the graph to the largest connected subgraph
        ConnectivityInspector<Point, DefaultEdge> myCI = new ConnectivityInspector<Point, DefaultEdge>(g);
        List<Set<Point>> myConnectedComponents = myCI.connectedSets();
        System.out.println("Der Graph hat " + myConnectedComponents.size() + " Zusammenhangskomponenten.");
        if (myConnectedComponents.size() > 1) {
        	this.reduceToLargestComponent();
        	return fit(outEnv, outestEnv, pointsWithScaleConstraints, scaleLB, scaleUB, gewichtung, edgeCrossing, triangulation, orientierung, allOrientations, auswahl, myStatistic, fep);
        }
       */
        
        
        

        //Punkte auslesen.
        int n = g.vertexSet().size();
        int m = g.edgeSet().size();
        int mTri;
        
        myStatistic.nodeCount = n;
        myStatistic.edgeCount = m;
        
        Point[] vertices = new Point[n];
	    {
	        int i = 0;
	        //System.out.println("Points");
	        for (Point p : g.vertexSet()) {
	            //System.out.println(p + " " + p.getX() + " " + p.getY());
	            vertices[i] = p;
	            i++;
	        }
	    }
        
        //Zuordnung Punkt->Variable
        HashMap<Point, Integer> vertexIndices = new HashMap<Point, Integer>();
        for (int i = 0; i < n; i++) {
            vertexIndices.put(vertices[i], i);
        }
        
        // triG muss nur berechnet werden, wenn trianguliert werden soll oder die orientierung beibehalten werden soll.
        if ((triangulation || orientierung) && n >= 3) {
            triangulate(gTri);
        	
		    // Kanten Anzahl hat sich durch die Triangulierung veraendert!
		    mTri = gTri.edgeSet().size();
		        
		    if(triangulation) { // Falls mit trianguliertem Graph gerechnet werden soll
		        g = gTri;
		        m = mTri;
		    }
        }
        
        try {
            IloCplex cplex = new IloCplex();
            
            //Abbruchtoleranz
            //cplex.setParam(IloCplex.DoubleParam.BarEpComp, 0.0000000001);
       
            String[] xNames = new String[n];
            String[] yNames = new String[n];
            String[] sNames = new String[n];
            for(int i = 0; i < xNames.length; i++) {
                xNames[i] = "x" + i;
                yNames[i] = "y" + i;
                sNames[i] = "s" + i;
            }
            
            String[] dxNames = new String[2 * m];
            String[] dyNames = new String[2 * m];
                            
            IloNumVar[] x = cplex.numVarArray(n, outEnv.getMinX(), outEnv.getMaxX(), xNames);
            IloNumVar[] y = cplex.numVarArray(n, outEnv.getMinY(), outEnv.getMaxY(), yNames);
            IloNumVar[] s = cplex.numVarArray(n, 0.0, Double.POSITIVE_INFINITY, sNames);
            IloNumVar[] dx = cplex.numVarArray(2 * m, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, dxNames);
            IloNumVar[] dy = cplex.numVarArray(2 * m, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, dyNames);
            
            for (int i = 0; i < n; i++) {
            	Point p = vertices[i];
            	if(!outEnv.contains(p.getX(), p.getY())) { // Falls ein Punkt nur im outestEnv ist.
            	    // Punkt fixieren.
            	    x[i] = cplex.numVar(p.getX(), p.getX(), "x" + i);
            	    y[i] = cplex.numVar(p.getY(), p.getY(), "y" + i);
            	}
            	
            	if (fep != null) { // Falls eine Fish-Eye-Projection angewandt werden soll
            	    //System.out.println("Fixiere Punkt");
            	    Point pNew = fep.projectPoint(p);
            	    x[i] = cplex.numVar(pNew.getX(), pNew.getX(), "x" + i);
            	    y[i] = cplex.numVar(pNew.getY(), pNew.getY(), "y" + i);
            	}
            }
            
            // Summanden fuer die Zielfunktion
            IloNumExpr[] zielSummandDx = new IloNumExpr[2 * m];
            IloNumExpr[] zielSummandDy = new IloNumExpr[2 * m];
            
            //Scale Constraints
            //System.out.println("Points with scale constraints:");
            for (int i = 0; i < pointsWithScaleConstraints.length; i++) {
                //System.out.println(pointsWithScaleConstraints[i] + " " + pointsWithScaleConstraints[i].getX() + " " + 
                //        pointsWithScaleConstraints[i].getY() + " " + scaleLB[i]);
                Integer pIndex = vertexIndices.get(pointsWithScaleConstraints[i]);
                if (pIndex != null) { // Knoten koennte beim rauswerfen einer Zusammenhangskomponente nicht mehr existieren.
                    //System.out.println("scale constraint added");
                    s[pIndex].setLB(scaleLB[i]);
                    s[pIndex].setUB(scaleUB[i]);
                    myStatistic.scaleNodeCount++;
                }
            }
            
            //Lokale Massstaeblichkeit
            HashMap<DefaultEdge, Integer> edgeIndices = new HashMap<DefaultEdge, Integer>();
            {int edgeIndex = 0;
            for (DefaultEdge e : g.edgeSet()) {
                Point p1 = (Point) g.getEdgeSource(e);
                Point p2 = (Point) g.getEdgeTarget(e);
                int p1Index = vertexIndices.get(p1);
                int p2Index = vertexIndices.get(p2);
                
                IloLinearNumExpr yExpr1 = cplex.linearNumExpr();
                yExpr1.addTerm(dy[edgeIndex], 1.0);
                yExpr1.addTerm(s[p1Index], p2.getY());
                yExpr1.addTerm(s[p1Index], -p1.getY());
                yExpr1.addTerm(y[p1Index], 1.0);
                yExpr1.addTerm(y[p2Index], -1.0);
                cplex.addEq(0.0, yExpr1);
                dy[edgeIndex].setName("dy_" + p1Index + "_" + p2Index);
                            
                IloLinearNumExpr yExpr2 = cplex.linearNumExpr();
                yExpr2.addTerm(dy[edgeIndex + 1], 1.0);
                yExpr2.addTerm(s[p2Index], p1.getY());
                yExpr2.addTerm(s[p2Index], -p2.getY());
                yExpr2.addTerm(y[p2Index], 1.0);
                yExpr2.addTerm(y[p1Index], -1.0);
                cplex.addEq(0.0, yExpr2);
                dy[edgeIndex + 1].setName("dy_" + p2Index + "_" + p1Index);
                
                IloLinearNumExpr xExpr1 = cplex.linearNumExpr();
                xExpr1.addTerm(dx[edgeIndex], 1.0);
                xExpr1.addTerm(s[p1Index], p2.getX());
                xExpr1.addTerm(s[p1Index], -p1.getX());
                xExpr1.addTerm(x[p1Index], 1.0);
                xExpr1.addTerm(x[p2Index], -1.0);
                cplex.addEq(0.0, xExpr1);
                dx[edgeIndex].setName("dx_" + p1Index + "_" + p2Index);
                
                IloLinearNumExpr xExpr2 = cplex.linearNumExpr();
                xExpr2.addTerm(dx[edgeIndex + 1], 1.0);
                xExpr2.addTerm(s[p2Index], p1.getX());
                xExpr2.addTerm(s[p2Index], -p2.getX());
                xExpr2.addTerm(x[p2Index], 1.0);
                xExpr2.addTerm(x[p1Index], -1.0);
                cplex.addEq(0.0, xExpr2);
                dx[edgeIndex + 1].setName("dx_" + p2Index + "_" + p1Index);

                // Punkte in der FocusRegion fixieren:
                
                if (fep == null && focusRegion.contains(p1) && focusRegion.contains(p2)) {
                    cplex.addEq(dy[edgeIndex], 0.0);     
                    cplex.addEq(dx[edgeIndex], 0.0);
                    cplex.addEq(dy[edgeIndex + 1], 0.0);
                    cplex.addEq(dx[edgeIndex + 1], 0.0);
                }
                
                /*if (fep == null && focusRegion.contains(p1)) {
                    cplex.addEq(dy[edgeIndex], 0.0);     
                    cplex.addEq(dx[edgeIndex], 0.0);
                }
                if (fep == null && focusRegion.contains(p2)) {
                    cplex.addEq(dy[edgeIndex + 1], 0.0);
                    cplex.addEq(dx[edgeIndex + 1], 0.0);
                }*/
                
                
                // Zielsummanden erzeugen: Fuer jede Kante:
                // Hinrichtung: dx[edgeIndex]^2*(1/laenge^2)
                // Rueckrichtung: dx[edgeIndex + 1]^2*(1/laenge^2)
                // analog dy wobei dessen Indizes sich zwischen 2*m und 4*m befinden.
                
                double deltaXdeltaX = (p1.getX() - p2.getX())*(p1.getX() - p2.getX());
                double deltaYdeltaY = (p1.getY() - p2.getY())*(p1.getY() - p2.getY());
                double laenge_2 = deltaXdeltaX + deltaYdeltaY;
                //double laenge = Math.sqrt(deltaXdeltaX + deltaYdeltaY);
                
                double gewicht;
                if(gewichtung) {
                    if (laenge_2 == 0) {
	               	System.out.println("Zwei Punkte liegen aufeinander!");
	               	gewicht = 1.0; // TEMP
                    } else {
                    	gewicht = 1.0 / laenge_2;
                    }	
                } else {
                    gewicht = 1.0;
            	}
                
              
                zielSummandDx[edgeIndex] = cplex.prod(gewicht, dx[edgeIndex], dx[edgeIndex]);
                zielSummandDx[edgeIndex + 1] = cplex.prod(gewicht, dx[edgeIndex + 1], dx[edgeIndex + 1]);
            	zielSummandDy[edgeIndex] = cplex.prod(gewicht, dy[edgeIndex], dy[edgeIndex]);
            	zielSummandDy[edgeIndex + 1] = cplex.prod(gewicht, dy[edgeIndex + 1], dy[edgeIndex + 1]);
                
            	edgeIndices.put(e, edgeIndex);
            	edgeIndex += 2;
            }}
            
            
            
            
            
            
            
            
            
            
            
            
            //// Auswahlbedingung Anfang ////
            List<Road> envRoads = new LinkedList<Road>(); // Die Strassen die eine Ecke im Graph (Envelope) haben.	
            IloNumVar[] z = cplex.boolVarArray(0);			
            if(auswahl) {
            	
            	for(int j = 0; j < roads.size(); j++) { // Alle Straßen durchgehen
                	for(Point p : roads.get(j).getVertices()) { // Alle Ecken der Strasse durchgehen.
                		if (outEnv.contains(p.getX(), p.getY())) { // Falls die Ecke im outEnv
                			if(!envRoads.contains(roads.get(j)))
                				envRoads.add(roads.get(j));
                			break; // Zur naechsten Strasse
                		}
                	}
                }
                z = cplex.boolVarArray(envRoads.size());
            	
            	// Dichte Bedingung:
            	IloLinearNumExpr Expr[] = new IloLinearNumExpr[n];
            	for(int i = 0; i < n; i++) {// Eine Bedingung pro Knoten
            		Point p = vertices[i];
            		if (!outEnv.contains(p.getX(), p.getY())) { // Falls knoten nicht im outEnv
            			continue;
            		}
            		Expr[i] = cplex.linearNumExpr(); // Sum(zj*d*dj)
            		
            		double gesamtFaktor = 0.0; // TEST
            		
	            	for(int j = 0; j < envRoads.size(); j++) { // Alle Strassen (im Graphen) durchgehen
	    	            
	    	            double indD = roads.get(j).roadLength(p,radius); // individuelle Dichtekonstante (TODOO: Kriteren festlegen)
	    	            if(indD != 0) {
	    	            	double faktor = dicht*indD/(radius*radius*Math.PI);
	    	            	Expr[i].addTerm(z[j], faktor); // Die Straße zur Bedingung des Knotens hinzufügen.
	    	            	System.out.println("Faktor von Knoten " + i + " und Road " + j + ": " + faktor);
	    	            	gesamtFaktor += faktor; // TEST
	    	            }
	            	}
	            	System.out.println("Gesamtfaktor von Knoten " + i + ": " + gesamtFaktor);
	            	cplex.addLe(Expr[i], s[i]); // e1 <= e2
            	}
            	//Relationsbedingung:
            	for(int i = 0; i < envRoads.size(); i++) {
            		for(int j = i+1; j < envRoads.size(); j++) {
            			Road road1 = envRoads.get(i);
            			Road road2 = envRoads.get(j);
        				boolean disjoint = true;
        				for (Point p : road1.getVertices()) {
        					if (road2.getVertices().contains(p)) {
        						disjoint = false;
        						break;
        					}
        				}
        				if (!disjoint) { //road1 und road2 haben gemeinsamen Punkt.
        					int c = road1.compareByLayer(road2); //compareByType(road2);
        					if(c > 0) {
        						cplex.addLe(z[i], z[j]); // e1 <= e2
        					}
        					else if(c < 0) {
        						cplex.addLe(z[j], z[i]); // e1 <= e2
        					}	
            			}
            		}
            	}  
            	System.out.println("Anzahl Integervariablen: " + envRoads.size());
            }
            ///// Auswahlbedingung Ende ////
            
            
            
            
            
            
            
            
            //// Zielfunktion ////
            
            // besteht aus vielen quadratischen Summanden (1/laenge^2 * dx[i]^2 bzw. 1/laenge^2 * dy[i+2*m]^2)
            // Die Abweichungen der Nachbarn sind also durch den Kehrwert der Laenge gewichtet.
            // Abweichungen gleicher Groesse (absolut) haben bei weiter entfernten Kanten weniger Gewicht!

            // Aufsummieren:
            IloNumExpr summeDx = cplex.sum(zielSummandDx);
            IloNumExpr summeDy = cplex.sum(zielSummandDy);
            
            IloNumExpr zielfunktion = cplex.sum(summeDx, summeDy);
            if(auswahl) {
	            IloLinearNumExpr strassen = cplex.linearNumExpr();         
	            for(int i = 1; i < envRoads.size(); i++) { // Alle Straßen im Envelope aufaddieren.
	            	double length = envRoads.get(i).getLength(); // Je laenger die strasse desdo hoeher die strafe.
	            	strassen.addTerm(z[i], -length/100.0); // TODOO Faktor festlegen
	            }
	            zielfunktion = cplex.sum(summeDx, summeDy, strassen);
            }
            
            
            // Als zu minimierende Funktion uebergeben:
            cplex.addMinimize(zielfunktion);
            
            //cplex.addMinimize(cplex.sum(cplex.scalProd(dx, dx), cplex.scalProd(dy, dy))); // Alte Version ohne Gewichtung!
            
            
           cplex.exportModel("problem.lp");
           
         
           //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
           //Loese das Problem
           //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
           boolean computeAgain = true;
           
           //boolean firstTime = true;
           HashMap<Point, Point> newPointOf = new HashMap<Point, Point>(); // Wird hier deklariert, damit nach mehrmaligem
           // berechnen hier die letzte Finale Abbildung gespeichert wird.
           // wird trivial initialisiert (wichtig falls allOrientations == true)
           for(Point p: g.vertexSet()) {
        	   newPointOf.put(p, p);
           }
           
           
           LinkedList<Edge> edges = new LinkedList<Edge>();
           while(computeAgain) {
        	   
        	   computeAgain = false;
        	   
        	   if(allOrientations && orientierung) { // Falls eh alle Orientierungsbedingung hinzugefuegt werden, dann jetzt.
        		   int orientationCount = Util.orientationConstraints(cplex, x, y, gTri, vertexIndices, newPointOf, epsilon2, false, true);
        		   myStatistic.addOrientation(orientationCount);
		           System.out.println("Added " + orientationCount + " orientation-constraints.");
        	   }
        	   
        	   if (fep == null)
        		   myStatistic.start();
        	   
	           boolean test = cplex.solve();
	           
	           if (fep == null)
	        	   myStatistic.stop();
	           
	           if(allOrientations && orientierung) { // Falls alle Orientierungsbedingungen direkt hinzugefuegt wurden, soll nicht nochmal gerechnet werden.
	        	   break;
	           }
	           
	           
	           if (test) {
	               System.out.println("Done.");
	           } else {
	               System.out.println("Problem not solved to optimality.");
	           }
	           cplex.writeSolution("solution.txt");
	           
	           
	           // Allen alten Punkten den neuen Punkt zuordnen:
	           for(Point p: g.vertexSet()) {
	        	   Integer index1 = vertexIndices.get(p);
                   if (index1 != null) {    
                       try {
                           double xNew = cplex.getValue(x[index1]);
                           double yNew = cplex.getValue(y[index1]); 
                           newPointOf.put(p, new Point(xNew, yNew));
                       } catch (Exception e) {}
                       
                   }
                   
	           }
	           
	           edges = new LinkedList<Edge>();
	           HashMap<Edge, Point[]> oldPointsOfEdge  = new HashMap<Edge, Point[]>();
	            
	            //add all new edges to list 
	            for (Road rOld : roads) {
	                Vector<Point> points = rOld.getVertices();
	                for(int i = 0; i < points.size() - 1; i++) {
	                    Point[] ePoints = new Point[2];
	                    
	                    //first point of edge
	                    ePoints[0] = points.get(i);
	                    double x1_new = newPointOf.get(ePoints[0]).getX();
	                    double y1_new = newPointOf.get(ePoints[0]).getY();
	                    
	                    //second point of edge
	                    ePoints[1] = points.get(i + 1);
	                    double x2_new = newPointOf.get(ePoints[1]).getX();
	                    double y2_new = newPointOf.get(ePoints[1]).getY();
	                    
	                    //create edge
	                    Coordinate[] c = new Coordinate[2];
	                    c[0] = new Coordinate(x1_new, y1_new);
	                    c[1] = new Coordinate(x2_new, y2_new);
	                    Edge e = new Edge(c);
	                    edges.add(e);
	                    
	                    //memorize old points
	                    oldPointsOfEdge.put(e, ePoints);
	                }
	            }
	           
	           if (fep != null || (edgeCrossing == false && orientierung == false)) break;
	           
	           //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	           // Teste Zulaessigkeit
	           //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	           
	           
	           ///// EdgeCrossing-Bedingung Anfang ////
	           if (edgeCrossing) {
		       		
	        	   HashMap<Coordinate, LinkedList<Edge>> myIntersections = Util.computeIntersections(edges, newPointOf);
		           
		           if (myIntersections.size() > 0) 
		               computeAgain = true;
		           
		           myStatistic.addConstraint(myIntersections.size());
		           
		           //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		           //add constraints for intersections
		           //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		           Util.addIntersectionConstraints(cplex, x, y, myIntersections, oldPointsOfEdge,vertexIndices, outEnv, epsilon);
		           
	           }
	           //// EdgeCrossing-Bedingung Ende ////
           
	           
	           
	           
	           
	           
	           //// Orientierungsbedingungen Anfang ////
	            if (orientierung) {
	            
	            	int orientationCount = Util.orientationConstraints(cplex, x, y, gTri, vertexIndices, newPointOf, epsilon2, false, false);
	            	
	            	if (orientationCount > 0)
	            		computeAgain = true;
		            myStatistic.addOrientation(orientationCount);
		            System.out.println("Added " + orientationCount + " orientation-constraints.");
	            } 
	            //// Orientierungsbedingung Ende /////
	            
	           
	            //firstTime = false;
           }
          
           //// Fuer Statistik die Crossings zaehlen: //////

           HashMap<Coordinate, LinkedList<Edge>> myIntersections = Util.computeIntersections(edges, newPointOf);
           
           myStatistic.crossingCount = myIntersections.size();
           
           
           
           //// Fuer Statistik Orientierungsaenderungen zaehlen ////
	       myStatistic.orientationChangedCount = Util.orientationConstraints(cplex, x, y, gTri, vertexIndices, newPointOf, epsilon2, true, false);
           
           
           myStatistic.optimalValue = cplex.getObjValue();
           
           
           //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
           //Konstruiere neues RoadNetwork
           //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                      
           int removedRoads = 0;
           RoadNetwork rnTransformed = new RoadNetwork();
           PointSet newPoints = new PointSet();
           HashMap<Point, Double> scales = new HashMap<Point, Double>();
           HashMap<Point, Point> newOldPointMap = new HashMap<Point, Point>(); // alten Punkt auf neuen abbilden (inklusive snap)
           
           for (Road rOld : roads) {
               if (auswahl && envRoads.contains(rOld)) {
                   if (cplex.getValue(z[envRoads.indexOf(rOld)]) <= 0.00001 ) {
                       removedRoads++;
                       continue;
                   }
               }
               
               
               Vector<Point> pointsOld = rOld.getVertices();
               Vector<Point> pointsNew = new Vector<Point>();
               for (Point pOld : pointsOld) {
                   Integer pOldIndex = vertexIndices.get(pOld);
                   if (pOldIndex != null) {
                       try {
                           Point pNew = new Point(cplex.getValue(x[pOldIndex]), cplex.getValue(y[pOldIndex]));
                           Point pSnap = newPoints.getNearestNeighbor(pNew, 0.0);
                           if (pSnap != null) {
                               //System.out.println("snap to point");
                               pNew = pSnap;
                           } else {
                               newPoints.addPoint(pNew);
                               scales.put(pNew, cplex.getValue(s[pOldIndex]));
                           }
                           pointsNew.add(pNew);
                           newOldPointMap.put(pOld, pNew);
                           
                           
                       } catch(Exception e) {
                           System.out.println("Punkt " + pOldIndex + " befindet sich nicht in cplex");
                           pointsNew.add(pOld); 
                       }
                   } else {
                      // System.out.println("old point added");
                       pointsNew.add(pOld);
                   }
               }
               if (!pointsNew.isEmpty()) {
                   Road rNew = new Road(pointsNew, rOld.getType(), rOld.getId());
                   rNew.setSymbol(rOld.getSymbol());
                   rnTransformed.add(rNew);
               }
           }
           if(auswahl)
        	   System.out.println(removedRoads + " roads removed.");
           
	       myStatistic.totalStop();
	       myStatistic.print();
	       myStatistic.printToFile();
	       
	       LinkedList<LineString> residuenLines = new LinkedList<LineString>();
	       HashMap<LineString, Double> residuen = new HashMap<LineString, Double>();
	       GeometryFactory gf = new GeometryFactory();
	       
	       double sumRes = 0;
	       
	       for (DefaultEdge e : g.edgeSet()) {
	    	   
	    	   Point pOld1 = (Point) g.getEdgeSource(e);
               Point pOld2 = (Point) g.getEdgeTarget(e);
	    	   
               Point p1 = newOldPointMap.get(pOld1);
               Point p2 = newOldPointMap.get(pOld2);
               
               
               
               int edgeIndex = edgeIndices.get(e);
               Coordinate middle = new Coordinate((p1.getX() + p2.getX())*0.5, (p1.getY() + p2.getY())*0.5);
               
               // Gewicht berechnen:
               double deltaXdeltaX = (pOld1.getX() - pOld2.getX())*(pOld1.getX() - pOld2.getX());
               double deltaYdeltaY = (pOld1.getY() - pOld2.getY())*(pOld1.getY() - pOld2.getY());
               double laenge_2 = deltaXdeltaX + deltaYdeltaY;
               //double laenge = Math.sqrt(deltaXdeltaX + deltaYdeltaY);
               
               double gewicht;
               if(gewichtung) {
                   if (laenge_2 == 0) {
	               	System.out.println("Zwei Punkte liegen aufeinander!");
	               	gewicht = 1.0; // TEMP
                   } else {
                   	gewicht = 1.0 / laenge_2;
                   }	
               } else {
                   gewicht = 1.0;
           		}

               
               // Abschnitt A (edgeIndex)
               Coordinate[] coordA = new Coordinate[2];
               coordA[0] = new Coordinate(p1.getX(), p1.getY());
               coordA[1] = middle;

               LineString lineA = gf.createLineString(coordA);
               
               residuenLines.add(lineA);
               double dxA = cplex.getValue(dx[edgeIndex]);
               double dyA = cplex.getValue(dy[edgeIndex]);
               double resA = gewicht * (dxA*dxA + dyA*dyA);
               
               residuen.put(lineA, resA);
               
               
               // Abschnitt B (edgeIndex+1)
               Coordinate[] coordB = new Coordinate[2];
               coordB[0] = new Coordinate(p2.getX(), p2.getY());
               coordB[1] = middle;

               LineString lineB = gf.createLineString(coordB);
               
               residuenLines.add(lineB);
               
               double dxB = cplex.getValue(dx[edgeIndex+1]);
               double dyB = cplex.getValue(dy[edgeIndex+1]);
               double resB = gewicht * (dxB*dxB + dyB*dyB);
               
               residuen.put(lineB, resB);
               
               
               sumRes += resA + resB;
	       }
	       
	       System.out.println("Summe der Residuen: " + sumRes);
	       
	       Util.ExportResiduen(residuenLines, residuen);
	       
	       Util.ExportScalePointMap(newPoints.getAsList(), scales);
	        
	       return rnTransformed;
        } catch (IloException e) {
            e.printStackTrace();
        }
        return null;
    }
    
    

  
    public double getMinX() {
        return minX;
    }

    public double getMinY() {
        return minY;
    }

    public double getMaxX() {
        return maxX;
    }

    public double getMaxY() {
        return maxY;
    }    
    
    public void setName(String name) {
    	this.name = name;
    }
    
    public String getName() {
    	return name;
    }

    public Envelope getEnv() {
        return new Envelope(minX, maxX, minY, maxY);
    }
/*
    public RoadNetwork project(Envelope env, double s, double r0, Point center) {
        
        double xMid = center.getX();
        double yMid = center.getY();
        
        double r1 = env.getMaxX() - xMid;
        if (env.getMaxY() - yMid < r1) r1 = env.getMaxY() - yMid; 
        if (xMid - env.getMinX() < r1) r1 = xMid - env.getMinX();
        if (yMid - env.getMinY() < r1) r1 = yMid - env.getMinY();
        
        RoadNetwork rnNew = new RoadNetwork();
        
        for(Road r : roads) {
            Vector<Point> points = r.getVertices();
            Vector<Point> pointsNew = new Vector<Point>(); 
            for (Point p : points) {
                double dx = p.getX() - xMid;
                double dy = p.getY() - yMid;
                double rOld = Math.sqrt(dx * dx + dy * dy);
                double rNew;
                if (rOld <= r0) {
                    rNew = s * rOld;
                } else if (r0 < rOld && rOld <= r1) {
                    rNew = s * r0 + (rOld - r0) * (r1 - s * r0) / (r1 - r0);
                } else {
                    rNew = rOld;
                }
                
                double dxNew = dx * rNew / rOld;
                double dyNew = dy * rNew / rOld;
                pointsNew.add(new Point(xMid + dxNew, yMid + dyNew));
            }
            Road rNew = new Road(pointsNew, r.getType(), r.getId());
            rNew.setSymbol(r.getSymbol());
            
            rnNew.add(rNew);
        }
        return rnNew;
    }
    
    public RoadNetwork projectHarrie(Envelope env) {
        
        double xMid = (env.getMaxX() + env.getMinX()) / 2.0;
        double yMid = (env.getMaxY() + env.getMinY()) / 2.0;
        
        double r0 = 0.5;
        double r1 = Math.sqrt((env.getMaxX() - xMid) * (env.getMaxX() - xMid) + (env.getMaxY() - yMid) * (env.getMaxY() - yMid)) - 200 ;
        
        
        double sL = 1.0;
        double sS = 0.01;
        
        RoadNetwork rnNew = new RoadNetwork();
        
        for(Road r : roads) {
            Vector<Point> points = r.getVertices();
            Vector<Point> pointsNew = new Vector<Point>(); 
            for (Point p : points) {
                double dx = p.getX() - xMid;
                double dy = p.getY() - yMid;
                double rOld = Math.sqrt(dx * dx + dy * dy);
                double rNew;
                if (rOld <= r0) {
                    rNew = rOld;
                } else if (r0 < rOld && rOld <= r1) {
                    rNew = rOld + (rOld - r0) * (rOld - r0) * (sS - sL) / (2 * sL * (r1 - r0));
                } else {
                    rNew = r1 + (r1 - r0) * (sS - sL) / (2 * sL) + (rOld - r1) + sS / sL;
                }
                
                double dxNew = dx * rNew / rOld;
                double dyNew = dy * rNew / rOld;
                pointsNew.add(new Point(xMid + dxNew, yMid + dyNew));
            }
            Road rNew = new Road(pointsNew, r.getType(), r.getId());
            rNew.setSymbol(r.getSymbol());
            
            rnNew.add(rNew);
        }
        return rnNew;
    }
    */
  
}
