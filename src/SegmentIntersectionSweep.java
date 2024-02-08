package focusmap;

import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;

import com.vividsolutions.jts.algorithm.RobustLineIntersector;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geomgraph.Edge;
import com.vividsolutions.jts.geomgraph.EdgeIntersection;
import com.vividsolutions.jts.geomgraph.EdgeIntersectionList;
import com.vividsolutions.jts.geomgraph.index.SegmentIntersector;
import com.vividsolutions.jts.geomgraph.index.SimpleSweepLineIntersector;

public class SegmentIntersectionSweep {
    public SegmentIntersectionSweep() {
        
    } 
    public HashMap<Coordinate, LinkedList<Edge>> segmentIntersection(LinkedList<Edge> edges) {
                
        //compute edge intersections
        RobustLineIntersector li = new RobustLineIntersector();
        SegmentIntersector si = new SegmentIntersector(li, true, false);
        SimpleSweepLineIntersector intersector = new SimpleSweepLineIntersector();
        intersector.computeIntersections(edges, si, false);
        
        //set up hashtable that contains for each intersection the supporting edges         
        HashMap<Coordinate, LinkedList<Edge>> edgesByCoordinate  = new HashMap<Coordinate, LinkedList<Edge>>();
        for (Edge e : edges) {
            EdgeIntersectionList intersections = e.getEdgeIntersectionList();
            Iterator i = intersections.iterator();
            
            //iterate intersections with e
            while(i.hasNext()) {
                EdgeIntersection intersection = (EdgeIntersection) i.next();
                
                //is it an interior intersection?
                if (!intersection.getCoordinate().equals(e.getCoordinate(0)) && !intersection.getCoordinate().equals(e.getCoordinate(1))) {
                    LinkedList<Edge> supportingEdges = edgesByCoordinate.get(intersection.coord);
                    //if e is the first edge supporting this intersection
                    if(supportingEdges == null) {
                        supportingEdges = new LinkedList<Edge>();
                        edgesByCoordinate.put(intersection.coord, supportingEdges);
                    }
                    supportingEdges.add(e);
                }
            }
        }
        return edgesByCoordinate;
    }
}
