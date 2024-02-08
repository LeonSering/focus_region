package focusmap;


import java.io.File;
import java.util.LinkedList;
import java.util.Vector;

import javax.swing.JFileChooser;
import javax.swing.JScrollPane;

/**
 * Das Map Fenster
 * @author Jan Haunert
 *
 */
public class MapFrame extends JScrollPane {
    
    private MapDisplay myDisplay;
    //private MainFrame myMainFrame;
    private RoadNetwork myRoadNetwork;
    private static final long serialVersionUID = 1L;

    
    public MapFrame(MainFrame mf, RoadNetwork rn) {

    	setVerticalScrollBarPolicy(VERTICAL_SCROLLBAR_ALWAYS);
    	setHorizontalScrollBarPolicy(HORIZONTAL_SCROLLBAR_ALWAYS);
    	//rn.reduceToLargestComponent();
		//myMainFrame = mf;
        myRoadNetwork = rn;
        //setLayout(new BorderLayout());

        
        myDisplay = new MapDisplay(mf, this);
        myDisplay.setFrameRatio(0.1);
        
        //add the roads to the map
        LinkedList<Road> myRoads = rn.getRoads();
        for (Road r : myRoads) {
            Vector<Point> vertices = r.getVertices();
            RoadSymbol rs = r.getSymbol();
            for (int i = 0; i < vertices.size() - 1; i++) {
                Point p1 = vertices.get(i);
                Point p2 = vertices.get(i + 1);
                //String s = r.getType();
                LineObject ls = new LineObject(p1, p2,
                        rs.getWidth(), (float) 1.0,
                        rs.getStrokeColor(), rs.getColor());
                myDisplay.add(ls, rs.getLayer());
            }
        }
        
        if (rn.getTriRoads() != null) {
	        LinkedList<Road> myTriRoads = rn.getTriRoads();
	        for (Road r : myTriRoads) {
	            Vector<Point> vertices = r.getVertices();
	            RoadSymbol rs = r.getSymbol();
	            for (int i = 0; i < vertices.size() - 1; i++) {
	                Point p1 = vertices.get(i);
	                Point p2 = vertices.get(i + 1);
	                //String s = r.getType();
	                LineObject ls = new LineObject(p1, p2,
	                        rs.getWidth(), (float) 1.0,
	                        rs.getStrokeColor(), rs.getColor());
	                myDisplay.add(ls, rs.getLayer());
	            }
	        }
        }
        
        this.setViewportView(myDisplay);
        myDisplay.fitMapToDisplay(); // TODO
        
        
    }    
	
    public void exportParameter() {
        JFileChooser fc = new JFileChooser();
        File dir = new File("GIS_data");        
        fc.setCurrentDirectory(dir);
        fc.setFileFilter(new RoadNetwork.SHPFileFilter());
        int returnVal = fc.showSaveDialog(this);
        if(returnVal == JFileChooser.APPROVE_OPTION) {
            String filename = fc.getSelectedFile().getPath();
            Statement st = myDisplay.getStatement();
            st.exportToShapefile(filename);
        }
    }
	
    public void importParameter() {
        JFileChooser fc = new JFileChooser();
        File dir = new File("GIS_data");        
        fc.setCurrentDirectory(dir);
        fc.setFileFilter(new RoadNetwork.SHPFileFilter());
        int returnVal = fc.showOpenDialog(this);
        if(returnVal == JFileChooser.APPROVE_OPTION) {
            String filename = fc.getSelectedFile().getPath();
            Statement st = Statement.importFromShapefile(filename);
	       
            myDisplay.setStatement(st);
        }
    }
    
    public RoadNetwork getRoadNetwork() {
        return myRoadNetwork;
    }
    
    public void berechnen() {
    	myDisplay.optimize();
    }
    
    public MapDisplay getMapDisplay() {
    	return myDisplay;
    }

    public void project() {
        myDisplay.project();        
    }


    
}
