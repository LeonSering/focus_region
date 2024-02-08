package focusmap;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import java.awt.BasicStroke;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.Rectangle;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
//import java.util.LinkedList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.TreeMap;
//import java.util.Vector;

import com.vividsolutions.jts.geom.Envelope;
/**
 * This class represents a simple GUI component that allows MapObjects to
 * be displayed.
 * @author Jan-Henrik Haunert
 */
public class MapDisplay extends JPanel implements MouseListener, MouseMotionListener, MouseWheelListener {

    /**
     * The class JComponent is serializable, so we need a serialVersionUID.
     */
    private static final long serialVersionUID = 1L;
    
    /**
     * If frameRation > 0, a margin is left between the map and the frame of the display. 
     */
    private double frameRatio;
    
    /**
     * The Vector of MapObjects that are displayed in this MapDisplay.
     */
    private TreeMap<Integer, Layer> layers;
    
    /**
     * The transformation that is used to transform map coordinates into image coordinates.
     */
    private Transformation myTransformation;
    
    /**
     * Gibt an, ob die Mouse im Scale-Modus gedrueckt ist.
     */
    private boolean mouseIsPressed;
    /**
     * The column of the mouse cursor when the mouse was pressed the last time.
     */
    private int mouseColumn;
    
    private int mouseColumnTemp;
    
    /**
     * The row of the mouse cursor when the mouse was pressed the last time.
     */
    private int mouseRow;
    
    private int mouseRowTemp;
    
    /**
     * The minimum x coordinate of the map.
     */
    private double xMin;
    
    /**
     * The minimum y coordinate of the map.
     */
    private double yMin;
    
    /**
     * The maximum x coordinate of the map.
     */
    private double xMax;
    
    /**
     * The maximum y coordinate of the map.
     */
    private double yMax;
    
    /**
     * The MapFrame containing this MapDisplay.
     */
    private MapFrame myMapFrame;
    
    /**
     * The MainFrame containing the Controlelements
     */
    private MainFrame myMainFrame;
        
    /**
     * Das sichtbare Rechteck der Panels, zum Zeitpunkt wenn die Maus gedrueckt wird.
     */
    private Rectangle visRec;
    
    /**
     * Die ausgewaehlten Rechtecke die beim Klick auf Berechnen berechnet werden.
     */
    private Statement currentStatement = null;
    
    /**
     * This constructor generates a new empty MapDisplay.
     */
    
    //private RoadNetwork myRoadNetwork;
    
    public MapDisplay(MainFrame mainF, MapFrame mapF) {
        super();
        myMapFrame = mapF;
        myMainFrame = mainF;
        
        
        /*Envelope env = myMapFrame.getRoadNetwork().getEnv();
        double x = (env.getMaxX() + env.getMinX()) * 0.5;
        double y = (env.getMaxY() + env.getMinY()) * 0.5;
        currentStatement = new Statement(2.0, new Point(x - 150, y - 150), new Point(x + 150, y + 150));
        */
        //myRoadNetwork = myMapFrame.getRoadNetwork();
        myTransformation = new Transformation();
        
        layers = new TreeMap<Integer, Layer>();
        
        xMin = Double.POSITIVE_INFINITY;     
        yMin = Double.POSITIVE_INFINITY;
        xMax = Double.NEGATIVE_INFINITY;
        yMax = Double.NEGATIVE_INFINITY;
                 
        addComponentListener(new ComponentAdapter(){
        	public void componentResized(ComponentEvent evt) {
        			fitMapToDisplay();
        		}});
        
        addMouseListener(this);
        addMouseMotionListener(this);
        addMouseWheelListener(this);
        
        
    }
        
    
    
    
    
    /**
     * Redefines the transformation that transforms map coordinates to
     * image coordinates.
     * @param t the new transformation
     */
    public void setTransformation(Transformation t) {
        myTransformation = t;
    }
    
    /**
     * Returns the transformation that transforms map coordinates to
     * image coordinates.
     * @return the transformation
     */
    public Transformation getTransformation() {
        return myTransformation;
    }
    
    /**
     * Defines the margin between the map and the frame of this
     * map display.
     * @param ratio the margin relative to the size of the frame.
     */
    public void setFrameRatio(double ratio) {
        frameRatio = ratio;
    }
    
    /**
     * Adds a MapObject to the display on the specified layer.
     * @param mapObject the map object to be added
     * @param layerID the id of the layer
     */
    public void add(MapObject mapObject, int layerID) {
        Layer l = layers.get(layerID);
        if (l == null) {
            l = new Layer(layerID);
            layers.put(layerID, l);
        }
        l.add(mapObject);
                
        Envelope e = mapObject.getBoundingBox();
        if(e.getMinX() < xMin) xMin = e.getMinX();    
        if(e.getMinY() < yMin) yMin = e.getMinY();
        if(e.getMaxX() > xMax) xMax = e.getMaxX();
        if(e.getMaxY() > yMax) yMax = e.getMaxY();
        
    }
    
    public Statement getStatement() {
        return currentStatement;
    }
    
      
    /**
     * Draws all MapObject (first their background contents, then their foreground contents).
     */
    public void paint(Graphics gSimple) {
        Graphics2D g = (Graphics2D) gSimple;
                
        double x1 = myTransformation.getX(0);
        double y1 = myTransformation.getY(this.getHeight());
        double x2 = myTransformation.getX(this.getWidth());;
        double y2 = myTransformation.getY(0);
        
        Envelope e = new Envelope(x1, x2, y1, y2);
        
        g.clearRect(0, 0, this.getWidth(), this.getHeight());        
        for (int i : layers.keySet()) {
        	if (i == -1 && !myMainFrame.getViewTriangulationState()) // triedges
        		continue;
        	Layer l = layers.get(i);
            for (Object o : l.query(e)) {
                MapObject mo = (MapObject) o; 
                mo.drawBackground(g, myTransformation);
            }
        }
        for (Layer l : layers.values()) {
            for (Object o : l.query(e)) {
                MapObject mo = (MapObject) o; 
                mo.drawForeground(g, myTransformation);
            }
        }

        
        if ( myMainFrame.isInScaleOptimizerMode()) { 
        	if (mouseIsPressed) {
        		// Das aufgezogene Rechteck/Kreis:
        		Envelope currentEnv = new Envelope(mouseColumn, mouseColumnTemp, mouseRow, mouseRowTemp);
        		g.setColor(new Color(100, 0, 0));
                g.setStroke(new BasicStroke(3));
        		
        		if (myMainFrame.getCircleState()) { // Es handelt sich um einen Kreis
    				Point center = Statement.getCenter(currentEnv);
    				int r = (int)Statement.getRadius(currentEnv);
    				g.drawOval((int)center.getX() -r/2, (int)center.getY()-r/2, r, r);
    				g.setColor(new Color(100, 100, 100));
                    g.setStroke(new BasicStroke(1));
    				g.drawRect(Util.min(mouseColumn, mouseColumnTemp), Util.min(mouseRow, mouseRowTemp), Util.abs(mouseColumnTemp - mouseColumn), Util.abs(mouseRowTemp - mouseRow));

        		} else {
	                
	            	g.drawRect(Util.min(mouseColumn, mouseColumnTemp), Util.min(mouseRow, mouseRowTemp), Util.abs(mouseColumnTemp - mouseColumn), Util.abs(mouseRowTemp - mouseRow));
        	
        		}
        	}
        	// Das Statement anzeigen, falls Vorhanden
        	if (this.currentStatement != null && (!mouseIsPressed || myMainFrame.getEnvAddState())) {
        		//System.out.println("Statement wird gezeichnet!");
        		
        		// Außersten Rahmen anzeigen:
        		/*g.setStroke(new BasicStroke(1));
        		g.setColor(new Color(100, 100, 100));
        		Envelope outest = this.currentStatement.getOutestEnvelope();
        		int outestMoutestX = myTransformation.getColumn(outest.getMinX());
        		int outestMoutestY = myTransformation.getRow(outest.getMinY());
        		int outestMaxX = myTransformation.getColumn(outest.getMaxX());
        		int outestMaxY = myTransformation.getRow(outest.getMaxY());
        		g.drawRect(Util.min(outestMoutestX, outestMaxX), Util.min(outestMoutestY, outestMaxY), Util.abs(outestMaxX-outestMoutestX), Util.abs(outestMoutestY-outestMaxY));        		
        		*/
        		g.setStroke(new BasicStroke(3));
        		g.setColor(new Color(100, 100, 100));
        		Envelope out = this.currentStatement.getOutEnv();
        		int outMoutX = myTransformation.getColumn(out.getMinX());
        		int outMoutY = myTransformation.getRow(out.getMinY());
        		int outMaxX = myTransformation.getColumn(out.getMaxX());
        		int outMaxY = myTransformation.getRow(out.getMaxY());
        		g.drawRect(Util.min(outMoutX, outMaxX), Util.min(outMoutY, outMaxY), Util.abs(outMaxX-outMoutX), Util.abs(outMoutY-outMaxY));

        		g.setColor(new Color(100, 0, 0));
                g.setStroke(new BasicStroke(3));
        		LinkedList<Envelope> inEnvs = this.currentStatement.getInEnvs();
        		for(Envelope in: inEnvs) {
        			if (currentStatement.isCircle(in)) { // Es handelt sich um einen Kreis
        				Point center = Statement.getCenter(in);
        				int r = myTransformation.getLength(Statement.getRadius(in));
        				g.drawOval(myTransformation.getColumn(center.getX()) -r/2, myTransformation.getRow(center.getY())-r/2, r, r);
        				
        			} else { // Es handelt sich um ein Rechteck.
		        		int inMinX = myTransformation.getColumn(in.getMinX());
		        		int inMinY = myTransformation.getRow(in.getMinY());
		        		int inMaxX = myTransformation.getColumn(in.getMaxX());
		        		int inMaxY = myTransformation.getRow(in.getMaxY());
		        		g.drawRect(Util.min(inMinX, inMaxX), Util.min(inMinY, inMaxY), Util.abs(inMaxX-inMinX), Util.abs(inMinY-inMaxY));
	        		}
        		}
        	}
        }
    }
    
    /**
     * Fits the map to this MapDisplay.
     */
    public void fitMapToDisplay() {
        fitBoxToDisplay(xMin, yMin, xMax, yMax);
    }

    /**
     * Fits a part of the map to the MapDisplay, 
     * specified by its coordinate bounds.
     * @param xMinBox the minimum x coordinate
     * @param yMinBox the minimum y coordinate
     * @param xMaxBox the maximum x coordinate
     * @param yMaxBox the maximum y coordinate
     */
    public void fitBoxToDisplay(double xMinBox, double yMinBox, double xMaxBox, double yMaxBox) {
        double dx = frameRatio * (xMaxBox - xMinBox);
        double dy = frameRatio * (yMaxBox - yMinBox);
        xMinBox = xMinBox - dx;
        xMaxBox = xMaxBox + dx;
        yMinBox = yMinBox - dy;
        yMaxBox = yMaxBox + dy;
            
        int mapWidth = this.getSize().width;
        int mapHeight = this.getSize().height;
            
        if(xMaxBox == xMinBox && yMaxBox == yMinBox) {
            xMaxBox += 10.0;
            yMaxBox += 10.0;
            xMinBox -= 10.0;
            yMinBox -= 10.0;
        } else if(xMaxBox == xMinBox) {
            xMaxBox += 0.01 * (yMaxBox-yMinBox);
            xMinBox -= 0.01 * (yMaxBox-yMinBox);
        } else if(yMaxBox == yMinBox) {
            yMaxBox += 0.01 * (xMaxBox-xMinBox);
            yMinBox -= 0.01 * (xMaxBox-xMinBox);
        }
  
        double m1 = mapWidth / (xMaxBox - xMinBox);
        double m2 = mapHeight / (yMaxBox - yMinBox);
        double m;
        int frameSizeX =0;
        int frameSizeY =0;
                  
        if(m1 < m2) {
            m = m1;
            frameSizeY = (int)(0.5 * (mapHeight - m * (yMaxBox - yMinBox)));
        } else {
            m = m2;
            frameSizeX = (int)(0.5 * (mapWidth - m * (xMaxBox - xMinBox)));
        }
        int ColumnOrigin = frameSizeX - (int)(m * xMinBox);
        int RowOrigin    = frameSizeY + (int)(m * yMaxBox);
    
        myTransformation = new Transformation(m, ColumnOrigin, RowOrigin);
        repaint();
    }
    
    /*
    private void initSize() {
    	double dx = xMax - xMin;
    	double dy = yMax - yMin;
    	
    	// Sollte eins 0 sein, wird das Panel Quadratisch:
    	if (dx <= 0 || dy <= 0) {
    		dx = 1;
    		dy = 1;
    	}
    	
    	double verhaeltnis = dy / dx;
    	

    	double visWidth = myMainFrame.BREITE;
    	double visHeight = myMainFrame.HOEHE;
    	System.out.println(visWidth + ", " + visHeight);
    	double width = 0;
    	double height = 0;
    	if (verhaeltnis < visHeight / visWidth) {
    		height = visHeight;
    		width = 1/verhaeltnis * visHeight;
    	} else {
    		width = visWidth;
			width = verhaeltnis * visWidth;
    	}

    	
        setPreferredSize(new Dimension((int)width,(int)height));
        setSize(new Dimension((int)width,(int)height));
    	myMapFrame.revalidate();
    }*/
    
    private void zoom(double zoom, double mausX, double mausY) {
        /*int row = e.getY();
        int column = e.getX();
        double xCenter = myTransformation.getX(column);
        double yCenter = myTransformation.getY(row);
            if (e.getButton() == MouseEvent.BUTTON1) { 
                myTransformation.setM(2.0 * myTransformation.getM());
            } else {
                myTransformation.setM(0.5 * myTransformation.getM());
            }
        int rowTemp = myTransformation.getRow(yCenter);
        int columnTemp = myTransformation.getColumn(xCenter);
        myTransformation.setColumnOrigin(myTransformation.getColumnOrigin() + column - columnTemp);
        myTransformation.setRowOrigin(myTransformation.getRowOrigin() + row - rowTemp);
        repaint();*/


    	Rectangle rec = getVisibleRect();
    	
    	// Karte skalieren:

    	int width = (int)(this.getSize().getHeight() * zoom);
    	int height = (int)(this.getSize().getHeight() * zoom);

    	this.setVisible(false); // Bis richtig gescrollt wurde, wird das MapDisplay invisible, um Spruenge beim Scrollen zu vermeiden. 
        setPreferredSize(new Dimension(width,height));
        setSize(new Dimension(width,height));

    	//myMapFrame.revalidate();
    	
    	// Karte an die richtige Stelle scrollen:
    	
    	
    	mausX = mausX * zoom; // Mausposition (beim Klicken) (zur Map) hat sich wegen Zoom veraendert.
    	mausY = mausY * zoom; // Mausposition ist Zentrum des sichtbaren Rechtecks.
    	
    	int x = (int)(mausX-rec.getWidth()*0.5); // linke obere Ecke des sichtbaren Rechtecks.
    	int y = (int)(mausY-rec.getHeight()*0.5);
    	
    	rec.setLocation(x, y);

    	//scrollRectToVisible(rec);
    	
    	
    	// Damit nicht zu frueh richtig gescrollt wird, wird das erst gemacht nachdem die Karte richtig scalliert wurde (passiert in anderem Thread)
    	class ViewPositionSetter implements Runnable{
			MapDisplay mapDisplay;
			Rectangle rec;
			public ViewPositionSetter(MapDisplay mapDisplay,Rectangle rec){
				this.mapDisplay=mapDisplay;
				this.rec=rec;
			}
			public void run(){
				mapDisplay.setVisible(true);
				mapDisplay.scrollRectToVisible(rec);
			}
		}
		SwingUtilities.invokeLater(new ViewPositionSetter(this,rec));
    }

    @Override
    public void mouseClicked(MouseEvent e) {
        if ( myMainFrame.isInZoomPanMode() || (e.getButton() != MouseEvent.BUTTON1 && e.getButton() != MouseEvent.BUTTON3)) {
        	
        	double zoom = 1;
        	if (e.getButton() == MouseEvent.BUTTON1) {
        		zoom = myMainFrame.getZoomFactor();
        	} else if (e.getButton() == MouseEvent.BUTTON3){
        		zoom = 1/myMainFrame.getZoomFactor(); 		
        	}
        	
        	zoom(zoom, e.getX(), e.getY());
        	
        	
        } else if ( myMainFrame.isInScaleOptimizerMode() ) {
                
        }
    }

    @Override
    public void mouseEntered(MouseEvent e) {
    }

    @Override
    public void mouseExited(MouseEvent e) {
    }

    @Override
    public void mousePressed(MouseEvent e) {
        mouseColumn = e.getX();
        mouseRow = e.getY();
        visRec = getVisibleRect();
        if ( myMainFrame.isInScaleOptimizerMode() && e.getButton() == MouseEvent.BUTTON1) {
        		mouseIsPressed = true;
        		//this.currentStatement = null;
        		//System.out.println("Statement reset!");
        }
    }

    @Override
    public void mouseReleased(MouseEvent e) {
    	
        if ( myMainFrame.isInZoomPanMode() || (e.getButton() != MouseEvent.BUTTON1 && e.getButton() != MouseEvent.BUTTON3)) {
        	// Sprunghafte Bewegung ersetzt durch weiche Bewegung.
        	/*Rectangle rec = new Rectangle(visRec);
        	
    		double x = visRec.getLocation().getX();
    		double y = visRec.getLocation().getY();
    		
    		x = x + mouseColumn - e.getX();
    		y = y + mouseRow - e.getY();
        	rec.setLocation((int)x,(int)y);
        	scrollRectToVisible(rec);*/
        } else if ( myMainFrame.isInScaleOptimizerMode() && e.getButton() == MouseEvent.BUTTON1) {
        	mouseIsPressed = false;
            mouseColumnTemp = e.getX();
            mouseRowTemp = e.getY();
            
            // Statement setzen:
            
            // inneres Rechteck:
            
            Point punkt1 = new Point(myTransformation.getX(mouseColumn), myTransformation.getY(mouseRow));
            Point punkt2 = new Point(myTransformation.getX(mouseColumnTemp), myTransformation.getY(mouseRowTemp));
            if (myMainFrame.getEnvAddState() == false || currentStatement == null) {
            	currentStatement = new Statement(myMainFrame.getZoomFactor(), punkt1, punkt2, myMainFrame.getCircleState());
            	currentStatement.setAbstand_in_out(myMainFrame.getAreaSize());
            }
            else {
            	currentStatement.addInEnv(Statement.PointsToEnv(punkt1, punkt2), myMainFrame.getCircleState());
            	currentStatement.setLowerBoundScale(myMainFrame.getZoomFactor());
            }
            
            
            
            aktualisiereStatement();
            
            
            
            //System.out.println("Statement wurde erstellt!");

            repaint();
        }
    }

    @Override
    public void mouseDragged(MouseEvent e) {
    	if ( myMainFrame.isInZoomPanMode() 
    			|| ( (e.getModifiers() & MouseEvent.BUTTON1_MASK) == 0 && (e.getModifiers() & MouseEvent.BUTTON3_MASK) == 0 ) ) {
    		// Wenn der Zoom-Modus aktiviert ist oder weder Button1 noch Button3 gedrueckt sind. (Damit mit mit der mittleren Maustaste
    		// auch im Fisheye-Modus navigieren kann.
        	Rectangle rec = new Rectangle(visRec);
        	
    		double x = visRec.getLocation().getX();
    		double y = visRec.getLocation().getY();
    		
    		
    		x = x + mouseColumn - e.getX();
    		y = y + mouseRow - e.getY();
        	rec.setLocation((int)x,(int)y);
        	scrollRectToVisible(rec);
        	
        	visRec = rec;
        }
        else if ( myMainFrame.isInScaleOptimizerMode() ) {
            mouseColumnTemp = e.getX();
            mouseRowTemp = e.getY();
            repaint();
       }
    }

    public void optimize() {
        optimize(null, new Statistic());
    }
    
    
    /**
     * Wir ausgefuert sobald ein Rechteck gezogen wurde und der berechnen Button geklickt wurde.
     */
    public void optimize(FishEyeProjection fep,Statistic myStatistic) {
    	//System.out.println("Es wird versucht zu optimieren!");
    	if (this.currentStatement == null)
    		return;
    	if (this.currentStatement.getInEnvs() == null)
    		return;
    	
        
        // TODO
        Envelope eMapExtent = currentStatement.getOutEnv();
        
        
        
        /*double xMapExtent1 = myTransformation.getX(0);
        double xMapExtent2 = myTransformation.getX(this.getSize().width);
        double yMapExtent1 = myTransformation.getY(this.getSize().height);
        double yMapExtent2 = myTransformation.getY(0);
        Envelope eMapExtent = new Envelope(xMapExtent1, xMapExtent2, yMapExtent1, yMapExtent2);
        */
        
        // Punkte die Scale-Werte zuordnen (nur Punkte im InEnvelope)
        HashSet<Point> myPointSet = new HashSet<Point>();
        
        for (Layer l : layers.values()) {
            for (Object o : l.query(currentStatement.getInEnvBoundingBox())) {
                //System.out.println("object found");
                if (o instanceof LineObject) {
                    LineObject line = (LineObject) o;
                    //System.out.println("road found");
                    
                    if (currentStatement.inContains(line.getP1())) {
                        myPointSet.add(line.getP1());
                    }
                    if (currentStatement.inContains(line.getP2())) {
                        myPointSet.add(line.getP2());
                    }
                }
            }
        }
        Object[] myObjArray =  myPointSet.toArray();
        Point[] myPointArray = new Point[myObjArray.length];
        for (int i = 0; i < myObjArray.length; i++) {
            myPointArray[i] = (Point) myObjArray[i];
        }
        
        double[] scaleLB = new double[myPointArray.length];
        double[] scaleUB = new double[myPointArray.length];
        System.out.println("Lower Bound for Scale: " + currentStatement.getLowerBoundScale());
        for (int i = 0; i < scaleLB.length; i++) {
            scaleLB[i] = currentStatement.getLowerBoundScale();
            scaleUB[i] = currentStatement.getLowerBoundScale();
        }
        
        
        RoadNetwork eigenesRoadNetwork = myMapFrame.getRoadNetwork();
        //Envelope eMapExtent = eigenesRoadNetwork.getEnv();
        RoadNetwork myNewRoadNetwork = eigenesRoadNetwork.fit(eMapExtent, currentStatement.getOutestEnv(), myPointArray, 
        		scaleLB, scaleUB, myMainFrame.getGewichtungState(), myMainFrame.getEdgeCrossingState(), myMainFrame.getTriangulationState(), myMainFrame.getOrientierungState(), myMainFrame.getAllOrientationsState(), myMainFrame.getAuswahlState(), myStatistic, fep);
        
        if (myNewRoadNetwork != null && fep == null) {
        	String name = eigenesRoadNetwork.getName() + " - Output (";
        	if(myMainFrame.getGewichtungState())
        		name += "w"; // weight
        	else
        		name += "-";
        	if(myMainFrame.getEdgeCrossingState())
        		name += "e"; // edgeCrossing
        	else
        		name += "-";
        	if(myMainFrame.getTriangulationState())
        		name += "t";
        	else
        		name += "-";        	
        	if(myMainFrame.getOrientierungState())
        		name += "o";
        	else
        		name += "-";   
        	if(myMainFrame.getAuswahlState())
        		name += "s)"; // selection
        	else
        		name += "-)"; 
        	name += " x" + currentStatement.getLowerBoundScale();
        	myNewRoadNetwork.setName(name);

            myMainFrame.addMapFrame(name, myNewRoadNetwork);
        }
        
        this.repaint();
        
    }
    
    
    public void mouseMoved(MouseEvent e) {
        int r = e.getY();
        int c = e.getX();
        myMainFrame.setXY(myTransformation.getX(c), myTransformation.getY(r));
        
    }
    
    public void resetMouseCoordinates() {
        mouseRow = -100;
        mouseRowTemp = -100;
        mouseColumn = -100;
        mouseColumnTemp = -100;
    }

	@Override
	public void mouseWheelMoved(MouseWheelEvent e) {
    	double zoom = 1;
    	if (e.getWheelRotation() < 0) {
    		zoom = myMainFrame.getZoomFactor();
    	} else if (e.getWheelRotation() > 0){
    		zoom = 1/myMainFrame.getZoomFactor(); 		
    	}
    	
    	zoom(zoom, e.getX(), e.getY());
		
	}
	
	// Andert den Faktor ABSTAND_IN_OUT in currentStatement (falls vorhanden) nimmt den Wert vom Slider.
	public void setAreaSize() {
		
		if (this.currentStatement != null) {
			this.currentStatement.setAbstand_in_out(myMainFrame.getAreaSize());
			this.currentStatement.setDefaultOutest();
			repaint();
		}
	}
	
	public void setZoomFactor(double value) {
		if (this.currentStatement != null) {
			this.currentStatement.setLowerBoundScale(value);
		}
	}
	
	public double getAreaSize() {
		if (this.currentStatement != null) {
			return this.currentStatement.getAbstand_in_out();
		}
		return -1.0; // Fehler
	}
	
	public double getZoomFactor() {
		if (this.currentStatement != null) {
			return this.currentStatement.getLowerBoundScale();
		}
		return -1.0; // Fehler
	}

    public void project() {
        Statistic myFepStatistic = new Statistic();
        myFepStatistic.start();
        RoadNetwork eigenesRoadNetwork = myMapFrame.getRoadNetwork();
        Envelope eMapExtent = this.currentStatement.getOutEnv();
        
        Point p  = this.currentStatement.getCenter(); // Das Zentrum der InEnvBoundingBox
        
        double r = this.currentStatement.getRadius(); // Der Radius der InEnvBoundingBox
        double m = this.currentStatement.getLowerBoundScale();
            
        FishEyeProjection fep = new FishEyeProjection(p, eMapExtent, m, r);
        RoadNetwork myNewRoadNetwork = fep.projectRoadNetwork(eigenesRoadNetwork);
        myFepStatistic.totalStop();
        if (myNewRoadNetwork != null) {
            String name = eigenesRoadNetwork.getName() + " - projected";
            myNewRoadNetwork.setName(name);
            myMainFrame.addMapFrame(name, myNewRoadNetwork);
        }    
        this.repaint();      
        optimize(fep, myFepStatistic);
    }

    public void setStatement(Statement st) {
        currentStatement = st;
        myMainFrame.setZoomFactor(st.getLowerBoundScale());
        myMainFrame.setAreaSize(st.getAbstand_in_out());
        aktualisiereStatement();
        repaint();
    }
    
    public void aktualisiereStatement() {
    	if (currentStatement != null) {
			if (myMainFrame.getAreaState()) {
	            currentStatement.setAbstand_in_out(getAreaSize());
	            currentStatement.setDefaultOut();
	            currentStatement.setDefaultOutest();
			}
			else {
				currentStatement.setOutEnv(myMapFrame.getRoadNetwork().getEnv()); 
			}
			currentStatement.setDefaultOutest();
			repaint();
    	}
    }
	
	
}
