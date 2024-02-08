package focusmap;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.GridLayout;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Dictionary;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTabbedPane;
import javax.swing.JToggleButton;
import javax.swing.border.EtchedBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;


/**
 * Das Hauptfenster
 * @author Leon Sering
 *
 */
public class MainFrame extends JFrame {

    // Konstanten
    /** Default serialVersionUID. */
    private static final long serialVersionUID = 1L;   
    /** Fensterbreite. */
    public static final int BREITE = 1000;   
    /** Fensterhoehe. */
    public static final int HOEHE = 800;
    
    // Die drei Hauptpanel:
	private JPanel controlPanel = new JPanel();
	private JPanel infoPanel = new JPanel();
	private JTabbedPane mapPane = new JTabbedPane();
	
    ChangeListener MapPaneListener = new ChangeListener() {
        public void stateChanged(ChangeEvent e) {
          JTabbedPane source = (JTabbedPane) e.getSource();
          MapFrame currentMapFrame = (MapFrame) source.getSelectedComponent();
          MapDisplay currentMapDisplay = currentMapFrame.getMapDisplay();
          // Es muss der ZoomSlider und der areaSlider ans Statement angepasst werden.
          int zoomFactor = (int)(currentMapDisplay.getZoomFactor() * 100);
          if (zoomSlider.getMinimum() <= zoomFactor && zoomFactor <= zoomSlider.getMaximum()) {
        	  zoomSlider.setValue(zoomFactor);
          }
          int areaSize = (int)(currentMapDisplay.getAreaSize() * 100);
          if (areaSlider.getMinimum() <= areaSize && areaSize <= areaSlider.getMaximum()) {
        	  areaSlider.setValue(areaSize);
          }
          currentMapDisplay.aktualisiereStatement();

        }
      };
	
	// controlPanel
	private JButton loadButton;
	
	private ActionListener LoadButtonListener = new ActionListener() {
		public void actionPerformed(ActionEvent e) {
			RoadNetwork rn = RoadNetwork.importFromShapefile();
			if (rn.getName() != "Empty RoadMap")
				addMapFrame(rn.getName(), rn);
		}
	};
	
	private JButton saveButton;
	private JButton strokeButton;
	private JButton parameterButtonImp;
	private JButton parameterButtonExp;
    private JToggleButton moveButton;
    private JToggleButton fisheyeButton;
    
    private JCheckBox EnvAddCheckBox;
    private JCheckBox CircleCheckBox;
    
	private ActionListener ModeButtonListener = new ActionListener() {
		public void actionPerformed(ActionEvent e) {
			MapFrame currentMapFrame = (MapFrame) mapPane.getSelectedComponent();
			if(currentMapFrame != null)
				currentMapFrame.getMapDisplay().repaint();
		}


		
	};
    
    private JSlider zoomSlider;
    
	private ChangeListener ZoomSliderListener = new ChangeListener() {
		public void stateChanged(ChangeEvent e) {
			JSlider source = (JSlider)e.getSource();
			if(!source.getValueIsAdjusting()) {
				MapFrame currentMapFrame = (MapFrame) mapPane.getSelectedComponent();
				currentMapFrame.getMapDisplay().setZoomFactor(source.getValue()/100.0);
			}
		}
	};
    
	private JCheckBox areaCheckBox;
	
	private ActionListener areaCheckBoxListener = new ActionListener() {
		public void actionPerformed(ActionEvent e) {
			MapFrame selectedMapFrame = (MapFrame) mapPane.getSelectedComponent();
			if (areaCheckBox.isSelected()) {
				areaSlider.setEnabled(true);
			}
			else {
				areaSlider.setEnabled(false);
			}
			selectedMapFrame.getMapDisplay().aktualisiereStatement();
		}
	};
	
    private JSlider areaSlider;
    
	private ChangeListener AreaSliderListener = new ChangeListener() {
		public void stateChanged(ChangeEvent e) {
			JSlider source = (JSlider)e.getSource();
			if(!source.getValueIsAdjusting()) {
				MapFrame currentMapFrame = (MapFrame) mapPane.getSelectedComponent();
				currentMapFrame.getMapDisplay().setAreaSize();
			}
		}
	};
	private JCheckBox ViewTriangulationCheckBox;
	
	private ActionListener ViewTriangulationListener = new ActionListener() {
		public void actionPerformed(ActionEvent e) {
			repaint();
		}
	};
	
	private JCheckBox GewichtungCheckBox;
	private JCheckBox EdgeCrossingCheckBox;
	private JCheckBox TriangulationCheckBox;
	private JCheckBox OrientierungCheckBox;
	private JCheckBox AllOrientationsCheckBox;
	private JCheckBox AuswahlCheckBox;
    
    private JButton berechnenButton;
    
	private ActionListener berechnenButtonListener = new ActionListener() {
		public void actionPerformed(ActionEvent e){
			System.out.println("Berechnenbutton geklickt!");
			MapFrame selectedMapFrame = (MapFrame) mapPane.getSelectedComponent();
			selectedMapFrame.berechnen();
		}
	};
	
    private JButton projectButton;
    
    private ActionListener projectListener = new ActionListener() {
        public void actionPerformed(ActionEvent e) {
            System.out.println("Projectbutton geklickt");
            MapFrame selectedMapFrame = (MapFrame) mapPane.getSelectedComponent();
            selectedMapFrame.project();
        }
    };
    
    private ActionListener parameterImpListener = new ActionListener() {
        public void actionPerformed(ActionEvent e) {
            System.out.println("Button fuer Parameterimport geklickt");
            MapFrame selectedMapFrame = (MapFrame) mapPane.getSelectedComponent();
            selectedMapFrame.importParameter();
        }
    };
    
    private ActionListener parameterExpListener = new ActionListener() {
        public void actionPerformed(ActionEvent e) {
            System.out.println("Button fuer Parameterexport geklickt");
            MapFrame selectedMapFrame = (MapFrame) mapPane.getSelectedComponent();
            selectedMapFrame.exportParameter();
        }
    };
    
    
    // infoPanel
    private JLabel xLabel;
    private JLabel yLabel;
	
	@SuppressWarnings("unchecked")
	private void initControlPanel() {
		
		int breite = 180;

		
		controlPanel.setPreferredSize(new Dimension(breite, 0));
		controlPanel.setLayout(new BorderLayout(5, 5));
		
		JPanel hilfsPanel = new JPanel(); // Alle Elemente kommen ins hilfspanel, diese befindet sich oben vom Controlpanel
		
		//hilfsPanel.setLayout(new GridLayout(0, 1));
		hilfsPanel.setLayout(new BoxLayout(hilfsPanel, BoxLayout.PAGE_AXIS));
		controlPanel.add(hilfsPanel, BorderLayout.PAGE_START);

        //filePanel
		JPanel filePanel = new JPanel();
        filePanel.setLayout(new GridLayout(0, 1, 6, 3));
        filePanel.setBorder(BorderFactory.createTitledBorder(new EtchedBorder(), "File", 0, 0));
        
        loadButton = new JButton("Load Shapefile");
        loadButton.addActionListener(LoadButtonListener);
        
        saveButton = new JButton("Save to Shapefile");
        saveButton.addActionListener(new SaveButtonListener(mapPane));
        
        filePanel.add(loadButton);
        filePanel.add(saveButton);
        
        strokeButton = new JButton("Build Strokes");
        strokeButton.addActionListener(new StrokeButtonListener(mapPane, this));
                
        projectButton = new JButton("Project");
        projectButton.addActionListener(projectListener);
        
        hilfsPanel.add(filePanel);
        
        // modePanel
        JPanel modePanel = new JPanel();
        
        modePanel.setBorder(BorderFactory.createTitledBorder(new EtchedBorder(), "Selected Mode", 0, 0));
        modePanel.setLayout(new GridLayout(0, 1, 6, 3));
        
        ButtonGroup modeButtonGroup = new ButtonGroup();
        
        moveButton = new JToggleButton("Zoom & Pan");
        fisheyeButton = new JToggleButton("Fisheye Projection");
        
        moveButton.addActionListener(ModeButtonListener);
        fisheyeButton.addActionListener(ModeButtonListener);
        
        EnvAddCheckBox = new JCheckBox("Add Envelopes");
        EnvAddCheckBox.setSelected(false);
        
        CircleCheckBox = new JCheckBox("Circle");
        CircleCheckBox.setSelected(true);
        
        modeButtonGroup.add(moveButton);
        modeButtonGroup.add(fisheyeButton);
        
        modePanel.add(moveButton);
        modePanel.add(fisheyeButton);
        modePanel.add(EnvAddCheckBox);
        modePanel.add(CircleCheckBox);
            
        fisheyeButton.doClick();	
        
        hilfsPanel.add(modePanel);

        
        
        // viewPanel
        JPanel viewPanel = new JPanel();
        viewPanel.setLayout(new GridLayout(0, 1, 6, 3));
        viewPanel.setBorder(BorderFactory.createTitledBorder(new EtchedBorder(), "View", 0, 0));
        
        //ViewTriangulation-CheckBox
		ViewTriangulationCheckBox = new JCheckBox("Triangulation");
		ViewTriangulationCheckBox.setSelected(false);
        viewPanel.add(ViewTriangulationCheckBox);
        ViewTriangulationCheckBox.addActionListener(ViewTriangulationListener);
        
        hilfsPanel.add(viewPanel);
        
        // zoomPanel
        JPanel zoomPanel = new JPanel();
        
        zoomPanel.setBorder(BorderFactory.createTitledBorder(new EtchedBorder(), "Zoom Factor", 0, 0));
		
		zoomSlider = new JSlider(JSlider.HORIZONTAL, 100, 1000, 200);
        zoomSlider.setPaintTicks(true);
        zoomSlider.setMajorTickSpacing(100);
        zoomSlider.setPaintLabels(true);
        zoomSlider.setPreferredSize(new Dimension(breite - 20, 50));
        
        Dictionary<Integer, JLabel> d = zoomSlider.getLabelTable();
        for (int i = 1; i <= 11; i++) {
            d.put(new Integer(i * 100), new JLabel("" + i));
        }
        zoomSlider.setLabelTable(d);
        
        zoomSlider.addChangeListener(ZoomSliderListener);
        
        zoomPanel.add(zoomSlider);
        
        hilfsPanel.add(zoomPanel);
               
        
        
        // areaPanel
        JPanel areaPanel = new JPanel();
        
        areaPanel.setBorder(BorderFactory.createTitledBorder(new EtchedBorder(), "Area Size", 0, 0));
		areaPanel.setLayout(new BorderLayout());
        
        areaCheckBox = new JCheckBox();
        areaPanel.add(areaCheckBox, BorderLayout.LINE_START);
        
        areaCheckBox.addActionListener(areaCheckBoxListener);
        
        areaCheckBox.setSelected(false);
        
		areaSlider = new JSlider(JSlider.HORIZONTAL, 0, 2000, 400);
		
		areaSlider.setEnabled(false);
		
        areaSlider.setPaintTicks(true);
        areaSlider.setMajorTickSpacing(400);
        areaSlider.setPaintLabels(true);
        areaSlider.setPreferredSize(new Dimension(breite - 40, 50));
        
        d = areaSlider.getLabelTable();
        for (int i = 0; i <= 5; i++) {
            d.put(new Integer(i * 400), new JLabel("" + 2*i));
        }
        areaSlider.setLabelTable(d);
        
        areaSlider.addChangeListener(AreaSliderListener);
        
        areaPanel.add(areaSlider, BorderLayout.LINE_END);
        
        hilfsPanel.add(areaPanel);
        
        
        //optionsPanel
		JPanel optionsPanel = new JPanel();
		optionsPanel.setLayout(new GridLayout(0, 1, 6, 3));
		optionsPanel.setBorder(BorderFactory.createTitledBorder(new EtchedBorder(), "Optionen", 0, 0));
 
		
		
		
        //Gewicht-CheckBox
		GewichtungCheckBox = new JCheckBox("Use Weights");
		GewichtungCheckBox.setSelected(true);
        optionsPanel.add(GewichtungCheckBox);
        
      //Gewicht-CheckBox
		EdgeCrossingCheckBox = new JCheckBox("Avoid Edge-Crossing");
		EdgeCrossingCheckBox.setSelected(false);
        optionsPanel.add(EdgeCrossingCheckBox);
		
        // Triangulations-CheckBox
        TriangulationCheckBox = new JCheckBox("Treat Triangles as Roads");
        TriangulationCheckBox.setSelected(false);
        optionsPanel.add(TriangulationCheckBox);
        
        JPanel orientationPanel = new JPanel();
		orientationPanel.setLayout(new BorderLayout());
        // Orientierungs-CheckBox
        OrientierungCheckBox = new JCheckBox("Orientation");
        OrientierungCheckBox.setSelected(true);
        orientationPanel.add(OrientierungCheckBox,BorderLayout.LINE_START);
        
     // Orientierungs-CheckBox
        AllOrientationsCheckBox = new JCheckBox("all");
        AllOrientationsCheckBox.setSelected(false);
        orientationPanel.add(AllOrientationsCheckBox,BorderLayout.LINE_END);
        
        optionsPanel.add(orientationPanel);
        
        // Auswahl-CheckBox
        AuswahlCheckBox = new JCheckBox("Optimize Selection");
        AuswahlCheckBox.setSelected(false);
        optionsPanel.add(AuswahlCheckBox);
        
        hilfsPanel.add(optionsPanel);
        
        // panel for import and export of parameters defining the focus region
        JPanel parametersPanel = new JPanel();
        parametersPanel.setLayout(new GridLayout(0, 1, 6, 3));
        parametersPanel.setBorder(BorderFactory.createTitledBorder(new EtchedBorder(), "Parameters", 0, 0));
        parameterButtonImp = new JButton("Import Parameters");
        parameterButtonImp.addActionListener(parameterImpListener);
        parametersPanel.add(parameterButtonImp);
        parameterButtonExp = new JButton("Export Parameters");
        parameterButtonExp.addActionListener(parameterExpListener);
        parametersPanel.add(parameterButtonExp);
        hilfsPanel.add(parametersPanel);

        

        berechnenButton = new JButton("Compute");
        berechnenButton.addActionListener(berechnenButtonListener);
        
        
        JPanel manipulatePanel = new JPanel();
        
        manipulatePanel.setLayout(new GridLayout(0, 1, 6, 3));
        manipulatePanel.setBorder(BorderFactory.createTitledBorder(new EtchedBorder(), "Map Manipulation", 0, 0));
        manipulatePanel.add(strokeButton);
        manipulatePanel.add(projectButton);
        manipulatePanel.add(berechnenButton);
        controlPanel.add(manipulatePanel, BorderLayout.PAGE_END);
        
        
	}
	
	private void initInfoPanel() {
		
		infoPanel.setPreferredSize(new Dimension(0, 20));
		infoPanel.setLayout(new BorderLayout(5, 5));
		
		JPanel hilfsPanel = new JPanel(); // Alle Elemente kommen ins hilfspanel, diese befindet sich oben vom Controlpanel
		
		hilfsPanel.setLayout(new GridLayout(1, 0));
		infoPanel.add(hilfsPanel, BorderLayout.LINE_START);
		
        // Position
        JPanel positionsPanel = new JPanel();
        positionsPanel.setLayout(new FlowLayout());
        
        JLabel positionLabel = new JLabel("Position: ");
        xLabel = new JLabel(" x = 0 ");
        yLabel = new JLabel(" y = 0 ");
        
        positionsPanel.add(positionLabel);
        positionsPanel.add(xLabel);
        positionsPanel.add(yLabel);
        
        hilfsPanel.add(positionsPanel);
        
	}
	
	
	public MainFrame(String firstRNTitle, RoadNetwork firstRN) {
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(BREITE, HOEHE);
        
        // Fenster in die Mitte des Bildschirms setzen:
        Dimension d = Toolkit.getDefaultToolkit().getScreenSize(); 
        setLocation((d.width - getSize().width) / 2, 
                     (d.height - getSize().height) / 2);
        
        setTitle("FocusMap");
        
        setLayout(new BorderLayout(5, 5));
        
        // Drei Hauptpanels setzen.
        add(controlPanel, BorderLayout.LINE_START);
        add(infoPanel, BorderLayout.PAGE_END);
        add(mapPane, BorderLayout.CENTER);
        
        mapPane.addChangeListener(MapPaneListener);
        
        initControlPanel();
        initInfoPanel();
       
        addMapFrame(firstRNTitle, firstRN);
             
	}
	
    public boolean isInZoomPanMode() {
        return moveButton.isSelected();
    }
    
    public boolean isInScaleOptimizerMode() {
        return fisheyeButton.isSelected();
    }
    
    public double getZoomFactor() {
        return ((double) zoomSlider.getValue()) / 100.0;
    }
    
    public void setZoomFactor(double zoomFactor) {
    	this.zoomSlider.setValue((int)(zoomFactor*100));
    }
    
    public void setAreaSize(double areaSize) {
    	this.areaSlider.setValue((int)(areaSize*100));
    }
    
    public double getAreaSize() {
    	return ((double) areaSlider.getValue()) / 200.0;
    }
    
    public boolean getViewTriangulationState() {
    	return ViewTriangulationCheckBox.isSelected();
    }
    
    public boolean getGewichtungState() {
    	return GewichtungCheckBox.isSelected();
    }
    
    public boolean getEdgeCrossingState() {
    	return EdgeCrossingCheckBox.isSelected();
    }
    
    public boolean getTriangulationState() {
    	return TriangulationCheckBox.isSelected();
    }
    
    public boolean getOrientierungState() {
    	return OrientierungCheckBox.isSelected();
    }
    
    public boolean getAllOrientationsState() {
    	return AllOrientationsCheckBox.isSelected();
    }
    
    public boolean getAuswahlState() {
    	return AuswahlCheckBox.isSelected();
    }
    
    public boolean getAreaState() {
    	return areaCheckBox.isSelected();
    }
    
    public boolean getEnvAddState() {
    	return EnvAddCheckBox.isSelected();
    }
    
    public boolean getCircleState() {
    	return CircleCheckBox.isSelected();
    }
    
    
    public void setXY(double x, double y) {
        xLabel.setText(" x = " + x);
        xLabel.repaint();
        yLabel.setText(" y = " + y);
        yLabel.repaint();
        
    }
    
    
    
    
    public void addMapFrame(String title, RoadNetwork rn) {
	    MapFrame myNewMapFrame = new MapFrame(this, rn);
	    mapPane.setSelectedComponent(mapPane.add(title, myNewMapFrame));
	    mapPane.setTabComponentAt(mapPane.getTabCount()-1, new ButtonTabComponent(mapPane));
    }
    
    public MapFrame getSelectedMapFrame() {
    	return (MapFrame) mapPane.getSelectedComponent();
    }
}
